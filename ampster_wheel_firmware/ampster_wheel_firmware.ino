#include <ESP32Servo.h>
#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_VL53L0X tofSensor = Adafruit_VL53L0X();
Servo treatServo;

// Pin Definitions
const int SERVO_PIN = 18;        // PWM pin for FS90R servo (D18, GPIO02 or A1)
const int NEOPIXEL_PIN = 17;     // Data pin for NeoPixel LED strip (D8, GPIO17)
const int BUTTON_PIN = 2;        // Digital pin for momentary push button (D2, GPIO2)
// Stepper Motor (17HS4023 Nema 17) is used for power generation (see documentation for wiring)

// Constants
const unsigned long DEBOUNCE_DELAY = 50;  // Debounce time in milliseconds
const int ROTATION_THRESHOLD = 75;       // Distance threshold for detecting rotation

// NeoPixel LED strip
#define NUM_PIXELS 40
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
bool ledStripEnabled = true;

// OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayEnabled = true;

// Global Variables
unsigned long lastButtonDebounce = 0;        // For debouncing button
bool lastButtonState = HIGH;                 // Last button state
bool buttonState = HIGH;                     // Current button state
unsigned long buttonPressTime = 0;           // Time when button was pressed
bool ledActive = true;                      // LED strip state
bool displayActive = true;                  // Display state

// --- AmpsterWheel Stats ---
#define WHEEL_DIAMETER_INCHES 12.0
#define WHEEL_CIRCUMFERENCE_MILES ((WHEEL_DIAMETER_INCHES * 3.14159265359) / 63360.0)
#define WHEEL_CIRCUMFERENCE_METERS (WHEEL_DIAMETER_INCHES * 0.0254 * 3.14159265359)
#define STEPPER_PHASES 2
#define STEPPER_PHASE_RESISTANCE 3.5
#define STEPPER_EFFICIENCY 0.65
#define AVERAGE_VOLTAGE 3.5
#define CIRCUIT_RESISTANCE (STEPPER_PHASE_RESISTANCE * STEPPER_PHASES)

unsigned long rotationCount = 0;
unsigned long totalRotations = 0;
unsigned long treatThreshold = 100;
unsigned long lastRotationMillis = 0;
double distanceMiles = 0.0;
double energyWattHours = 0.0;
double currentSpeedMPH = 0.0;
double avgSpeedMPH = 0.0;
double peakSpeedMPH = 0.0;
double peakPowerWatts = 0.0;
double rotationTimes[10] = {0};
int rotationTimesIndex = 0;
double sessionStartMillis = 0;
double activeTime = 0.0;

// Serial Communication
const unsigned long SERIAL_BAUD = 9600;
const unsigned long REPORT_INTERVAL = 100;  // Report every 100ms
unsigned long lastReport = 0;

unsigned long lastRotationDetectedTime = 0;

// Variables for LED dimming
static unsigned long lastMovementTime = 0;
static bool isResting = false;
static int ledBrightness = 5; // Initial brightness

void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD);
  
  // Configure pins
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Configure button pin with internal pull-up
  pinMode(SERVO_PIN, OUTPUT); // Initialize servo pin as output
  // No need to pinMode for NeoPixel (library handles it)
  
  // Initialize servo
  treatServo.attach(SERVO_PIN);
  treatServo.writeMicroseconds(1500);  // Stop position for continuous rotation servo
  
  // Initialize NeoPixel strip
  pixels.begin();
  pixels.setBrightness(5); // 0-255
  pixels.show(); // Initialize all pixels to 'off'

  // Initialize OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    displayEnabled = false;
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(F("AmpsterWheel Boot"));
    display.display();
    delay(200);
    display.clearDisplay();
    display.display();
  }

  // Initialize VL53L0X sensor
  Wire.begin();
  if (!tofSensor.begin()) {
    Serial.println("Failed to boot VL53L0X");
    while (1);
  }
}

void handleSerialCommands() {
  // Handle incoming serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "LED_ON") {
      ledActive = true;
    } else if (command == "LED_OFF") {
      ledActive = false;
    } else if (command == "SERVO_FORWARD") {
      treatServo.writeMicroseconds(1700);  // Forward motion
    } else if (command == "SERVO_BACKWARD") {
      treatServo.writeMicroseconds(1300);  // Backward motion
    } else if (command == "SERVO_STOP") {
      treatServo.writeMicroseconds(1500);  // Stop motion
    }
  }
}

void dispenseTreat() {
  // Run servo backward slowly for 1.5 seconds
  treatServo.writeMicroseconds(1300); // Reverse
  delay(1500);
  treatServo.writeMicroseconds(1500); // Stop
}

void ledCelebration() {
  // Animate NeoPixel strip with random colors for 4 seconds
  for (int t = 0; t < 40; t++) {
    for (int i = 0; i < NUM_PIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(random(0,256), random(0,256), random(0,256)));
    }
    pixels.show();
    delay(100);
  }
  // Turn off LEDs after celebration
  pixels.clear();
  pixels.show();
}

void loop() {
  // Check for rodent rest
  unsigned long now = millis();
  if (now - lastRotationDetectedTime > 15000) { // 15 seconds of inactivity
    if (!isResting) {
      isResting = true;
      currentSpeedMPH = 0.0; // Set speed to 0
      displayActive = false; // Enter rest mode by turning off the display
      ledActive = false; // Turn off LEDs
      pixels.clear();
      pixels.show();
    }
  } else {
    if (isResting) {
      isResting = false;
      displayActive = true; // Wake up the display
      ledActive = true; // Turn on LEDs
    }
  }

  // Handle button state
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) {
    lastButtonDebounce = millis();
  }
  if ((millis() - lastButtonDebounce) > DEBOUNCE_DELAY) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {  // Button pressed
        buttonPressTime = millis();
        // Wake up system from rest mode
        if (isResting) {
          isResting = false;
          displayActive = true;
          ledActive = true;
          ledBrightness = 15; // Reset LED brightness
        }
      } else if (buttonState == HIGH) {  // Button released
        unsigned long pressDuration = millis() - buttonPressTime;
        if (pressDuration < 1000) {  // Short press
          displayActive = !displayActive;  // Toggle display
        } else if (pressDuration >= 8000) {  // Very long press (8 seconds)
          if (!displayActive) {
            displayActive = true;  // Ensure display is active
          }
          display.clearDisplay();
          display.setCursor(0,0);
          display.setTextSize(1);
          display.setTextColor(SSD1306_WHITE);
          display.println("Restarting...");
          display.display();
          delay(3000);  // Display message for 3 seconds
          ESP.restart();  // Restart ESP32
        }
      }
    }
  }
  lastButtonState = reading;
  
  // Handle any incoming serial commands
  handleSerialCommands();
  
  // --- Rotation Detection and Stats ---
  VL53L0X_RangingMeasurementData_t measure;
  tofSensor.rangingTest(&measure, false);
  static bool lastRotationDetected = false;
  static unsigned long lastRotationTime = 0;
  static int lastMeasuredDistance = 0;
  if (measure.RangeStatus != 4) {
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    // Check if the wheel has moved a significant distance since the last measurement
    if (measure.RangeMilliMeter < ROTATION_THRESHOLD && abs(measure.RangeMilliMeter - lastMeasuredDistance) > 20) {
      // Wake up system from rest mode if a new rotation is detected
      if (isResting) {
        isResting = false;
        displayActive = true;
        ledActive = true;
        ledBrightness = 15; // Reset LED brightness
      }
      if (!lastRotationDetected) {
        // New rotation event
        rotationCount++;
        totalRotations++;
        unsigned long timeDiff = now - lastRotationTime;
        lastRotationTime = now;
        if (timeDiff > 0) {
          double timeHours = timeDiff / 3600000.0;
          currentSpeedMPH = WHEEL_CIRCUMFERENCE_MILES / timeHours;
          distanceMiles += WHEEL_CIRCUMFERENCE_MILES;
          if (currentSpeedMPH > peakSpeedMPH) peakSpeedMPH = currentSpeedMPH;
          // Store last 10 rotation times
          rotationTimes[rotationTimesIndex] = timeDiff / 1000.0;
          rotationTimesIndex = (rotationTimesIndex + 1) % 10;
          double sum = 0.0; int n = 0;
          for (int i = 0; i < 10; i++) if (rotationTimes[i] > 0) { sum += rotationTimes[i]; n++; }
          if (n > 0) avgSpeedMPH = distanceMiles / ((activeTime + sum) / 3600.0);
        }
        // Calculate power
        double currentPower = (AVERAGE_VOLTAGE * AVERAGE_VOLTAGE * STEPPER_EFFICIENCY) / CIRCUIT_RESISTANCE;
        if (currentPower > peakPowerWatts) peakPowerWatts = currentPower;
        // Energy
        energyWattHours += (currentPower * (timeDiff / 1000.0)) / 3600.0; // Convert to hours
        activeTime += timeDiff / 1000.0;
        // Placeholder: treat dispensing logic
        if (rotationCount >= treatThreshold) {
          // Turn all LEDs green before dispensing treat
          for (int i = 0; i < NUM_PIXELS; i++) {
            pixels.setPixelColor(i, pixels.Color(0, 255, 0));
          }
          pixels.show();
          dispenseTreat();
          ledCelebration();
          rotationCount = 0;
        }
      }
      lastRotationDetected = true;
      lastRotationDetectedTime = now; // Update the last rotation detected time
    } else {
      lastRotationDetected = false;
    }
    lastMeasuredDistance = measure.RangeMilliMeter;
  } else {
    Serial.println(" out of range ");
  }

  // Gradually dim LEDs before turning off
  if (isResting && ledBrightness > 0) {
    if (now - lastMovementTime > 150) { // Adjust time for smooth dimming over 3 seconds
      ledBrightness--;
      lastMovementTime = now;
    }
  } else if (!isResting && ledBrightness < 20) {
    if (now - lastMovementTime > 150) { // Adjust time for smooth brightening
      ledBrightness++;
      lastMovementTime = now;
    }
  }
  pixels.setBrightness(ledBrightness);
  pixels.show();

  // Ensure the servo is stopped when not dispensing treats
  treatServo.writeMicroseconds(1500); // Stop position

  // --- OLED Display Update ---
  if (displayEnabled && displayActive) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(1);
    display.print("Dist: "); display.print(distanceMiles, 3); display.println(" mi");
    display.print("Speed: "); display.print(currentSpeedMPH, 2); display.println(" mph");
    display.print("Peak: "); display.print(peakSpeedMPH, 2); display.println(" mph");
    display.print("Energy: "); display.print(energyWattHours, 2); display.println(" Wh");
    
    // Progress label, bar, and rotation count
    display.print("Progress: ");
    display.print(rotationCount);
    int barWidth = map(rotationCount, 0, treatThreshold, 0, SCREEN_WIDTH);
    display.fillRect(0, SCREEN_HEIGHT - 8, barWidth, 8, SSD1306_WHITE);
    display.drawRect(0, SCREEN_HEIGHT - 8, SCREEN_WIDTH, 8, SSD1306_WHITE);
    
    display.display();
  } else if (!displayActive) {
    display.clearDisplay();
    display.display();
  }

  // --- NeoPixel Smooth Progress Animation ---
  if (ledStripEnabled && ledActive) {
    int progress = (rotationCount * NUM_PIXELS) / treatThreshold;
    for (int i = 0; i < NUM_PIXELS / 2; i++) {
      if (i < progress / 2 || progress >= NUM_PIXELS) {
        for (int brightness = 0; brightness <= 255; brightness += 5) {
          pixels.setPixelColor(i, pixels.Color(0, brightness, 0));
          pixels.setPixelColor(NUM_PIXELS - 1 - i, pixels.Color(0, brightness, 0));
        }
      } else {
        pixels.setPixelColor(i, pixels.Color(0, 0, 0));
        pixels.setPixelColor(NUM_PIXELS - 1 - i, pixels.Color(0, 0, 0));
      }
    }
    pixels.show(); // Only call show once after setting all pixels
  } else if (ledStripEnabled && !ledActive) {
    if (pixels.getBrightness() != 0) {
      pixels.clear();
      pixels.show();
    }
  }

  // Report status at intervals
  if (millis() - lastReport >= REPORT_INTERVAL) {
    lastReport = millis();
    Serial.print("LED State: ");
    Serial.print(ledActive ? "ON" : "OFF");
    Serial.print(", Display State: ");
    Serial.print(displayActive ? "Active" : "Inactive");
    Serial.print(", Rotations: ");
    Serial.print(rotationCount);
    Serial.print(", Speed: ");
    Serial.print(currentSpeedMPH, 2);
    Serial.println(" mph");
  }
}
