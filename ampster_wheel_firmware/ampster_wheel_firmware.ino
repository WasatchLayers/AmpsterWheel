#include <Servo.h>

// Pin Definitions
const int HALL_SENSOR_PIN = 2;  // Digital pin for Hall sensor
const int SERVO_PIN = 9;        // PWM pin for servo
const int LED_PIN = 6;          // Data pin for LED strip
const int BUTTON_PIN = 3;       // Digital pin for LED toggle button
const int VOLTAGE_SENSE_PIN = A0; // Analog pin for voltage monitoring

// Constants
const unsigned long DEBOUNCE_DELAY = 50;  // Debounce time in milliseconds
const int VOLTAGE_SAMPLES = 10;           // Number of samples for voltage averaging

// Global Variables
Servo treatServo;
volatile unsigned long lastHallTrigger = 0;  // For debouncing Hall sensor
volatile bool hallState = HIGH;              // Current Hall sensor state
unsigned long lastButtonDebounce = 0;        // For debouncing button
bool lastButtonState = HIGH;                 // Last button state
bool buttonState = HIGH;                     // Current button state

// Serial Communication
const unsigned long SERIAL_BAUD = 9600;
const unsigned long REPORT_INTERVAL = 100;  // Report every 100ms
unsigned long lastReport = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD);
  
  // Configure pins
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(VOLTAGE_SENSE_PIN, INPUT);
  
  // Initialize servo
  treatServo.attach(SERVO_PIN);
  treatServo.write(0);  // Set to initial position
  
  // Attach interrupt for Hall sensor
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, CHANGE);
}

// Hall sensor interrupt service routine
void hallSensorISR() {
  unsigned long currentTime = millis();
  // Debounce
  if (currentTime - lastHallTrigger > DEBOUNCE_DELAY) {
    hallState = digitalRead(HALL_SENSOR_PIN);
    lastHallTrigger = currentTime;
    // Send hall sensor state change to Python
    Serial.print("H:");
    Serial.println(hallState);
  }
}

// Read and average voltage from the generator
float readVoltage() {
  long sum = 0;
  for(int i = 0; i < VOLTAGE_SAMPLES; i++) {
    sum += analogRead(VOLTAGE_SENSE_PIN);
    delay(1);
  }
  float voltage = (sum / VOLTAGE_SAMPLES) * (5.0 / 1023.0);
  return voltage;
}

void handleButton() {
  int reading = digitalRead(BUTTON_PIN);
  unsigned long currentTime = millis();
  
  if (reading != lastButtonState) {
    lastButtonDebounce = currentTime;
  }
  
  if ((currentTime - lastButtonDebounce) > DEBOUNCE_DELAY) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {  // Button pressed
        Serial.println("B:TOGGLE");  // Send button press event to Python
      }
    }
  }
  
  lastButtonState = reading;
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("SERVO:")) {
      int angle = command.substring(6).toInt();
      treatServo.write(angle);
    }
  }
}

void loop() {
  // Handle button input
  handleButton();
  
  // Handle any incoming serial commands
  handleSerialCommands();
  
  // Report voltage periodically
  unsigned long currentTime = millis();
  if (currentTime - lastReport >= REPORT_INTERVAL) {
    float voltage = readVoltage();
    Serial.print("V:");
    Serial.println(voltage);
    lastReport = currentTime;
  }
}
