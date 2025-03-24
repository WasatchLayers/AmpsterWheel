import serial
import time
import math
from neopixel import NeoPixel
import board
import digitalio
import busio
import adafruit_ssd1306
from adafruit_motor import servo

class RodentWheel:
    # Constants for calculations
    WHEEL_DIAMETER_INCHES = 12.0  # 12" diameter wheel
    WHEEL_CIRCUMFERENCE_MILES = (WHEEL_DIAMETER_INCHES * math.pi) / 63360  # Convert to miles
    WHEEL_CIRCUMFERENCE_METERS = (WHEEL_DIAMETER_INCHES * 0.0254 * math.pi)  # Convert to meters
    
    # Stepper motor specifications - 17HS4023 Nema 17
    STEPPER_PHASE_RESISTANCE = 4.0  # Ohms per phase (from specs: 4 ± 10% Ω/Phase)
    STEPPER_PHASE_INDUCTANCE = 3.2  # mH per phase (from specs: 3.2 ± 20% mH/Phase)
    STEPPER_PHASES = 2  # Bipolar stepper has 2 phases
    STEPPER_RATED_CURRENT = 0.7  # Amps per phase (from specs: 700 mA/phase)
    STEPPER_RATED_VOLTAGE = 12.0  # Volts (from specs: 12V)
    STEPPER_STEP_ANGLE = 1.8  # Degrees per step (from specs: 1.8 ± 5% °/Step)
    STEPPER_STEPS_PER_REV = 200  # 360 / 1.8 = 200 steps per revolution
    STEPPER_EFFICIENCY = 0.65  # Typical efficiency when used as generator (conservative estimate)
    
    # Power calculation constants
    AVERAGE_VOLTAGE = 3.5  # Average voltage when active
    CIRCUIT_RESISTANCE = STEPPER_PHASE_RESISTANCE * STEPPER_PHASES  # Total resistance
    
    # OLED Display specifications
    OLED_WIDTH = 128
    OLED_HEIGHT = 64
    OLED_ADDRESS = 0x3C  # 7-bit I2C address for SSD1306
    
    def __init__(self, arduino_port='COM3', baud_rate=9600):
        # Serial connection to Arduino
        self.arduino = serial.Serial(arduino_port, baud_rate)
        time.sleep(2)  # Wait for Arduino connection
        
        # Initialize Hall sensor
        self.hall_sensor = digitalio.DigitalInOut(board.D2)
        self.hall_sensor.direction = digitalio.Direction.INPUT
        self.hall_sensor.pull = digitalio.Pull.UP
        
        # Initialize LED strip
        pixel_pin = board.D6
        num_pixels = 30
        self.pixels = NeoPixel(pixel_pin, num_pixels, brightness=0.3, auto_write=False)
        
        # Initialize continuous rotation servo for treat dispenser (FS90R)
        self.pwm = board.D9
        self.servo = servo.ContinuousServo(self.pwm)
        self.servo_speed = 0  # Range: -100 to 100
        
        # Initialize OLED display
        i2c = busio.I2C(board.SCL, board.SDA)
        self.display = adafruit_ssd1306.SSD1306_I2C(self.OLED_WIDTH, self.OLED_HEIGHT, i2c, addr=self.OLED_ADDRESS)
        
        # Initialize battery monitoring pins
        self.lbat_pin = digitalio.DigitalInOut(board.D3)
        self.lbat_pin.direction = digitalio.Direction.INPUT
        
        self.pld_pin = digitalio.DigitalInOut(board.D4)
        self.pld_pin.direction = digitalio.Direction.INPUT
        
        # Initialize variables
        self.rotation_count = 0
        self.total_rotations = 0  # Track total rotations for energy calculation
        self.last_hall_state = True
        self.treat_threshold = 100  # Number of rotations before treat
        self.led_enabled = True
        
        # Initialize stats
        self.distance_miles = 0.0
        self.energy_watthours = 0.0
        self.last_rotation_time = time.time()
        self.is_active = False
        self.active_time = 0.0  # Time spent running in seconds
        self.inactivity_timer = 0  # Timer to track inactivity even if magnet is detected
        
        # Additional stats
        self.session_start_time = time.time()
        self.current_speed_mph = 0.0
        self.avg_speed_mph = 0.0
        self.peak_speed_mph = 0.0
        self.peak_power_watts = 0.0
        self.rotation_times = []  # Store last 10 rotation times for speed calculation
        self.last_button_press = time.time()
    
    def update_hall_sensor(self):
        """Read Hall sensor and update rotation count and stats"""
        current_state = self.hall_sensor.value
        current_time = time.time()
        
        # Check for inactivity regardless of hall sensor state
        time_since_last_rotation = current_time - self.last_rotation_time
        if time_since_last_rotation > 3.0:  # If no rotation for 3 seconds
            self.is_active = False
            self.current_speed_mph = 0.0
            
            # If hall sensor is still detecting magnet after timeout, consider it a new baseline
            if not current_state and time_since_last_rotation > 10.0:
                self.last_hall_state = current_state  # Reset baseline to prevent false triggers
        
        if current_state != self.last_hall_state:
            if current_state:  # Rising edge
                self.rotation_count += 1
                self.total_rotations += 1
                
                # Update distance
                self.distance_miles += self.WHEEL_CIRCUMFERENCE_MILES
                
                # Calculate time since last rotation
                time_diff = current_time - self.last_rotation_time
                
                # Store rotation times (keep last 10)
                self.rotation_times.append(time_diff)
                if len(self.rotation_times) > 10:
                    self.rotation_times.pop(0)
                
                # Calculate current speed
                if time_diff > 0 and time_diff < 2.0:  # Ignore very slow or stopped wheel
                    # Speed = distance / time (mph)
                    self.current_speed_mph = self.WHEEL_CIRCUMFERENCE_MILES / time_diff * 3600
                    
                    # Update peak speed
                    if self.current_speed_mph > self.peak_speed_mph:
                        self.peak_speed_mph = self.current_speed_mph
                    
                    # Calculate average speed
                    if self.active_time > 0:
                        self.avg_speed_mph = self.distance_miles / (self.active_time / 3600)
                
                # Consider wheel active if rotation occurs within 2 seconds
                if time_diff < 2.0:
                    if not self.is_active:
                        self.is_active = True
                    self.active_time += time_diff
                    
                    # Calculate power based on stepper motor characteristics
                    # P = V² * efficiency / (phase resistance * number of phases)
                    current_power = (self.AVERAGE_VOLTAGE ** 2) * self.STEPPER_EFFICIENCY / self.CIRCUIT_RESISTANCE
                    
                    # Update peak power
                    if current_power > self.peak_power_watts:
                        self.peak_power_watts = current_power
                    
                    # Calculate energy (P * time)
                    self.energy_watthours += (current_power * time_diff) / 3600  # Convert to watt-hours
                
                self.last_rotation_time = current_time
            self.last_hall_state = current_state
    
    def check_button_press(self):
        """Check for button press from Arduino to cycle display modes"""
        if self.arduino.in_waiting > 0:
            data = self.arduino.readline().decode('utf-8').strip()
            if data == "B:TOGGLE":
                current_time = time.time()
                # Debounce button press (ignore presses within 0.5 seconds)
                if current_time - self.last_button_press > 0.5:
                    self.toggle_led()  # Now just toggles LED on/off
                    self.last_button_press = current_time
            elif data.startswith("V:"):
                # Process voltage reading from Arduino if available
                try:
                    measured_voltage = float(data[2:])
                    # Update power calculations with real measured voltage
                    if self.is_active and measured_voltage > 0.1:  # Only update if significant voltage
                        self.AVERAGE_VOLTAGE = (self.AVERAGE_VOLTAGE * 0.9) + (measured_voltage * 0.1)  # Smooth updates
                except ValueError:
                    pass  # Ignore malformed voltage readings
    
    def draw_progress_bar(self, x, y, width, height, progress, color=1, border=True):
        """Draw a progress bar on the OLED display"""
        if border:
            # Draw border
            self.display.rect(x, y, width, height, color)
            # Draw filled portion
            fill_width = max(0, min(width - 2, int((width - 2) * progress)))
            if fill_width > 0:
                self.display.fill_rect(x + 1, y + 1, fill_width, height - 2, color)
        else:
            # Draw filled portion without border
            fill_width = max(0, min(width, int(width * progress)))
            if fill_width > 0:
                self.display.fill_rect(x, y, fill_width, height, color)
    
    def update_led_strip(self):
        """Update LED strip based on progress"""
        if not self.led_enabled:
            self.pixels.fill((0, 0, 0))
            return
            
        progress = min(1.0, self.rotation_count / self.treat_threshold)
        lit_pixels = int(self.num_pixels * progress)
        
        # Create progress bar effect
        for i in range(self.num_pixels):
            if i < lit_pixels:
                # Gradient from blue to green based on progress
                self.pixels[i] = (0, int(255 * (i/lit_pixels)), int(255 * (1-i/lit_pixels)))
            else:
                self.pixels[i] = (0, 0, 0)
    
    def dispense_treat(self):
        """Dispense a treat by rotating the continuous servo slowly"""
        # Convert from -100 to 100 range to -1.0 to 1.0 range
        slow_speed = 15  # Adjust this value for optimal dispensing speed
        
        # Rotate slowly clockwise for a short time
        self.set_servo_speed(slow_speed)
        time.sleep(1.5)  # Run for 1.5 seconds
        
        # Stop the servo
        self.set_servo_speed(0)
        
        # Reset rotation counter
        self.rotation_count = 0
    
    def set_servo_speed(self, speed):
        """Set the speed of the continuous rotation servo
        
        Args:
            speed: Integer from -100 to 100, where:
                  0 = stopped
                  positive = clockwise rotation
                  negative = counter-clockwise rotation
                  larger magnitude = faster rotation
        """
        # Convert from -100 to 100 range to -1.0 to 1.0 range
        self.servo_speed = speed
        throttle = speed / 100.0
        
        # Ensure throttle is within valid range
        throttle = max(-1.0, min(1.0, throttle))
        
        # Apply to servo
        self.servo.throttle = throttle
    
    def format_time(self, seconds):
        """Format seconds into HH:MM:SS"""
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        seconds = int(seconds % 60)
        return f"{hours:02d}:{minutes:02d}:{seconds:02d}"
    
    def update_display(self):
        """Update OLED display with comprehensive stats in a grid layout"""
        self.display.fill(0)
        
        # Calculate progress percentage
        progress_pct = min(100, int(self.rotation_count / self.treat_threshold * 100))
        
        # Format values for display
        dist_str = f'{self.distance_miles:.2f}'
        energy_str = f'{self.energy_watthours:.1f}'
        speed_str = f'{self.current_speed_mph:.1f}'
        
        # Top row: Status and progress bar
        status_text = "ACTIVE" if self.is_active else "IDLE"
        self.display.text(status_text, 0, 0, 1)
        
        # Progress bar (treat progress)
        self.display.text(f'{progress_pct}%', 110, 0, 1)
        self.draw_progress_bar(45, 0, 60, 7, progress_pct/100)
        
        # Grid layout - left column labels
        self.display.text("ROT:", 0, 16, 1)
        self.display.text("DIST:", 0, 28, 1)
        self.display.text("PWR:", 0, 40, 1)
        self.display.text("SPD:", 0, 52, 1)
        
        # Grid layout - right column values
        self.display.text(f'{self.rotation_count}/{self.total_rotations}', 35, 16, 1)
        self.display.text(f'{dist_str}mi', 35, 28, 1)
        self.display.text(f'{energy_str}Wh', 35, 40, 1)
        self.display.text(f'{speed_str}mph', 35, 52, 1)
        
        # Right column labels
        self.display.text("TIME:", 75, 16, 1)
        self.display.text("PEAK:", 75, 28, 1)
        self.display.text("AVG:", 75, 40, 1)
        
        # Right column values
        session_mins = int((time.time() - self.session_start_time) / 60)
        self.display.text(f'{session_mins}m', 110, 16, 1)
        self.display.text(f'{self.peak_speed_mph:.1f}', 110, 28, 1)
        self.display.text(f'{self.avg_speed_mph:.1f}', 110, 40, 1)
        
        # Battery Information
        battery_status = "OK" if not self.lbat_pin.value else "LOW"
        power_loss = "ON" if not self.pld_pin.value else "OFF"
        
        self.display.text("BAT:", 0, 56, 1)
        self.display.text(battery_status, 35, 56, 1)
        self.display.text("PWR:", 75, 56, 1)
        self.display.text(power_loss, 110, 56, 1)
        
        # Update display
        self.display.show()
    
    def toggle_led(self):
        """Toggle LED strip on/off"""
        self.led_enabled = not self.led_enabled
    
    def run(self):
        """Main loop to run the rodent wheel monitoring"""
        try:
            while True:
                self.update_hall_sensor()
                self.check_button_press()
                self.update_led_strip()
                self.update_display()
                
                # Check if treat threshold is reached
                if self.rotation_count >= self.treat_threshold:
                    self.dispense_treat()
                
                time.sleep(0.01)  # Small delay to prevent CPU hogging
                
        except KeyboardInterrupt:
            # Clean up
            self.pixels.fill((0, 0, 0))
            self.pixels.show()
            self.display.fill(0)
            self.display.show()
            self.set_servo_speed(0)  # Stop the servo
            self.arduino.close()

if __name__ == "__main__":
    wheel = RodentWheel()
    wheel.run()