# Self-Powered Interactive Rodent Wheel

This project implements a self-sustaining, interactive rodent wheel system that harvests energy from rodent activity while providing feedback and rewards.

## Features
- Energy harvesting from wheel motion via stepper motor generator
- Real-time LED feedback showing exercise progress
- Automated treat dispensing system
- Activity monitoring via Hall effect sensor
- Optional OLED display for stats
- Power-efficient design with manual LED control

## Hardware Requirements
- Arduino Nano
- Stepper Motor (e.g., Usongshine Nema 17)
- Bridge Rectifier Module
- Battery Charging Module
- Hall Effect Sensor (KY-024)
- WS2812B LED Strip (144 LEDs)
- SG90 Servo Motor
- SSD1306 OLED Display (optional)
- Push Button
- Smoothing Capacitor (1000 µF, 16V)

## Setup Instructions
1. Install required Python packages:
   ```bash
   pip install -r requirements.txt
   ```

2. Connect hardware components:
   - Hall sensor to D2
   - LED strip data line to D6 (with 300-500Ω resistor)
   - Servo to D9
   - OLED display to I2C pins (A4/SDA and A5/SCL)
   - Push button to digital input pin

3. Upload the Arduino firmware (coming soon)

4. Run the Python controller:
   ```bash
   python main.py
   ```

## Configuration
- `treat_threshold`: Number of rotations before treat dispensing (default: 100)
- `LED brightness`: Set to 20% by default for power efficiency
- `COM port`: Update `arduino_port` in main.py to match your system

## Power Management
The system uses a self-sustaining power architecture:
1. Energy harvested from wheel motion
2. Stored in lithium battery
3. Powers Arduino and peripherals
4. LED brightness optimized for efficiency
