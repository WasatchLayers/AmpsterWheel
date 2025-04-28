# AmpsterWheel Project

This project implements a self-sustaining, interactive rodent wheel system that harvests energy from rodent activity while providing feedback and rewards.

## Build Video
https://www.youtube.com/watch?v=JMp5FNHOx8Y

## 3D Printable Files
https://www.printables.com/model/1278881-ampster-wheel

## Features
- Energy harvesting from wheel motion via stepper motor as a generator
- Real-time LED feedback showing exercise progress
- Automated treat dispensing system
- Activity monitoring via time-of-flight sensor
- OLED display for stats
- Power-efficient design with auto off feature for LEDs and screen

## Hardware Requirements
- Arduino Nano ESP32
- (24 AWG) Hookup Wire Kit
- Stepper Motor (17HS4023 Nema 17)
- 2x Bridge Rectifier Module
- WS2812B LED Strip (40 LEDs)
- FS90R Servo Motor (connect signal to D18/GPIO02/A1)
- VL53L0X Time-of-Flight Sensor
- 18650 UPS
- SSD1306 OLED Display
- Momentary Push Button
- Smoothing Capacitors (1000 µF, 16V and 100µF, 16V)
- 2x 4.7kΩ Pull-up Resistors
- 1x 330Ω Resistor
- 1x 1kΩ Resistor
- 1x 10kΩ Resistor
- 8mmx19mm Thrust Bearings
- (4) 608-2RS Bearings
- 3D Printed Rodent Wheel
- Clear Filament
- 8mm x 100mm linear shaft
- 8mm flange coupling connector
- M2x4 screws
- M2x8 screws
- M3x4 screws
- M3x8 screws
- M3x10 screws
- M3x20 screws
- #49 o'ring



## Software Requirements
- Arduino IDE
  - Install the ESP32 board package
  - Install the following libraries (via Library Manager):
    - Adafruit VL53L0X
    - Adafruit SSD1306
    - Adafruit GFX Library
    - Adafruit NeoPixel
    - ESP32Servo
    - Adafruit BusIO

## Setup Instructions
1. **Install Required Software:**
   - Download and install the Arduino IDE.
   - Install the ESP32 board package and all required libraries listed above via the Library Manager.

2. **Connect Hardware Components:**
   - Attach the LED strip data line to pin D8 (GPIO17) on the Arduino, using a 330Ω resistor.
   - Connect the FS90R servo motor signal wire to D18 (GPIO02 or A1).
   - Attach the OLED display to the I2C pins (D21 for SDA and D22 for SCL) with a 4.7kΩ pull-up resistor.
   - Connect the push button to pin D12 (GPIO47) on the Arduino.

3. **Upload the Arduino Firmware:**
   - Open the Arduino IDE.
   - Load the `ampster_wheel_firmware.ino` file.
   - Connect your Arduino Nano ESP32 to the computer and upload the firmware.



## Adding Arduino Nano ESP32 to Board List
1. **Open the Arduino IDE.**
2. **Go to** `File` > `Preferences`.
3. **In the** `Additional Boards Manager URLs` **field, add the following URL:**
   ```
   https://dl.espressif.com/dl/package_esp32_index.json
   ```

## Notes
- Ensure all connections are secure and insulated to prevent short circuits.
- Double-check polarity for all power connections to avoid damage.
- Use appropriate resistor values for any additional components as needed.
