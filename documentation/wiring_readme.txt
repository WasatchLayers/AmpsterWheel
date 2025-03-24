# AmpsterWheel Wiring Guide

## Wiring Instructions

### Arduino Nano
- **Power**: Connect to 5V and GND
- **I2C**: Connect SDA to A4 and SCL to A5

### VL53L0X Sensor
- **Power**: Connect to 3.3V and GND
- **I2C**: Connect SDA to A4 and SCL to A5

### FS90R Servo
- **Power**: Connect to 5V and GND
- **Control**: Connect signal wire to D9

### Stepper Motor (17HS4023)
- **Connection**: Use a bridge rectifier to connect to the generator circuit

### OLED Display
- **Power**: Connect to 3.3V or 5V and GND
- **I2C**: Connect SDA to A4 and SCL to A5

### LED Strip (WS2812B)
- **Power**: Connect to 5V and GND
- **Data**: Connect to D6

### Geekworm X-UPS1
- **Power Input**: Connect to 12V power source
- **Battery Output**: Connect to Arduino power input
- **LBAT/PLD**: Connect to D3 and D4 for battery and power loss detection

### Transistor Switch
- **Control**: Connect gate to a digital pin for manual control
- **Load**: Connect between generator and battery circuit

## Notes
- Ensure all connections are secure and insulated to prevent short circuits.
- Double-check polarity for all power connections to avoid damage.
- Use appropriate resistor values for any additional components as needed.
