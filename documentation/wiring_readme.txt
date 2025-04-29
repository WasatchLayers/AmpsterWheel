# AmpsterWheel Wiring Guide

## Components Overview

1. **Arduino Nano ESP32**
   - Microcontroller for processing and control

2. **VL53L0X Time-of-Flight Sensor**
   - Detects wheel rotations by sensing a rotating paddle connected to the wheels shaft

3. **FS90R Continuous Rotation Servo**
   - Used for treat dispensing mechanism

4. **17HS4023 Nema 17 Stepper Motor**
   - Acts as a generator to power the system

5. **0.96 Inch OLED Display**
   - Displays system stats and battery information

6. **WS2812B LED Strip**
   - Provides visual feedback

7. **18650 Battery Charger**
   - Manages power supply and battery backup

8. **18650 Battery**
   - Provides power storage

9. **Bridge Rectifiers**
   - Converts AC from the generator to DC

10. **Small Components and Consumables**

    - **4.7kΩ Resistors** (x2): For I2C pull-up on SDA and SCL lines
    - **330Ω Resistor** (x1): On LED strip data line   
    - **1kΩ Resistor** (x1): in series on the data line for the treat motor
    - **10kΩ Resistor** (x1): pull down resistor on data line for treat motor
    - **1000µF 16v Capacitor** (x1): Across LED strip power supply for voltage smoothing
    - **100µF 16v Capacitor** (x1): Stabalize voltage comminig from UPS
    - **24 AWG Hookup Wire Kit**: Assorted colors for all connections
    - **Breadboard or Solderable PCB**: For prototyping
    - **Jumper Wires, Connectors, Heatshrink, and Solder**: For assembly and insulation

## Wiring Instructions

### Arduino Nano ESP32 - microcontroller
- **Power**: Connect 5V VIN to 5V rail and GND to GND rail
- **Pin Configurations**
   - **I2C**: SDA is D21 (A4) (GPIO11) and SCL is D22 (A5) (GPIO12). (Use a 4.7kΩ pull-up resistors from 3.3V on arduino on each line.)
   - **Others**:  D8 (GPIO17) is LED strip (use 330Ω Resistor on data line).
                  D18 (A1) (GPIO02) is treat motor (pull to ground with 10kΩ resistor and a 1kΩ Resistor in series on the data line) .
                  D2 (GPIO05) is normally open momentary button to ground.

### VL53L0X Sensor - TOF sensor
- **Power**: Connect to VIN to 3.3V from arduino and GND rail
- **I2C**: Connect SDA to D21 (A4) (GPIO11) and SCL to D22 (A5) (GPIO12)

### FS90R Servo - Treat motor
- **Power**: Connect to VIN to 5V rail and GND rail
- **Control**: Connect signal wire to D18 (GPIO02 or A1) with 10k pull down resistor and 1kΩ in series with this wire

### Stepper Motor (17HS4023) - Generator
- **Connection**: Connect each phase to separate bridge rectifiers
- **Output**: Combine rectifier outputs and connect to VIN on Step Up Boost Power Converter.

### Step Up Boost Power Converter
- **Input**: Connect VIN+ to combined positives of rectifiers and VIN- to combined negatives of rectifiers
- **Output**: VOUT+ to 5V+ and VOUT- to 5V- on 18650 UPS. 

### 18650 UPS
- **Power Input**:VOUT+ to UPS+ and VOUT- to UPS- from Step Up Boost Power Converter
- **Output**: UPS+ to 5V rail via 100µF smoothing capacitor, and UPS- to GND rail.

### Screen - Display
- **Power**: Connect to 3.3V from arduino or 5v rail and ground to GND rail
- **I2C**: Connect SDA to D21 (A4) (GPIO11) and SCL to D22 (A5) (GPIO12)

### LED Strip (WS2812B)
- **Power**: Connect to 5V and GND rail. Use a 1000µF capacitor across power supply to stabilize voltage.
- **Data**: Connect to D8 (GPIO17) with a 330Ω resistor on the data line to prevent signal reflection.

### Momentary Button (normally open)
- **Connection**: Connect one terminal to D2 (GPIO2) and the other terminal to the GND rail.
- **Configuration**: Using internal pull-up resistor in code.

## Notes
- Ensure all connections are secure and insulated to prevent short circuits.
- Double-check polarity for all power connections to avoid damage.
- Use appropriate resistor values for any additional components as needed.
