# CAP1188 Capacitive Touch Sensor Overview

## Introduction

The CAP1188 is an 8-channel capacitive touch sensor manufactured by Microchip Technology that uses RightTouch® technology. It's a turnkey capacitive touch controller providing a wide variety of button and proximity functionality.

## Key Features

### Touch Sensing
- Eight (8) individually programmable capacitive touch sensor inputs
- Each sensor input automatically recalibrates to compensate for slow environmental changes
- Sensitivity can be adjusted through the CAP1188's configuration registers
- Individual thresholds for each button
- Multiple Pattern Touch recognition allowing specific button combinations

### LED Drivers
- Eight (8) LED drivers with full-on/off, variable rate blinking, dimming, and breathing capabilities
- Each LED driver can be linked to one of the sensor inputs and triggered when a touch is detected

### Communication Interfaces
- I2C interface with selectable addresses (0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D)
- SPI interface for higher speed communication
- Default I2C address: 0x29

### Power Management
- Operating voltage: 2.7V to 5.5V (compatible with 3.3V and 5V systems)
- Multiple power states available with low quiescent currents
- Deep Sleep mode draws only 5µA on average

### Device Identification
- Product ID: 0x50
- Manufacturer ID: 0x5D  
- Revision: 0x83

## Physical Specifications

- Package: QFN-24 (4mm x 4mm)
- Operating temperature: -40°C to +85°C
- I/O voltage tolerance: 5.5V

## Applications

- Human interface for appliances, lighting controls
- PC peripherals and gaming devices
- Consumer audio/video equipment
- Industrial control systems
- Automotive interior controls