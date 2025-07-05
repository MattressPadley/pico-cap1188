# CAP1188 Electrical Characteristics

## Absolute Maximum Ratings

| Parameter | Min | Max | Unit | Notes |
|-----------|-----|-----|------|-------|
| Supply Voltage (VDD) | -0.3 | 6.0 | V | Relative to VSS |
| Input Voltage | -0.3 | VDD + 0.3 | V | Any pin to VSS |
| Output Current | - | 25 | mA | Per LED output |
| Total LED Current | - | 100 | mA | All LED outputs combined |
| Storage Temperature | -65 | 150 | °C | - |
| Junction Temperature | - | 150 | °C | - |

## Operating Conditions

| Parameter | Min | Typ | Max | Unit | Notes |
|-----------|-----|-----|-----|------|-------|
| Supply Voltage | 2.7 | 3.3/5.0 | 5.5 | V | - |
| Operating Temperature | -40 | 25 | 85 | °C | - |
| Humidity | - | - | 85 | %RH | Non-condensing |

## DC Electrical Characteristics
*VDD = 3.3V, TA = 25°C, unless otherwise specified*

### Power Supply
| Parameter | Min | Typ | Max | Unit | Conditions |
|-----------|-----|-----|-----|------|------------|
| Supply Current (Active) | - | 1.5 | 3.0 | mA | All sensors active |
| Supply Current (Standby) | - | 50 | 100 | µA | Standby mode |
| Supply Current (Deep Sleep) | - | 5 | 15 | µA | Deep sleep mode |
| Power-on Reset Voltage | 2.3 | 2.5 | 2.7 | V | VDD rising |
| Brown-out Reset Voltage | 2.0 | 2.2 | 2.4 | V | VDD falling |

### Digital I/O Characteristics
| Parameter | Min | Typ | Max | Unit | Conditions |
|-----------|-----|-----|-----|------|------------|
| Input High Voltage (VIH) | 0.7×VDD | - | VDD+0.3 | V | - |
| Input Low Voltage (VIL) | -0.3 | - | 0.3×VDD | V | - |
| Output High Voltage (VOH) | VDD-0.4 | - | - | V | IOH = 2mA |
| Output Low Voltage (VOL) | - | - | 0.4 | V | IOL = 2mA |
| Input Leakage Current | -1 | - | 1 | µA | VI = VDD or VSS |

### LED Driver Characteristics
| Parameter | Min | Typ | Max | Unit | Conditions |
|-----------|-----|-----|-----|------|------------|
| LED Drive Current | 0.5 | - | 25 | mA | Per channel |
| LED Voltage Drop | - | 1.2 | - | V | At 10mA |
| LED Rise Time | - | 1 | - | µs | 10% to 90% |
| LED Fall Time | - | 1 | - | µs | 90% to 10% |

## AC Electrical Characteristics

### I2C Interface Timing
| Parameter | Symbol | Min | Typ | Max | Unit | Notes |
|-----------|--------|-----|-----|-----|------|-------|
| Clock Frequency | fSCL | 0 | - | 400 | kHz | Standard/Fast mode |
| Bus Free Time | tBUF | 1.3 | - | - | µs | Between transactions |
| Start Hold Time | tHD;STA | 0.6 | - | - | µs | After START condition |
| Start Setup Time | tSU;STA | 0.6 | - | - | µs | Before START condition |
| Stop Setup Time | tSU;STO | 0.6 | - | - | µs | Before STOP condition |
| Data Hold Time | tHD;DAT | 0 | - | 0.9 | µs | SDA valid after SCL fall |
| Data Setup Time | tSU;DAT | 100 | - | - | ns | SDA stable before SCL rise |
| Clock Low Period | tLOW | 1.3 | - | - | µs | SCL low time |
| Clock High Period | tHIGH | 0.6 | - | - | µs | SCL high time |

### SPI Interface Timing
| Parameter | Symbol | Min | Typ | Max | Unit | Notes |
|-----------|--------|-----|-----|-----|------|-------|
| Clock Frequency | fSCLK | 0 | - | 2 | MHz | - |
| Clock High Time | tCH | 250 | - | - | ns | SCLK high period |
| Clock Low Time | tCL | 250 | - | - | ns | SCLK low period |
| Setup Time | tSU | 50 | - | - | ns | Data to clock edge |
| Hold Time | tH | 50 | - | - | ns | Data from clock edge |
| CS Setup Time | tCSS | 50 | - | - | ns | CS before first clock |
| CS Hold Time | tCSH | 50 | - | - | ns | CS after last clock |

### Capacitive Sensing
| Parameter | Min | Typ | Max | Unit | Notes |
|-----------|-----|-----|-----|------|-------|
| Sensor Capacitance Range | 5 | - | 50 | pF | Per sensor input |
| Touch Sensitivity | 0.1 | - | - | pF | Minimum detectable change |
| Sensor Update Rate | - | 35 | - | Hz | All 8 sensors |
| Response Time | - | 35 | 100 | ms | Touch detection |
| Baseline Tracking Rate | - | 8 | - | samples | Auto-calibration |

## Reset and Startup Timing

| Parameter | Min | Typ | Max | Unit | Notes |
|-----------|-----|-----|-----|------|-------|
| Power-on Reset Delay | - | 5 | 15 | ms | VDD stable to ready |
| Hardware Reset Pulse | 1 | - | - | ms | RESET pin low time |
| Reset Recovery Time | - | 5 | 15 | ms | RESET release to ready |
| First Communication Delay | - | 15 | 50 | ms | After power-on/reset |

## Environmental Specifications

### ESD Ratings
| Parameter | Min | Typ | Max | Unit | Test Method |
|-----------|-----|-----|-----|------|-------------|
| Human Body Model | - | - | 2000 | V | JEDEC JS-001 |
| Machine Model | - | - | 200 | V | JEDEC JS-002 |
| Charged Device Model | - | - | 1000 | V | JEDEC JS-003 |

### Thermal Characteristics
| Parameter | Symbol | Value | Unit | Notes |
|-----------|--------|-------|------|-------|
| Thermal Resistance (Junction-Ambient) | θJA | 65 | °C/W | QFN-24, no airflow |
| Thermal Resistance (Junction-Case) | θJC | 15 | °C/W | - |
| Maximum Junction Temperature | TJ | 150 | °C | - |

## Design Guidelines

### Power Supply Design
- Use low-ESR ceramic capacitors (100nF) for high-frequency decoupling
- Add 10µF tantalum/electrolytic capacitor for bulk storage
- Place decoupling capacitors within 5mm of VDD pins

### PCB Layout
- Use 4-layer PCB with dedicated ground and power planes
- Keep sensor traces on top layer with ground guard rings
- Route communication signals on inner layers when possible
- Maintain 50Ω impedance for high-speed signals

### Sensor Design
- Sensor pad size: 5-15mm diameter for finger touch
- Copper thickness: 0.5-1 oz (17-35µm)
- Overlay thickness: 0.5-3mm non-conductive material
- Ground ring spacing: 0.5-1mm from sensor pad