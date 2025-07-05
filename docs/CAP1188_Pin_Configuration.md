# CAP1188 Pin Configuration

## Package Information
- Package Type: QFN-24 (4mm × 4mm)
- Pin Count: 24 pins
- Thermal pad on bottom for heat dissipation

## Pin Descriptions

### Capacitive Sensor Inputs
| Pin | Name | Description |
|-----|------|-------------|
| 2   | C1   | Capacitive sensor input 1 |
| 3   | C2   | Capacitive sensor input 2 |
| 4   | C3   | Capacitive sensor input 3 |
| 5   | C4   | Capacitive sensor input 4 |
| 6   | C5   | Capacitive sensor input 5 |
| 7   | C6   | Capacitive sensor input 6 |
| 8   | C7   | Capacitive sensor input 7 |
| 9   | C8   | Capacitive sensor input 8 |

### LED Driver Outputs
| Pin | Name | Description |
|-----|------|-------------|
| 10  | LED1 | LED driver output 1 (linked to C1) |
| 11  | LED2 | LED driver output 2 (linked to C2) |
| 12  | LED3 | LED driver output 3 (linked to C3) |
| 13  | LED4 | LED driver output 4 (linked to C4) |
| 14  | LED5 | LED driver output 5 (linked to C5) |
| 15  | LED6 | LED driver output 6 (linked to C6) |
| 16  | LED7 | LED driver output 7 (linked to C7) |
| 17  | LED8 | LED driver output 8 (linked to C8) |

### Communication Interface Pins
| Pin | Name | Description |
|-----|------|-------------|
| 19  | SDA  | I2C Serial Data (bidirectional) |
| 20  | SCL  | I2C Serial Clock (input) |
| 21  | MOSI | SPI Master Out Slave In |
| 22  | MISO | SPI Master In Slave Out |
| 23  | SCLK | SPI Serial Clock |
| 24  | CS   | SPI Chip Select (active low) |

### Control and Configuration Pins
| Pin | Name | Description |
|-----|------|-------------|
| 1   | VDD  | Power supply (2.7V - 5.5V) |
| 18  | VSS  | Ground |
| 19  | AD   | Address select / I2C address configuration |
| 20  | RESET| Hardware reset (active low, internal pull-up) |
| 21  | ALERT| Interrupt output (open drain, active low) |

## I2C Pin Configuration

### Minimum I2C Setup
```
VDD  ---- 3.3V or 5V power supply
VSS  ---- Ground
SDA  ---- I2C data line (with 4.7kΩ pull-up to VDD)
SCL  ---- I2C clock line (with 4.7kΩ pull-up to VDD)
AD   ---- Address select (see address configuration)
```

### I2C Address Configuration
Connect AD pin to set I2C address:
- **0x28**: Connect AD to VSS (ground)
- **0x29**: Connect AD to VDD through 47kΩ resistor (default)
- **0x2A**: Connect AD to VDD through 22kΩ resistor  
- **0x2B**: Connect AD to VDD through 15kΩ resistor
- **0x2C**: Connect AD directly to VDD
- **0x2D**: Connect AD to SDA line

## SPI Pin Configuration

### SPI Setup
```
VDD  ---- 3.3V or 5V power supply
VSS  ---- Ground
MOSI ---- SPI data input to CAP1188
MISO ---- SPI data output from CAP1188
SCLK ---- SPI clock input
CS   ---- Chip select (active low)
```

## Optional Control Pins

### Hardware Reset
- **RESET**: Connect to microcontroller GPIO for hardware reset control
- Internal pull-up resistor present
- Pull low for minimum 1ms to reset device

### Interrupt Output
- **ALERT**: Open-drain interrupt output
- Requires external pull-up resistor (4.7kΩ typical)
- Goes low when touch detected (if interrupts enabled)

## Power Supply Considerations

### Supply Voltage
- Operating range: 2.7V to 5.5V
- Recommended: 3.3V for 3.3V systems, 5V for 5V systems

### Supply Current
- Active mode: ~1.5mA typical
- Standby mode: ~50µA typical  
- Deep sleep: ~5µA typical

### Decoupling
- Place 100nF ceramic capacitor close to VDD/VSS pins
- Additional 10µF electrolytic capacitor recommended for noisy environments

## PCB Layout Recommendations

### Sensor Connections
- Keep sensor traces short and direct
- Use ground guard rings around sensor pads
- Minimize coupling between sensor traces

### Communication Lines
- Route I2C/SPI lines away from sensor traces
- Use appropriate trace impedance for high-speed signals
- Add series termination if needed for signal integrity

### Grounding
- Use solid ground plane under device
- Connect thermal pad to ground plane for heat dissipation
- Provide low-impedance ground return paths