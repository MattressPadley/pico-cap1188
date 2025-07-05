# CAP1188 I2C Communication Protocol

## I2C Configuration

### Bus Speed
- Standard mode: 100 kHz
- Fast mode: 400 kHz
- Fast mode plus: 1 MHz (check device specifications)

### Pull-up Resistors
- Required on SDA and SCL lines
- Typical values: 4.7kΩ for 5V systems, 2.2kΩ for 3.3V systems

## Basic I2C Operations

### Device Initialization Sequence

1. **Hardware Reset** (if RESET pin connected)
   - Pull RESET low for minimum 1ms
   - Release RESET and wait for device ready

2. **Verify Device Identity**
   ```
   Read Product ID (0xFD) - expect 0x50
   Read Manufacturer ID (0xFE) - expect 0x5D
   Read Revision (0xFF) - expect 0x83
   ```

3. **Configure Device**
   - Set multiple touch configuration (0x2A)
   - Configure LED linking (0x72)
   - Set standby configuration (0x41)

### Reading a Register

**I2C Transaction Format:**
```
START + DEVICE_ADDR(W) + REG_ADDR + RESTART + DEVICE_ADDR(R) + DATA + STOP
```

**Example: Read Sensor Status (0x03)**
```
START + 0x52 + 0x03 + RESTART + 0x53 + [DATA] + NACK + STOP
```

### Writing a Register

**I2C Transaction Format:**
```
START + DEVICE_ADDR(W) + REG_ADDR + DATA + STOP
```

**Example: Write LED Polarity (0x73)**
```
START + 0x52 + 0x73 + 0xFF + STOP
```

## Common Register Operations

### Reading Touch Status
```c
uint8_t touched = read_register(CAP1188_SENINPUTSTATUS);
// Bit 0 = C1, Bit 1 = C2, ..., Bit 7 = C8
```

### Configuring Multiple Touch
```c
write_register(CAP1188_MTBLK, 0x00);  // Disable multiple touch blocking
```

### Linking LEDs to Sensors
```c
write_register(CAP1188_LEDLINK, 0xFF);  // Link all LEDs to sensors
```

### Setting LED Polarity
```c
write_register(CAP1188_LEDPOL, 0x00);   // Active low
write_register(CAP1188_LEDPOL, 0xFF);   // Active high
```

## Address Selection

The I2C address is determined by the voltage on the AD pin:

| AD Pin Voltage | I2C Address |
|----------------|-------------|
| VSS (GND) | 0x28 |
| 0.33 × VDD | 0x29 (default) |
| 0.50 × VDD | 0x2A |
| 0.66 × VDD | 0x2B |
| VDD | 0x2C |
| SDA | 0x2D |

### Address Selection with Resistors

For precise address selection, use resistor dividers:

```
VDD ----[R1]---- AD ----[R2]---- VSS

Address voltage = VDD × R2 / (R1 + R2)
```

## Error Handling

### NACK Response
- Device not present at address
- Register address invalid
- Device in reset or power-down state

### Communication Timeouts
- Implement timeout for all I2C operations
- Typical timeout: 100ms for register operations

### Device Verification
- Always verify device IDs during initialization
- Check for communication errors before proceeding

## Power Management via I2C

### Standby Mode
```c
write_register(CAP1188_STANDBYCFG, 0x30);  // Configure standby
```

### Interrupt Configuration
```c
write_register(CAP1188_MAIN_INT, 0x01);    // Enable interrupts
```

## Performance Considerations

- Register caching for frequently accessed registers
- Burst reads for multiple sequential registers
- Interrupt-driven touch detection to reduce polling
- Proper I2C bus timing to avoid errors