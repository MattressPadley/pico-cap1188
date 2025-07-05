# CAP1188 Register Map

## I2C Device Addresses

The CAP1188 supports multiple I2C addresses selected by resistor configuration on the AD pin:

| Address | Description |
|---------|-------------|
| 0x28    | Easiest to set (wire AD to 3Vo) |
| 0x29    | Default address |
| 0x2A    | Alternative address |
| 0x2B    | Alternative address |  
| 0x2C    | Alternative address |
| 0x2D    | Alternative address |

## Core Control Registers

| Register Name | Address | Description |
|---------------|---------|-------------|
| CAP1188_MAIN | 0x00 | Main Control Register |
| CAP1188_MAIN_INT | 0x01 | Main Control Interrupt Register |
| CAP1188_SENINPUTSTATUS | 0x03 | Sensor Input Status Register |

## Configuration Registers

| Register Name | Address | Description |
|---------------|---------|-------------|
| CAP1188_MTBLK | 0x2A | Multiple Touch Configuration |
| CAP1188_STANDBYCFG | 0x41 | Standby Configuration |

## LED Control Registers

| Register Name | Address | Description |
|---------------|---------|-------------|
| CAP1188_LEDLINK | 0x72 | Sensor Input LED Linking |
| CAP1188_LEDPOL | 0x73 | LED Polarity |

## Device Identification Registers

| Register Name | Address | Expected Value | Description |
|---------------|---------|----------------|-------------|
| CAP1188_PRODID | 0xFD | 0x50 | Product ID |
| CAP1188_MANUID | 0xFE | 0x5D | Manufacturer ID |
| CAP1188_REV | 0xFF | 0x83 | Revision |

## Additional Configuration Registers

| Register Name | Address | Description |
|---------------|---------|-------------|
| REPEAT_RATE_ENABLE | 0x28 | Repeat Rate Enable Register |
| AVERAGING_SAMPLING_CONFIG | 0x24 | Averaging and Sampling Configuration |

## Register Bit Definitions

### Main Control Register (0x00)
- Controls general device operation
- Includes interrupt enable/disable functionality

### Sensor Input Status Register (0x03)
- Bit 0-7: Touch status for sensors C1-C8
- 1 = Touch detected, 0 = No touch

### Multiple Touch Configuration (0x2A)
- Controls simultaneous touch detection
- Default: 0x00 (multiple touch disabled)

### LED Link Register (0x72)
- Links LED outputs to sensor inputs
- Default: 0xFF (all LEDs linked to corresponding sensors)

### Standby Configuration (0x41)
- Controls power management settings
- Default: 0x30

## Register Access Notes

- All registers are 8-bit
- Register addresses use 0xXX format in code
- Some datasheets use XXh notation (equivalent to 0xXX)
- Read/write operations follow standard I2C protocol