# CAP1188 Library for Raspberry Pi Pico

A comprehensive C++ library for the CAP1188 8-channel capacitive touch sensor, designed for use with the Raspberry Pi Pico SDK.

## Features

- **Complete CAP1188 Support**: Full abstraction of all chip functionality
- **8-Channel Touch Sensing**: Individual channel configuration and monitoring
- **Integrated LED Control**: 8 LED drivers with advanced effects support
- **I2C Communication**: Configurable I2C addresses (0x28-0x2D)
- **Power Management**: Active, standby, and deep sleep modes
- **Interrupt Support**: Hardware interrupt handling for efficient operation
- **Multiple Touch Detection**: Simultaneous touch sensing on multiple channels
- **Noise Filtering**: Built-in digital and analog noise filtering
- **Auto-Calibration**: Automatic baseline calibration and drift compensation

## Hardware Requirements

- Raspberry Pi Pico or compatible RP2040 board
- CAP1188 breakout board or bare IC
- I2C pull-up resistors (4.7kΩ typical)
- Optional: Reset pin connection
- Optional: LEDs for visual feedback

## Quick Start

### 1. Hardware Connections

```
Pico Pin    CAP1188 Pin    Description
--------    -----------    -----------
3.3V        VDD            Power supply
GND         VSS            Ground
GPIO 4      SDA            I2C data line
GPIO 5      SCL            I2C clock line
GPIO 22     RESET          Hardware reset (optional)
```

### 2. Basic Usage

```cpp
#include "cap1188/cap1188.hpp"

using namespace CAP1188;

// Create device instance
CAP1188Device touch_sensor(i2c_default, DEFAULT_I2C_ADDRESS);

int main() {
    stdio_init_all();
    
    // Initialize the device
    if (touch_sensor.begin() != Error::SUCCESS) {
        printf("Failed to initialize CAP1188\\n");
        return -1;
    }
    
    // Main loop
    while (true) {
        uint8_t touched = touch_sensor.getTouchedChannels();
        
        for (int i = 0; i < 8; i++) {
            if (touched & (1 << i)) {
                printf("Channel C%d touched\\n", i + 1);
            }
        }
        
        sleep_ms(50);
    }
}
```

## Building

### Prerequisites

- CMake 3.13 or later
- GCC ARM toolchain
- Raspberry Pi Pico SDK (automatically downloaded if not present)

### Build Steps

#### Using Just (Recommended)

```bash
# Install just if not already installed
# macOS: brew install just
# Linux: cargo install just

# Build the library and examples
just build

# Development build with debug info
just dev-build

# Release build (optimized)
just release-build

# Test build: build, flash, and monitor
just test-build
```

#### Using CMake Directly

```bash
mkdir build
cd build
cmake ..
make
```

The build system will automatically:
- Download Pico SDK if `PICO_SDK_PATH` is not set
- Configure cross-platform build settings
- Build the library and examples

This will build:
- `libcap1188.a` - The main library
- `basic_touch.uf2` - Example application

### Flash to Pico

#### Using picotool (Recommended)

```bash
# Put Pico in BOOTSEL mode
picotool load build/basic_touch.uf2 --force
```

#### Manual Method

1. Hold the BOOTSEL button while connecting the Pico
2. Copy the `.uf2` file to the Pico drive
3. The Pico will automatically reboot and run the program

### Monitoring Serial Output

```bash
# Monitor serial output
just monitor

# Or use picocom directly
picocom -b 115200 --imap lfcrlf /dev/cu.usbmodem*
```

## Library Structure

```
include/cap1188/
├── cap1188.hpp           # Main library header
├── cap1188_registers.hpp # Register definitions
└── cap1188_types.hpp     # Type definitions

src/
└── cap1188.cpp           # Implementation

examples/
└── basic_touch/          # Basic touch detection example

docs/
├── CAP1188_Overview.md           # Device overview
├── CAP1188_Register_Map.md       # Register mapping
├── CAP1188_I2C_Protocol.md       # Communication protocol
├── CAP1188_Pin_Configuration.md  # Hardware setup
└── Implementation_Plan.md        # Development plan
```

## API Reference

### Device Initialization

```cpp
// Constructor
CAP1188Device(i2c_inst_t* i2c, uint8_t address = DEFAULT_I2C_ADDRESS,
              uint sda_pin = PICO_DEFAULT_I2C_SDA_PIN,
              uint scl_pin = PICO_DEFAULT_I2C_SCL_PIN,
              uint reset_pin = 255);  // 255 = no reset pin

// Initialize device
Error begin(uint baudrate = 100000);

// Check connection
bool isConnected();
```

### Touch Sensing

```cpp
// Read all touched channels
uint8_t getTouchedChannels();

// Check specific channel
bool isChannelTouched(TouchChannel channel);

// Configure channel sensitivity
Error setChannelThreshold(TouchChannel channel, uint8_t threshold);

// Enable/disable channels
Error enableChannel(TouchChannel channel, bool enable = true);
```

### LED Control

```cpp
// Basic LED control
Error setLEDState(TouchChannel channel, bool state);

// Advanced LED effects
Error setLEDState(TouchChannel channel, LEDState state);

// Link LEDs to touch detection
Error linkLEDToTouch(TouchChannel channel, bool linked = true);

// Set LED polarity
Error setLEDPolarity(bool active_high);
```

### Power Management

```cpp
// Enter low power modes
Error enterStandby();
Error enterDeepSleep();

// Exit low power modes
Error exitStandby();
Error exitDeepSleep();
```

### Configuration

```cpp
// Device-wide configuration
Error setConfiguration(const DeviceConfig& config);

// Per-channel configuration
Error setChannelConfig(TouchChannel channel, const TouchConfig& config);

// Multiple touch support
Error enableMultiTouch(bool enable = true);
```

## Examples

The library includes several examples:

- **basic_touch**: Simple touch detection with console output
- **led_control**: LED effects and touch-LED linking
- **interrupt_driven**: Interrupt-based touch handling (coming soon)
- **multi_device**: Multiple CAP1188 devices (coming soon)

## Configuration Options

### Device Configuration

```cpp
DeviceConfig config;
config.multi_touch_enabled = true;     // Allow simultaneous touches
config.interrupts_enabled = true;      // Enable interrupts
config.digital_noise_filter = true;    // Enable noise filtering
config.gain = Gain::GAIN_1X;           // Touch sensitivity
config.led_active_high = false;        // LED polarity
```

### Channel Configuration

```cpp
TouchConfig channel_config;
channel_config.threshold = 0x40;       // Touch sensitivity
channel_config.enabled = true;         // Channel enable
channel_config.linked_led = true;      // LED linking
channel_config.noise_threshold = 0x25; // Noise threshold
```

## Troubleshooting

### Common Issues

1. **Device not detected**
   - Check I2C wiring and pull-up resistors
   - Verify device address (default: 0x29)
   - Ensure power supply is stable

2. **Touch not detected**
   - Adjust threshold values (lower = more sensitive)
   - Check sensor pad connections
   - Verify channel is enabled

3. **False touches**
   - Increase threshold values
   - Enable noise filtering
   - Check for electrical interference

### Debug Features

```cpp
// Print device information
void printStatus();
void printConfiguration();

// Get device IDs
uint8_t getProductID();      // Should return 0x50
uint8_t getManufacturerID(); // Should return 0x5D
uint8_t getRevision();       // Should return 0x82
```

#### I2C Debugging

The basic_touch example includes comprehensive I2C debugging:

```cpp
// Built-in I2C scanner
i2c_scan();  // Scans for devices on I2C bus

// Device identification verification
// Automatically reads and verifies:
// - Product ID (0x50)
// - Manufacturer ID (0x5D)  
// - Revision (0x82)
```

## Advanced Features

### Interrupt Handling

```cpp
// Set up touch callback
touch_sensor.setTouchCallback([](const TouchEvent& event) {
    printf("Touch %s on C%d\\n", 
           event.pressed ? "PRESS" : "RELEASE",
           channelToIndex(event.channel) + 1);
});

// Enable interrupts
touch_sensor.enableInterrupts(true);
```

### Multiple Touch Patterns

```cpp
// Enable multiple simultaneous touches
touch_sensor.enableMultiTouch(true);

// Check for specific touch combinations
uint8_t touched = touch_sensor.getTouchedChannels();
if ((touched & 0x03) == 0x03) {
    printf("C1 and C2 touched simultaneously\\n");
}
```

### Custom Sensor Calibration

```cpp
// Manual calibration
touch_sensor.calibrateAllChannels();

// Get baseline values
for (int i = 0; i < 8; i++) {
    TouchChannel channel = static_cast<TouchChannel>(i);
    uint8_t baseline = touch_sensor.getBaseCount(channel);
    printf("C%d baseline: %d\\n", i + 1, baseline);
}
```
