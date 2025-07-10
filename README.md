# CAP1188 Library for Raspberry Pi Pico

<!--toc:start-->
- [CAP1188 Library for Raspberry Pi Pico](#cap1188-library-for-raspberry-pi-pico)
  - [Features](#features)
  - [Hardware Requirements](#hardware-requirements)
  - [Quick Start](#quick-start)
    - [1. Hardware Connections](#1-hardware-connections)
    - [2. Basic Usage](#2-basic-usage)
  - [Building](#building)
    - [Prerequisites](#prerequisites)
    - [Build Steps](#build-steps)
      - [Using Just (Recommended)](#using-just-recommended)
      - [Using CMake Directly](#using-cmake-directly)
    - [Flash to Pico](#flash-to-pico)
      - [Using picotool (Recommended)](#using-picotool-recommended)
      - [Manual Method](#manual-method)
    - [Monitoring Serial Output](#monitoring-serial-output)
  - [Library Structure](#library-structure)
  - [API Reference](#api-reference)
    - [Device Initialization](#device-initialization)
    - [Touch Sensing](#touch-sensing)
    - [LED Control](#led-control)
    - [Power Management](#power-management)
    - [Configuration](#configuration)
    - [Runtime Configuration Changes](#runtime-configuration-changes)
      - [Granular Updates (Individual Settings)](#granular-updates-individual-settings)
      - [Batch Updates (Setting Groups)](#batch-updates-setting-groups)
      - [Smart Configuration Merging](#smart-configuration-merging)
      - [Per-Channel Runtime Updates](#per-channel-runtime-updates)
      - [Runtime Configuration Features](#runtime-configuration-features)
  - [Design Pattern: Pre-Initialized I2C](#design-pattern-pre-initialized-i2c)
    - [Benefits](#benefits)
    - [Multi-Device Usage](#multi-device-usage)
  - [Examples](#examples)
    - [Runtime Configuration Demo](#runtime-configuration-demo)
  - [Configuration Options](#configuration-options)
    - [Device Configuration](#device-configuration)
    - [Channel Configuration](#channel-configuration)
    - [Advanced LED Configuration](#advanced-led-configuration)
  - [Troubleshooting](#troubleshooting)
    - [Common Issues](#common-issues)
    - [Debug Features](#debug-features)
      - [I2C Debugging](#i2c-debugging)
  - [Advanced Features](#advanced-features)
    - [Interrupt Handling](#interrupt-handling)
    - [Multiple Touch Patterns](#multiple-touch-patterns)
    - [Custom Sensor Calibration](#custom-sensor-calibration)
<!--toc:end-->

A comprehensive C++ library for the CAP1188 8-channel capacitive touch sensor, designed for use with the Raspberry Pi Pico SDK.

## Features

- **Complete CAP1188 Support**: Full abstraction of all chip functionality
- **8-Channel Touch Sensing**: Individual channel configuration and monitoring
- **Runtime Configuration Changes**: Dynamic settings updates without reinitialization
- **Human-Readable Configuration**: Intuitive enum-based configuration system
- **Integrated LED Control**: 8 LED drivers with advanced effects support
- **Pre-Initialized I2C Pattern**: Proper multi-device I2C support
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

// Configuration
constexpr uint SDA_PIN = PICO_DEFAULT_I2C_SDA_PIN;  // GPIO 4
constexpr uint SCL_PIN = PICO_DEFAULT_I2C_SCL_PIN;  // GPIO 5
constexpr uint BAUDRATE = 100000;                   // 100 kHz

// Create device instance (no pin parameters)
CAP1188Device touch_sensor(i2c_default, DEFAULT_I2C_ADDRESS);

int main() {
    stdio_init_all();
    
    // Initialize I2C hardware (Pre-Initialized I2C Pattern)
    i2c_init(i2c_default, BAUDRATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    
    // Initialize the device (no baudrate parameter)
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

**Pre-Initialized I2C Pattern** - Applications control I2C hardware setup:

```cpp
// Constructor (simplified - no pin parameters)
CAP1188Device(i2c_inst_t* i2c, uint8_t address = DEFAULT_I2C_ADDRESS,
              uint reset_pin = 255);  // 255 = no reset pin

// Initialize I2C hardware first (application responsibility)
i2c_init(i2c_default, BAUDRATE);
gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
gpio_pull_up(SDA_PIN);
gpio_pull_up(SCL_PIN);

// Initialize device (no baudrate parameter)
Error begin();

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

### Runtime Configuration Changes

The library supports dynamic configuration updates without requiring device reinitialization, enabling adaptive touch sensing based on environmental conditions or user preferences.

#### Granular Updates (Individual Settings)

```cpp
// Update individual settings without affecting others
Error updateSensitivity(TouchSensitivity sensitivity);
Error updateResponseSpeed(TouchResponseSpeed speed);
Error updateStability(TouchStability stability);
Error updateNoiseFiltering(NoiseFiltering filtering);
Error updateLEDBehavior(LEDBehavior behavior, LEDSpeed speed = LEDSpeed::MEDIUM);
Error updateMultiTouchMode(MultiTouchMode mode);

// Example: Increase sensitivity for better detection
touch_sensor.updateSensitivity(TouchSensitivity::HIGH);
```

#### Batch Updates (Setting Groups)

```cpp
// Update related settings together for optimal performance
Error updateTouchSettings(TouchSensitivity sensitivity, TouchResponseSpeed speed, TouchStability stability);
Error updateLEDSettings(LEDBehavior behavior, LEDSpeed speed, bool active_high);
Error updateNoiseSettings(NoiseFiltering filtering, bool digital_filter, bool analog_filter);
Error updatePowerSettings(bool interrupts, bool deep_sleep);

// Example: Configure for noisy environment
touch_sensor.updateTouchSettings(
    TouchSensitivity::LOW,        // Reduce sensitivity
    TouchResponseSpeed::SLOW,     // More stable response
    TouchStability::VERY_STABLE   // Maximum filtering
);
```

#### Smart Configuration Merging

```cpp
// Apply only changed settings automatically
Error updateConfiguration(const DeviceConfig& new_config, bool force_all = false);

// Example: Only updates changed settings
DeviceConfig config = touch_sensor.getConfiguration();
config.sensitivity = TouchSensitivity::HIGH;
config.led_behavior = LEDBehavior::PULSE_ON_TOUCH;
// Only these two settings will be updated
touch_sensor.updateConfiguration(config);
```

#### Per-Channel Runtime Updates

```cpp
// Customize individual channels dynamically
Error updateChannelSensitivity(TouchChannel channel, TouchSensitivity sensitivity);
Error updateChannelLEDBehavior(TouchChannel channel, LEDBehavior behavior);
Error updateChannelConfiguration(TouchChannel channel, const TouchConfig& new_config, bool force_all = false);

// Example: High sensitivity for finger touch, low for gloved hand
touch_sensor.updateChannelSensitivity(TouchChannel::C1, TouchSensitivity::HIGH);
touch_sensor.updateChannelSensitivity(TouchChannel::C2, TouchSensitivity::LOW);
```

#### Runtime Configuration Features

- **Change Detection**: Automatically detects and applies only modified settings
- **Atomic Updates**: All changes succeed or none are applied (rollback on failure)
- **Validation**: Prevents invalid configurations and disruptive changes
- **Performance Optimized**: Minimizes I2C transactions through intelligent batching
- **Non-Disruptive**: Maintains touch detection during configuration updates

## Design Pattern: Pre-Initialized I2C

This library implements the **Pre-Initialized I2C Pattern** for proper multi-device support:

### Benefits

✅ **True Multi-Device Support** - Multiple CAP1188 devices can share the same I2C bus
✅ **Predictable Behavior** - Applications control exactly when and how I2C is configured
✅ **Cleaner Architecture** - Clear separation between hardware and device management
✅ **Better Testability** - Libraries are more focused and easier to test

### Multi-Device Usage

```cpp
// App initializes I2C once for all devices
i2c_init(i2c_default, BAUDRATE);
gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
gpio_pull_up(SDA_PIN);
gpio_pull_up(SCL_PIN);

// Create multiple devices (no pin conflicts)
CAP1188Device touch1(i2c_default, 0x29);
CAP1188Device touch2(i2c_default, 0x2A);
CAP1188Device touch3(i2c_default, 0x2B);

// Initialize all devices
touch1.begin();
touch2.begin();
touch3.begin();

// All devices now share the same properly configured I2C bus
```

## Examples

The library includes several examples:

- **basic_touch**: Touch detection with runtime configuration demonstrations
- **led_control**: LED effects and touch-LED linking (coming soon)
- **interrupt_driven**: Interrupt-based touch handling (coming soon)
- **multi_device**: Multiple CAP1188 devices (coming soon)

### Runtime Configuration Demo

The `basic_touch` example includes a live demonstration of runtime configuration changes that cycles through different settings every 15 seconds:

1. **Sensitivity Adjustment** - Demonstrates `updateSensitivity()`
2. **Response Speed Changes** - Shows `updateResponseSpeed()`
3. **LED Effect Updates** - Illustrates `updateLEDBehavior()`
4. **Batch Setting Updates** - Uses `updateTouchSettings()`
5. **Smart Configuration Merging** - Demonstrates `updateConfiguration()`
6. **Per-Channel Customization** - Shows `updateChannelSensitivity()`
7. **Reset to Defaults** - Returns to baseline configuration

This provides a hands-on demonstration of how runtime configuration changes affect touch behavior and LED feedback in real-time.

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
channel_config.threshold = 64;         // Touch sensitivity (0-255)
channel_config.enabled = true;         // Channel enable
channel_config.linked_led = true;      // LED linking
channel_config.noise_threshold = 37;   // Noise threshold (0-255)
```

### Advanced LED Configuration

```cpp
LEDConfig led_config;
led_config.state = LEDState::BREATHE;
led_config.pulse1_period_ms = 1000;    // Pulse period in milliseconds (32-2048ms)
led_config.pulse2_period_ms = 1500;    // Pulse period in milliseconds (32-2048ms)
led_config.breathe_period_ms = 2000;   // Breathe period in milliseconds (32-2048ms)
led_config.duty_cycle_percent = 75;    // Brightness as percentage (0-100%)
led_config.ramp_rate = 128;            // Transition speed (0-255)
led_config.off_delay_ms = 500;         // Delay before turning off (milliseconds)
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
