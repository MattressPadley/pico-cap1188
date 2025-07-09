# Runtime Configuration Changes

The CAP1188 library supports dynamic configuration updates without requiring device reinitialization. This enables adaptive touch sensing that can respond to changing environmental conditions, user preferences, or application states.

## Overview

Runtime configuration changes allow you to:

- **Adapt to Environment**: Adjust sensitivity based on temperature, humidity, or interference
- **User Customization**: Allow users to tune touch responsiveness to their preferences  
- **Context-Aware Behavior**: Different settings for different application modes
- **Performance Optimization**: Fine-tune settings based on real-world usage patterns
- **Diagnostic Capability**: Test different configurations for troubleshooting

## Key Features

### 🔧 **Change Detection**
The library automatically detects which settings have changed and only updates the necessary hardware registers, minimizing I2C traffic and preserving device state.

### ⚡ **Performance Optimized**
- Only modified registers are written
- Batch updates group related changes
- Atomic operations ensure consistent state
- Minimal disruption to touch detection

### 🛡️ **Safety & Validation**
- Configuration validation prevents invalid states
- Atomic updates with rollback on failure
- Protection against disruptive changes
- Maintains device functionality during updates

### 🎯 **Granular Control**
Choose your update granularity:
- Individual settings (`updateSensitivity()`)
- Related setting groups (`updateTouchSettings()`)
- Complete configuration merging (`updateConfiguration()`)

### 🔢 **Human-Readable Values**
The library uses intuitive decimal values instead of hex register values:
- **Thresholds**: 0-255 decimal (not 0x00-0xFF hex)
- **LED Timing**: Milliseconds (not register values)
- **LED Brightness**: Percentage 0-100% (not 0-255 register values)
- **Delays**: Milliseconds (not register cycles)

## API Reference

### Granular Updates

Update individual settings without affecting others:

```cpp
// Touch sensitivity adjustment
Error updateSensitivity(TouchSensitivity sensitivity);

// Response timing control
Error updateResponseSpeed(TouchResponseSpeed speed);

// Touch stability vs speed tradeoff
Error updateStability(TouchStability stability);

// Environmental noise handling
Error updateNoiseFiltering(NoiseFiltering filtering);

// LED visual feedback
Error updateLEDBehavior(LEDBehavior behavior, LEDSpeed speed = LEDSpeed::MEDIUM);

// Multi-touch configuration
Error updateMultiTouchMode(MultiTouchMode mode);
```

### Batch Updates

Update related settings together for optimal performance:

```cpp
// Touch detection settings
Error updateTouchSettings(TouchSensitivity sensitivity, 
                         TouchResponseSpeed speed, 
                         TouchStability stability);

// LED appearance and behavior
Error updateLEDSettings(LEDBehavior behavior, 
                       LEDSpeed speed, 
                       bool active_high);

// Noise filtering configuration
Error updateNoiseSettings(NoiseFiltering filtering, 
                         bool digital_filter, 
                         bool analog_filter);

// Power and interrupt settings
Error updatePowerSettings(bool interrupts, bool deep_sleep);
```

### Smart Configuration Merging

Apply only changed settings from a complete configuration:

```cpp
// Intelligent configuration update
Error updateConfiguration(const DeviceConfig& new_config, bool force_all = false);

// Example usage
DeviceConfig config = touch_sensor.getConfiguration();
config.sensitivity = TouchSensitivity::HIGH;
config.led_behavior = LEDBehavior::PULSE_ON_TOUCH;
config.noise_filtering = NoiseFiltering::HEAVY;

// Only these three settings will be updated
Error result = touch_sensor.updateConfiguration(config);
```

### Per-Channel Updates

Customize individual channels dynamically:

```cpp
// Channel-specific sensitivity
Error updateChannelSensitivity(TouchChannel channel, TouchSensitivity sensitivity);

// Channel-specific LED behavior
Error updateChannelLEDBehavior(TouchChannel channel, LEDBehavior behavior);

// Complete channel configuration update
Error updateChannelConfiguration(TouchChannel channel, 
                                const TouchConfig& new_config, 
                                bool force_all = false);
```

### Advanced LED Configuration

Configure LED timing and brightness using human-readable values:

```cpp
// LED configuration with decimal values
LEDConfig led_config;
led_config.state = LEDState::BREATHE;
led_config.pulse1_period_ms = 1000;    // 1 second pulse period
led_config.pulse2_period_ms = 1500;    // 1.5 second pulse period
led_config.breathe_period_ms = 2000;   // 2 second breathe cycle
led_config.duty_cycle_percent = 75;    // 75% brightness
led_config.ramp_rate = 128;            // Medium transition speed (0-255)
led_config.off_delay_ms = 500;         // 500ms delay before turning off

// Apply LED configuration
Error setLEDConfiguration(TouchChannel channel, const LEDConfig& config);
```

**LED Timing Ranges:**
- **Pulse/Breathe Periods**: 32-2048 milliseconds
- **Duty Cycle**: 0-100% (converted to 0-255 register values)
- **Ramp Rate**: 0-255 (transition speed)
- **Off Delay**: 0-65535 milliseconds

## Usage Patterns

### Environmental Adaptation

```cpp
// Adapt to noisy environment
void configureForNoisyEnvironment() {
    touch_sensor.updateTouchSettings(
        TouchSensitivity::LOW,        // Reduce false triggers
        TouchResponseSpeed::SLOW,     // More stable response
        TouchStability::VERY_STABLE   // Maximum filtering
    );
    
    touch_sensor.updateNoiseSettings(
        NoiseFiltering::HEAVY,        // Aggressive filtering
        true,                         // Enable digital filter
        true                          // Enable analog filter
    );
}

// Adapt to quiet, controlled environment
void configureForCleanEnvironment() {
    touch_sensor.updateTouchSettings(
        TouchSensitivity::HIGH,       // Better responsiveness
        TouchResponseSpeed::FAST,     // Quick response
        TouchStability::FAST          // Minimal filtering
    );
    
    touch_sensor.updateNoiseSettings(
        NoiseFiltering::LIGHT,        // Basic filtering only
        true,                         // Keep digital filter
        false                         // Disable analog filter
    );
}
```

### Application Mode Changes

```cpp
// Gaming mode - maximum responsiveness
void enableGamingMode() {
    touch_sensor.updateTouchSettings(
        TouchSensitivity::VERY_HIGH,
        TouchResponseSpeed::VERY_FAST,
        TouchStability::INSTANT
    );
    
    touch_sensor.updateLEDBehavior(
        LEDBehavior::PULSE_ON_TOUCH,
        LEDSpeed::VERY_FAST
    );
}

// Accessibility mode - more stable detection
void enableAccessibilityMode() {
    touch_sensor.updateTouchSettings(
        TouchSensitivity::MEDIUM,
        TouchResponseSpeed::SLOW,
        TouchStability::VERY_STABLE
    );
    
    touch_sensor.updateLEDBehavior(
        LEDBehavior::TOUCH_FEEDBACK,
        LEDSpeed::SLOW
    );
}
```

### User Customization

```cpp
// Allow user to adjust sensitivity
void adjustSensitivity(int user_level) {
    TouchSensitivity sensitivity;
    
    switch (user_level) {
        case 1: sensitivity = TouchSensitivity::VERY_LOW; break;
        case 2: sensitivity = TouchSensitivity::LOW; break;
        case 3: sensitivity = TouchSensitivity::MEDIUM; break;
        case 4: sensitivity = TouchSensitivity::HIGH; break;
        case 5: sensitivity = TouchSensitivity::VERY_HIGH; break;
        default: sensitivity = TouchSensitivity::MEDIUM; break;
    }
    
    Error result = touch_sensor.updateSensitivity(sensitivity);
    if (result == Error::SUCCESS) {
        printf("Sensitivity updated to level %d\n", user_level);
    }
}
```

### Per-Channel Customization

```cpp
// Configure channels for different input types
void configureInputTypes() {
    // Channel 1: Finger touch (high sensitivity)
    touch_sensor.updateChannelSensitivity(TouchChannel::C1, TouchSensitivity::HIGH);
    
    // Channel 2: Stylus input (very high sensitivity)
    touch_sensor.updateChannelSensitivity(TouchChannel::C2, TouchSensitivity::VERY_HIGH);
    
    // Channel 3: Gloved hand (low sensitivity)
    touch_sensor.updateChannelSensitivity(TouchChannel::C3, TouchSensitivity::LOW);
    
    // Channel 4: Thick material (very low sensitivity)
    touch_sensor.updateChannelSensitivity(TouchChannel::C4, TouchSensitivity::VERY_LOW);
}
```

### Diagnostic and Testing

```cpp
// Test different configurations for optimal settings
void runSensitivityTest() {
    TouchSensitivity levels[] = {
        TouchSensitivity::VERY_LOW,
        TouchSensitivity::LOW,
        TouchSensitivity::MEDIUM,
        TouchSensitivity::HIGH,
        TouchSensitivity::VERY_HIGH
    };
    
    for (auto level : levels) {
        printf("Testing sensitivity level: %d\n", static_cast<int>(level));
        touch_sensor.updateSensitivity(level);
        
        // Test for 5 seconds
        uint32_t start_time = to_ms_since_boot(get_absolute_time());
        while (to_ms_since_boot(get_absolute_time()) - start_time < 5000) {
            uint8_t touched = touch_sensor.getTouchedChannels();
            if (touched) {
                printf("Touch detected: 0x%02X\n", touched);
            }
            sleep_ms(100);
        }
    }
}
```

## Implementation Details

### Change Detection Algorithm

The library uses a comprehensive change detection system:

```cpp
struct ConfigChangeSet {
    bool sensitivity_changed : 1;
    bool response_speed_changed : 1;
    bool stability_changed : 1;
    bool noise_filtering_changed : 1;
    bool led_behavior_changed : 1;
    bool led_speed_changed : 1;
    bool led_polarity_changed : 1;
    bool multi_touch_changed : 1;
    bool interrupts_changed : 1;
    bool power_settings_changed : 1;
    // ... additional change flags
    
    bool hasChanges() const;
};
```

### Atomic Update Process

1. **Validate Configuration**: Check for invalid settings and disruptive changes
2. **Detect Changes**: Compare old vs new configuration
3. **Apply Updates**: Write only changed registers in logical order
4. **Verify Success**: Confirm all updates succeeded
5. **Rollback on Failure**: Restore previous state if any update fails

### Register Update Optimization

The library groups related register updates to minimize I2C transactions:

- **Sensitivity Updates**: Batch threshold updates for all affected channels
- **LED Updates**: Group timing register writes together
- **Filter Updates**: Combine noise filter and configuration register updates
- **Power Updates**: Coordinate interrupt and sleep mode changes

## Error Handling

Runtime configuration methods return detailed error codes:

```cpp
Error result = touch_sensor.updateSensitivity(TouchSensitivity::HIGH);

switch (result) {
    case Error::SUCCESS:
        printf("Sensitivity updated successfully\n");
        break;
    case Error::NOT_INITIALIZED:
        printf("Device not initialized\n");
        break;
    case Error::I2C_ERROR:
        printf("Communication error\n");
        break;
    case Error::CONFIGURATION_ERROR:
        printf("Invalid configuration\n");
        break;
    default:
        printf("Update failed: %s\n", errorToString(result));
        break;
}
```

## Performance Considerations

### I2C Transaction Optimization

- **Granular Updates**: 1-3 register writes
- **Batch Updates**: 3-8 register writes (grouped)
- **Smart Merging**: Only changed registers (variable)
- **Full Configuration**: 15-20 register writes

### Timing Considerations

- **Update Duration**: 1-10ms depending on scope
- **Non-Blocking**: Touch detection continues during updates
- **Calibration Preservation**: Baseline values maintained when possible

### Memory Usage

- **ConfigChangeSet**: 2 bytes (bit-packed structure)
- **Stack Usage**: Minimal (local variables only)
- **No Dynamic Allocation**: All operations use stack memory

## Best Practices

### 1. Start with Batch Updates
Use batch methods when changing multiple related settings:

```cpp
// Good: Single atomic operation
touch_sensor.updateTouchSettings(sensitivity, speed, stability);

// Avoid: Multiple separate operations
touch_sensor.updateSensitivity(sensitivity);
touch_sensor.updateResponseSpeed(speed);
touch_sensor.updateStability(stability);
```

### 2. Use Smart Merging for Complex Changes
When updating multiple unrelated settings:

```cpp
DeviceConfig config = touch_sensor.getConfiguration();
config.sensitivity = new_sensitivity;
config.led_behavior = new_led_behavior;
config.noise_filtering = new_noise_filtering;
touch_sensor.updateConfiguration(config);
```

### 3. Validate Critical Changes
Check return values for important configuration changes:

```cpp
Error result = touch_sensor.updateSensitivity(TouchSensitivity::HIGH);
if (result != Error::SUCCESS) {
    // Handle error or retry with fallback configuration
    touch_sensor.updateSensitivity(TouchSensitivity::MEDIUM);
}
```

### 4. Consider Update Frequency
Avoid excessive configuration changes:

```cpp
// Good: Update when needed
if (environmental_change_detected) {
    touch_sensor.updateNoiseFiltering(NoiseFiltering::HEAVY);
}

// Avoid: Continuous updates
// Don't update configuration every loop iteration
```

### 5. Use Per-Channel Updates Sparingly
Channel-specific updates are powerful but should be used judiciously:

```cpp
// Good: Different input types
touch_sensor.updateChannelSensitivity(TouchChannel::C1, TouchSensitivity::HIGH);   // Finger
touch_sensor.updateChannelSensitivity(TouchChannel::C2, TouchSensitivity::LOW);    // Glove

// Consider: Whether device-wide settings would work just as well
```

## Troubleshooting

### Configuration Not Taking Effect

1. **Check Return Value**: Ensure the update method returned `Error::SUCCESS`
2. **Verify Device State**: Confirm device is initialized and connected
3. **Check for Conflicts**: Some settings may override others
4. **Timing Issues**: Allow time for settings to stabilize

### Unexpected Behavior After Updates

1. **Rollback to Known Good State**: Use the device's default configuration
2. **Incremental Changes**: Make one change at a time to identify issues
3. **Validate Settings**: Check current configuration with `getConfiguration()`
4. **Recalibrate**: Some changes may require recalibration

### Performance Issues

1. **Batch Related Changes**: Use batch update methods
2. **Minimize Update Frequency**: Only update when necessary
3. **Use Smart Merging**: Let the library optimize register writes
4. **Check I2C Bus Health**: Ensure stable communication

## Example: Complete Environmental Adaptation System

```cpp
class TouchEnvironmentManager {
private:
    CAP1188Device& touch_sensor;
    uint32_t last_adaptation_time = 0;
    
public:
    TouchEnvironmentManager(CAP1188Device& sensor) : touch_sensor(sensor) {}
    
    void adaptToEnvironment() {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // Only check every 5 seconds
        if (now - last_adaptation_time < 5000) return;
        
        // Get current device status
        DeviceStatus status = touch_sensor.getStatus();
        
        if (status.noise_detected) {
            // High noise environment detected
            configureForNoisyEnvironment();
            printf("Adapted to noisy environment\n");
        } else if (status.baseline_out_of_range) {
            // Environmental change detected
            recalibrateAndAdapt();
            printf("Recalibrated for environmental change\n");
        }
        
        last_adaptation_time = now;
    }
    
private:
    void configureForNoisyEnvironment() {
        touch_sensor.updateTouchSettings(
            TouchSensitivity::LOW,
            TouchResponseSpeed::SLOW,
            TouchStability::VERY_STABLE
        );
        
        touch_sensor.updateNoiseSettings(
            NoiseFiltering::HEAVY,
            true,  // Digital filter
            true   // Analog filter
        );
    }
    
    void recalibrateAndAdapt() {
        // Recalibrate all channels
        touch_sensor.calibrateAllChannels();
        
        // Wait for calibration to complete
        sleep_ms(100);
        
        // Apply balanced settings
        touch_sensor.updateTouchSettings(
            TouchSensitivity::MEDIUM,
            TouchResponseSpeed::MEDIUM,
            TouchStability::BALANCED
        );
    }
};

// Usage in main loop
TouchEnvironmentManager env_manager(touch_sensor);

while (true) {
    // Handle touch events
    handleTouchEvents();
    
    // Adapt to environment changes
    env_manager.adaptToEnvironment();
    
    sleep_ms(10);
}
```

This documentation provides comprehensive coverage of the runtime configuration system, enabling developers to leverage its full potential for creating adaptive and responsive touch interfaces.