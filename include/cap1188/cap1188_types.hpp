#pragma once

#include <cstdint>

namespace CAP1188 {

// Touch channel enumeration
enum class TouchChannel : uint8_t {
    C1 = 0,
    C2 = 1,
    C3 = 2,
    C4 = 3,
    C5 = 4,
    C6 = 5,
    C7 = 6,
    C8 = 7,
    ALL = 0xFF
};

// LED states for advanced control
enum class LEDState : uint8_t {
    OFF = 0,
    ON = 1,
    PULSE_1 = 2,
    PULSE_2 = 3,
    BREATHE = 4
};

// Human-readable LED behavior settings
enum class LEDBehavior : uint8_t {
    OFF = 0,                // LEDs always off
    TOUCH_FEEDBACK = 1,     // LED on while touched (default)
    PULSE_ON_TOUCH = 2,     // Brief pulse when touched
    BREATHE_ON_TOUCH = 3,   // Breathe effect when touched
    FADE_ON_RELEASE = 4,    // Fade out when released
    USE_GLOBAL = 255        // Use device-wide setting (for per-channel config)
};

// LED animation speed settings
enum class LEDSpeed : uint8_t {
    VERY_SLOW = 0,    // 2 second cycles
    SLOW = 1,         // 1 second cycles
    MEDIUM = 2,       // 0.5 second cycles (default)
    FAST = 3,         // 0.25 second cycles
    VERY_FAST = 4     // 0.125 second cycles
};

// Power management modes
enum class PowerMode : uint8_t {
    ACTIVE = 0,
    STANDBY = 1,
    DEEP_SLEEP = 2
};

// Gain settings for touch sensitivity
enum class Gain : uint8_t {
    GAIN_1X = 0x00,
    GAIN_2X = 0x20,
    GAIN_4X = 0x40,
    GAIN_8X = 0x60
};

// Delta sense settings
enum class DeltaSense : uint8_t {
    SENSE_128X = 0x00,
    SENSE_64X = 0x10,
    SENSE_32X = 0x20,
    SENSE_16X = 0x30,
    SENSE_8X = 0x40,
    SENSE_4X = 0x50,
    SENSE_2X = 0x60,
    SENSE_1X = 0x70
};

// Human-readable touch sensitivity settings
enum class TouchSensitivity : uint8_t {
    VERY_LOW = 0,    // For thick materials, gloves - high threshold
    LOW = 1,         // For normal materials with some interference
    MEDIUM = 2,      // Default balanced setting
    HIGH = 3,        // For thin materials, direct contact
    VERY_HIGH = 4,   // Maximum sensitivity - low threshold
    USE_GLOBAL = 255 // Use device-wide setting (for per-channel config)
};

// Touch response speed settings
enum class TouchResponseSpeed : uint8_t {
    VERY_SLOW = 0,   // 560ms intervals - most stable
    SLOW = 1,        // 280ms intervals  
    MEDIUM = 2,      // 140ms intervals (default)
    FAST = 3,        // 70ms intervals
    VERY_FAST = 4    // 35ms intervals - fastest response
};

// Touch stability vs speed settings
enum class TouchStability : uint8_t {
    INSTANT = 0,     // Immediate response, may be jittery
    FAST = 1,        // Quick response with minimal filtering
    BALANCED = 2,    // Good balance of speed and stability (default)
    STABLE = 3,      // Slower but very stable
    VERY_STABLE = 4  // Slowest but most stable
};

// Noise filtering settings
enum class NoiseFiltering : uint8_t {
    DISABLED = 0,    // No filtering - fastest response
    LIGHT = 1,       // Basic filtering for clean environments
    MEDIUM = 2,      // Standard filtering (default)
    HEAVY = 3,       // Aggressive filtering for noisy environments
    MAXIMUM = 4      // Maximum filtering for very noisy environments
};

// Multi-touch configuration
enum class MultiTouchMode : uint8_t {
    DISABLED = 0,        // Only single touch allowed
    ENABLED = 1,         // Multiple touches allowed
    TWO_FINGER_MAX = 2,  // Maximum 2 simultaneous touches
    GESTURE_MODE = 3     // Optimized for gesture recognition
};

// Error codes
enum class Error : int8_t {
    SUCCESS = 0,
    I2C_ERROR = -1,
    DEVICE_NOT_FOUND = -2,
    INVALID_PARAMETER = -3,
    TIMEOUT = -4,
    NOT_INITIALIZED = -5,
    CONFIGURATION_ERROR = -6
};

// Touch configuration for individual channels
struct TouchConfig {
    // Channel enable/disable
    bool enabled = true;
    
    // Human-readable per-channel overrides (USE_GLOBAL to use device-wide settings)
    TouchSensitivity sensitivity = TouchSensitivity::USE_GLOBAL;
    LEDBehavior led_behavior = LEDBehavior::USE_GLOBAL;
    
    // Advanced per-channel settings
    bool repeat_enabled = false;        // Auto-repeat when held
    uint8_t custom_threshold = 0;       // 0-255: Custom threshold value (0 = use calculated from sensitivity)
    uint8_t noise_threshold = 37;       // 0-255: Noise detection threshold (default 37)
    
    // Legacy settings (for backward compatibility)
    uint8_t threshold = 64;            // 0-255: Direct threshold control (overrides sensitivity if custom_threshold == 0)
    bool linked_led = true;            // LED linked to touch detection
    
    TouchConfig() = default;
    TouchConfig(uint8_t thresh, bool en = true, bool led = true, uint8_t noise = 37)
        : enabled(en), noise_threshold(noise), threshold(thresh), linked_led(led) {}
};

// Device-wide configuration
struct DeviceConfig {
    // Hardware interface
    uint8_t reset_pin = 255;  // GPIO pin for reset (255 = disabled/no reset)
    
    // Human-readable touch settings
    TouchSensitivity sensitivity = TouchSensitivity::MEDIUM;
    TouchResponseSpeed response_speed = TouchResponseSpeed::MEDIUM;
    TouchStability stability = TouchStability::BALANCED;
    
    // Noise and filtering
    NoiseFiltering noise_filtering = NoiseFiltering::MEDIUM;
    bool auto_recalibration = true;
    bool digital_noise_filter = true;
    bool analog_noise_filter = true;
    
    // Multi-touch settings
    MultiTouchMode multi_touch = MultiTouchMode::DISABLED;
    bool multi_touch_enabled = false;
    
    // LED behavior
    LEDBehavior led_behavior = LEDBehavior::TOUCH_FEEDBACK;
    LEDSpeed led_speed = LEDSpeed::MEDIUM;
    bool led_active_high = false;  // false = active low, true = active high
    
    // Power management
    bool interrupts_enabled = true;
    bool deep_sleep_enabled = false;
    
    // Legacy settings (for backward compatibility)
    bool max_duration_enabled = false;     // Enable maximum touch duration
    bool timeout_enabled = false;          // Enable communication timeout
    Gain gain = Gain::GAIN_1X;             // Direct gain control (overrides sensitivity if set)
    DeltaSense delta_sense = DeltaSense::SENSE_32X; // Direct delta sense control
    uint8_t standby_sensitivity = 47;      // 0-255: Standby mode sensitivity (default 47)
    
    DeviceConfig() = default;
};

// Touch event structure for callback functions
struct TouchEvent {
    TouchChannel channel;
    bool pressed;           // true = press, false = release
    uint32_t timestamp;     // System timestamp of event
    uint8_t delta_count;    // Touch strength indicator
    
    TouchEvent() = default;
    TouchEvent(TouchChannel ch, bool press, uint32_t time = 0, uint8_t delta = 0)
        : channel(ch), pressed(press), timestamp(time), delta_count(delta) {}
};

// LED configuration for advanced effects
struct LEDConfig {
    LEDState state = LEDState::OFF;
    uint16_t pulse1_period_ms = 1000;  // Pulse 1 period in milliseconds (32-2048ms)
    uint16_t pulse2_period_ms = 1500;  // Pulse 2 period in milliseconds (32-2048ms)
    uint16_t breathe_period_ms = 2000; // Breathe period in milliseconds (32-2048ms)
    uint8_t duty_cycle_percent = 50;   // Duty cycle as percentage (0-100%)
    uint8_t ramp_rate = 0;             // Ramp rate for transitions (0-255)
    uint16_t off_delay_ms = 0;         // Delay before turning off in milliseconds
    
    LEDConfig() = default;
    LEDConfig(LEDState st) : state(st) {}
};

// Device status information
struct DeviceStatus {
    bool touch_detected = false;        // Any touch currently detected
    bool multiple_touch = false;        // Multiple touches detected
    bool noise_detected = false;        // Noise detected on inputs
    bool calibration_failed = false;    // Auto-calibration failure
    bool baseline_out_of_range = false; // Baseline count out of range
    uint8_t touched_channels = 0x00;    // Bitmask of touched channels
    
    DeviceStatus() = default;
};

// Callback function types
using TouchCallback = void(*)(const TouchEvent& event);
using ErrorCallback = void(*)(Error error, const char* message);

// Helper functions for bit manipulation
constexpr bool isChannelTouched(uint8_t status_byte, TouchChannel channel) {
    return (status_byte & (1 << static_cast<uint8_t>(channel))) != 0;
}

constexpr uint8_t channelToMask(TouchChannel channel) {
    if (channel == TouchChannel::ALL) {
        return 0xFF;
    }
    return (1 << static_cast<uint8_t>(channel));
}

constexpr TouchChannel maskToChannel(uint8_t mask) {
    for (uint8_t i = 0; i < 8; ++i) {
        if (mask & (1 << i)) {
            return static_cast<TouchChannel>(i);
        }
    }
    return TouchChannel::C1; // Default fallback
}

// Threshold calculation helpers
constexpr uint8_t calculateThreshold(uint8_t base_count, uint8_t sensitivity_percent) {
    // Calculate threshold as percentage of base count
    // Clamp to valid range (1-255)
    uint16_t threshold = (static_cast<uint16_t>(base_count) * sensitivity_percent) / 100;
    if (threshold < 1) threshold = 1;
    if (threshold > 255) threshold = 255;
    return static_cast<uint8_t>(threshold);
}

// LED timing conversion helpers
constexpr uint8_t millisecondsToLEDPeriod(uint16_t ms) {
    // Convert milliseconds to LED period register value
    // Period = (register_value + 1) * 32ms
    // Clamp to valid range (32-2048ms)
    if (ms < 32) ms = 32;
    if (ms > 2048) ms = 2048;
    return static_cast<uint8_t>((ms / 32) - 1);
}

constexpr uint16_t ledPeriodToMilliseconds(uint8_t period_reg) {
    // Convert LED period register value to milliseconds
    return static_cast<uint16_t>((period_reg + 1) * 32);
}

constexpr uint8_t percentToDutyCycle(uint8_t percent) {
    // Convert percentage (0-100) to duty cycle register value (0-255)
    if (percent > 100) percent = 100;
    return static_cast<uint8_t>((static_cast<uint16_t>(percent) * 255) / 100);
}

constexpr uint8_t dutyCycleToPercent(uint8_t duty_cycle) {
    // Convert duty cycle register value (0-255) to percentage (0-100)
    return static_cast<uint8_t>((static_cast<uint16_t>(duty_cycle) * 100) / 255);
}

// Configuration change detection structure
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
    bool gain_changed : 1;
    bool delta_sense_changed : 1;
    bool standby_sensitivity_changed : 1;
    bool auto_recalibration_changed : 1;
    bool digital_noise_filter_changed : 1;
    bool analog_noise_filter_changed : 1;
    
    ConfigChangeSet() : sensitivity_changed(false), response_speed_changed(false), 
                       stability_changed(false), noise_filtering_changed(false),
                       led_behavior_changed(false), led_speed_changed(false),
                       led_polarity_changed(false), multi_touch_changed(false),
                       interrupts_changed(false), power_settings_changed(false),
                       gain_changed(false), delta_sense_changed(false),
                       standby_sensitivity_changed(false), auto_recalibration_changed(false),
                       digital_noise_filter_changed(false), analog_noise_filter_changed(false) {}
    
    bool hasChanges() const {
        return sensitivity_changed || response_speed_changed || stability_changed ||
               noise_filtering_changed || led_behavior_changed || led_speed_changed ||
               led_polarity_changed || multi_touch_changed || interrupts_changed ||
               power_settings_changed || gain_changed || delta_sense_changed ||
               standby_sensitivity_changed || auto_recalibration_changed ||
               digital_noise_filter_changed || analog_noise_filter_changed;
    }
};

} // namespace CAP1188