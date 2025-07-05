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
    uint8_t threshold = 0x40;          // Touch detection threshold (0-255)
    bool enabled = true;               // Channel enable state
    bool linked_led = true;            // LED linked to touch detection
    uint8_t noise_threshold = 0x25;    // Noise detection threshold
    
    TouchConfig() = default;
    TouchConfig(uint8_t thresh, bool en = true, bool led = true, uint8_t noise = 0x25)
        : threshold(thresh), enabled(en), linked_led(led), noise_threshold(noise) {}
};

// Device-wide configuration
struct DeviceConfig {
    bool multi_touch_enabled = false;      // Allow multiple simultaneous touches
    bool interrupts_enabled = true;        // Enable interrupt generation
    bool max_duration_enabled = false;     // Enable maximum touch duration
    bool digital_noise_filter = true;      // Enable digital noise filtering
    bool analog_noise_filter = true;       // Enable analog noise filtering
    bool timeout_enabled = false;          // Enable communication timeout
    Gain gain = Gain::GAIN_1X;             // Touch sensitivity gain
    DeltaSense delta_sense = DeltaSense::SENSE_32X; // Delta count scaling
    bool led_active_high = false;          // LED polarity (false = active low)
    uint8_t standby_sensitivity = 0x2F;    // Standby mode sensitivity
    
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
    uint8_t pulse1_period = 0x84;      // Pulse 1 period
    uint8_t pulse2_period = 0x85;      // Pulse 2 period  
    uint8_t breathe_period = 0x86;     // Breathe period
    uint8_t duty_cycle = 0x50;         // Duty cycle (0-255)
    uint8_t ramp_rate = 0x00;          // Ramp rate for transitions
    uint8_t off_delay = 0x00;          // Delay before turning off
    
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

} // namespace CAP1188