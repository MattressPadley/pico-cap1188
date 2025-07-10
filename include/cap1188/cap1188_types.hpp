/**
 * @file cap1188_types.hpp
 * @brief Type definitions and enumerations for the CAP1188 library
 *
 * This header contains all the type definitions, enumerations, structures, and
 * constants used throughout the CAP1188 library. It provides human-readable
 * abstractions for hardware registers and configurations.
 *
 * @author CAP1188 Library Contributors
 * @date 2024
 * @version 0.0.1
 */

#pragma once

#include <cstdint>

/**
 * @namespace CAP1188
 * @brief Main namespace for the CAP1188 capacitive touch sensor library
 *
 * Contains all classes, functions, types, and constants for interfacing
 * with the CAP1188 8-channel capacitive touch sensor.
 */
namespace CAP1188 {

/**
 * @enum TouchChannel
 * @brief Enumeration of available touch sensor channels
 *
 * The CAP1188 supports 8 individual capacitive touch channels (C1-C8).
 * Each channel can be configured independently for sensitivity, thresholds,
 * and LED behavior.
 *
 * @note Channel numbering starts from 0 internally but is typically
 *       referenced as C1-C8 in documentation and user interfaces.
 */
enum class TouchChannel : uint8_t {
    C1 = 0,    ///< Touch channel 1
    C2 = 1,    ///< Touch channel 2
    C3 = 2,    ///< Touch channel 3
    C4 = 3,    ///< Touch channel 4
    C5 = 4,    ///< Touch channel 5
    C6 = 5,    ///< Touch channel 6
    C7 = 6,    ///< Touch channel 7
    C8 = 7,    ///< Touch channel 8
    ALL = 0xFF ///< Special value representing all channels
};

/**
 * @enum LEDState
 * @brief LED control states for advanced LED effects
 *
 * Controls the behavior of LEDs connected to the CAP1188's LED drivers.
 * Each LED can be set to various states including static on/off and
 * dynamic effects like pulsing and breathing.
 *
 * @see LEDConfig for timing and duty cycle configuration
 */
enum class LEDState : uint8_t {
    OFF = 0,     ///< LED is off
    ON = 1,      ///< LED is on (static)
    PULSE_1 = 2, ///< LED pulses with pulse 1 timing
    PULSE_2 = 3, ///< LED pulses with pulse 2 timing
    BREATHE = 4  ///< LED breathes (fade in/out)
};

/**
 * @enum LEDBehavior
 * @brief Human-readable LED behavior settings
 *
 * Defines how LEDs respond to touch events. These settings provide
 * intuitive control over LED behavior without needing to understand
 * the underlying hardware registers.
 *
 * @note When used in per-channel configuration, USE_GLOBAL allows
 *       the channel to inherit the device-wide LED behavior setting.
 */
enum class LEDBehavior : uint8_t {
    OFF = 0,                ///< LEDs always off
    TOUCH_FEEDBACK = 1,     ///< LED on while touched (default)
    PULSE_ON_TOUCH = 2,     ///< Brief pulse when touched
    BREATHE_ON_TOUCH = 3,   ///< Breathe effect when touched
    FADE_ON_RELEASE = 4,    ///< Fade out when released
    USE_GLOBAL = 255        ///< Use device-wide setting (for per-channel config)
};

/**
 * @enum LEDSpeed
 * @brief LED animation speed settings
 *
 * Controls the speed of LED animations like pulsing and breathing.
 * These settings affect the timing of LED effects.
 *
 * @note Actual timing depends on the specific LED effect being used.
 *       These values provide relative speed control.
 */
enum class LEDSpeed : uint8_t {
    VERY_SLOW = 0,    ///< 2 second cycles
    SLOW = 1,         ///< 1 second cycles
    MEDIUM = 2,       ///< 0.5 second cycles (default)
    FAST = 3,         ///< 0.25 second cycles
    VERY_FAST = 4     ///< 0.125 second cycles
};

/**
 * @enum PowerMode
 * @brief Power management modes for the CAP1188
 *
 * Controls the power consumption and responsiveness of the device.
 * Lower power modes consume less current but may have reduced
 * functionality or slower response times.
 *
 * @note Deep sleep mode requires wake-up through reset or specific
 *       touch events depending on configuration.
 */
enum class PowerMode : uint8_t {
    ACTIVE = 0,     ///< Normal operation mode
    STANDBY = 1,    ///< Reduced power, slower response
    DEEP_SLEEP = 2  ///< Minimum power, limited functionality
};

/**
 * @enum Gain
 * @brief Gain settings for touch sensitivity (low-level)
 *
 * Controls the analog gain of the touch sensing circuitry.
 * Higher gain values increase sensitivity but may also increase
 * noise susceptibility.
 *
 * @note Most users should use TouchSensitivity instead of direct
 *       gain control for better abstraction.
 * @see TouchSensitivity
 */
enum class Gain : uint8_t {
    GAIN_1X = 0x00, ///< 1x gain (lowest sensitivity)
    GAIN_2X = 0x20, ///< 2x gain
    GAIN_4X = 0x40, ///< 4x gain
    GAIN_8X = 0x60  ///< 8x gain (highest sensitivity)
};

/**
 * @enum DeltaSense
 * @brief Delta sense settings for touch detection (low-level)
 *
 * Controls the sensitivity of the delta counting algorithm used
 * for touch detection. Higher values increase sensitivity to
 * small changes in capacitance.
 *
 * @note Most users should use TouchSensitivity instead of direct
 *       delta sense control for better abstraction.
 * @see TouchSensitivity
 */
enum class DeltaSense : uint8_t {
    SENSE_128X = 0x00, ///< 128x sensitivity (highest)
    SENSE_64X = 0x10,  ///< 64x sensitivity
    SENSE_32X = 0x20,  ///< 32x sensitivity (default)
    SENSE_16X = 0x30,  ///< 16x sensitivity
    SENSE_8X = 0x40,   ///< 8x sensitivity
    SENSE_4X = 0x50,   ///< 4x sensitivity
    SENSE_2X = 0x60,   ///< 2x sensitivity
    SENSE_1X = 0x70    ///< 1x sensitivity (lowest)
};

/**
 * @enum TouchSensitivity
 * @brief Human-readable touch sensitivity settings
 *
 * Provides intuitive control over touch sensitivity without requiring
 * knowledge of underlying hardware parameters. The library automatically
 * translates these values to appropriate register settings.
 *
 * @note When used in per-channel configuration, USE_GLOBAL allows
 *       the channel to inherit the device-wide sensitivity setting.
 */
enum class TouchSensitivity : uint8_t {
    VERY_LOW = 0,    ///< For thick materials, gloves - high threshold
    LOW = 1,         ///< For normal materials with some interference
    MEDIUM = 2,      ///< Default balanced setting
    HIGH = 3,        ///< For thin materials, direct contact
    VERY_HIGH = 4,   ///< Maximum sensitivity - low threshold
    USE_GLOBAL = 255 ///< Use device-wide setting (for per-channel config)
};

/**
 * @enum TouchResponseSpeed
 * @brief Touch response speed settings
 *
 * Controls how quickly the device responds to touch events.
 * Faster speeds provide quicker response but may be less stable
 * in noisy environments.
 *
 * @note Response speed affects the repeat rate for touch detection
 *       and the stability of touch readings.
 */
enum class TouchResponseSpeed : uint8_t {
    VERY_SLOW = 0,   ///< 560ms intervals - most stable
    SLOW = 1,        ///< 280ms intervals
    MEDIUM = 2,      ///< 140ms intervals (default)
    FAST = 3,        ///< 70ms intervals
    VERY_FAST = 4    ///< 35ms intervals - fastest response
};

/**
 * @enum TouchStability
 * @brief Touch stability vs speed settings
 *
 * Controls the trade-off between response speed and stability.
 * Higher stability settings use more averaging and filtering
 * to reduce jitter but increase response time.
 *
 * @note This setting affects the averaging and sampling configuration
 *       of the touch detection algorithm.
 */
enum class TouchStability : uint8_t {
    INSTANT = 0,     ///< Immediate response, may be jittery
    FAST = 1,        ///< Quick response with minimal filtering
    BALANCED = 2,    ///< Good balance of speed and stability (default)
    STABLE = 3,      ///< Slower but very stable
    VERY_STABLE = 4  ///< Slowest but most stable
};

/**
 * @enum NoiseFiltering
 * @brief Noise filtering settings
 *
 * Controls the level of noise filtering applied to touch readings.
 * Higher filtering levels improve noise immunity but may reduce
 * response speed and sensitivity.
 *
 * @note Combines both digital and analog noise filtering settings
 *       for simplified configuration.
 */
enum class NoiseFiltering : uint8_t {
    DISABLED = 0,    ///< No filtering - fastest response
    LIGHT = 1,       ///< Basic filtering for clean environments
    MEDIUM = 2,      ///< Standard filtering (default)
    HEAVY = 3,       ///< Aggressive filtering for noisy environments
    MAXIMUM = 4      ///< Maximum filtering for very noisy environments
};

/**
 * @enum MultiTouchMode
 * @brief Multi-touch configuration modes
 *
 * Controls whether and how the device handles multiple simultaneous
 * touch events. Different modes optimize for different use cases.
 *
 * @note GESTURE_MODE may have specific requirements for channel
 *       configuration and may not be suitable for all applications.
 */
enum class MultiTouchMode : uint8_t {
    DISABLED = 0,        ///< Only single touch allowed
    ENABLED = 1,         ///< Multiple touches allowed
    TWO_FINGER_MAX = 2,  ///< Maximum 2 simultaneous touches
    GESTURE_MODE = 3     ///< Optimized for gesture recognition
};

/**
 * @enum Error
 * @brief Error codes returned by library functions
 *
 * All library functions that can fail return an Error code.
 * SUCCESS (0) indicates successful operation, while negative
 * values indicate various error conditions.
 *
 * @note Error codes are designed to be used in conditional statements
 *       and can be converted to human-readable strings using errorToString().
 */
enum class Error : int8_t {
    SUCCESS = 0,              ///< Operation completed successfully
    I2C_ERROR = -1,           ///< I2C communication error
    DEVICE_NOT_FOUND = -2,    ///< CAP1188 device not found or not responding
    INVALID_PARAMETER = -3,   ///< Invalid parameter passed to function
    TIMEOUT = -4,             ///< Operation timed out
    NOT_INITIALIZED = -5,     ///< Device not initialized (call begin() first)
    CONFIGURATION_ERROR = -6  ///< Configuration validation failed
};

/**
 * @struct TouchConfig
 * @brief Configuration for individual touch channels
 *
 * Defines the settings for a single touch channel, including sensitivity,
 * LED behavior, thresholds, and various behavioral options. Each channel
 * can be configured independently.
 *
 * @note Per-channel settings can override device-wide defaults by using
 *       specific values instead of USE_GLOBAL.
 */
struct TouchConfig {
    bool enabled = true;                                      ///< Enable/disable this channel
    
    // Human-readable per-channel overrides (USE_GLOBAL to use device-wide settings)
    TouchSensitivity sensitivity = TouchSensitivity::USE_GLOBAL; ///< Touch sensitivity (USE_GLOBAL for device default)
    LEDBehavior led_behavior = LEDBehavior::USE_GLOBAL;         ///< LED behavior (USE_GLOBAL for device default)
    
    // Advanced per-channel settings
    bool repeat_enabled = false;        ///< Auto-repeat when held
    uint8_t custom_threshold = 0;       ///< Custom threshold value (0 = use calculated from sensitivity)
    uint8_t noise_threshold = 37;       ///< Noise detection threshold (0-255, default 37)
    
    // Legacy settings (for backward compatibility)
    uint8_t threshold = 64;            ///< Direct threshold control (overrides sensitivity if custom_threshold == 0)
    bool linked_led = true;            ///< LED linked to touch detection
    
    /**
     * @brief Default constructor
     * Initializes with default values
     */
    TouchConfig() = default;
    
    /**
     * @brief Legacy constructor for backward compatibility
     * @param thresh Touch threshold value (0-255)
     * @param en Enable channel
     * @param led Enable LED linking
     * @param noise Noise threshold (0-255)
     */
    TouchConfig(uint8_t thresh, bool en = true, bool led = true, uint8_t noise = 37)
        : enabled(en), noise_threshold(noise), threshold(thresh), linked_led(led) {}
};

/**
 * @struct DeviceConfig
 * @brief Device-wide configuration settings
 *
 * Contains all the configuration settings that apply to the entire device,
 * including touch sensitivity, LED behavior, power management, and hardware
 * interface settings.
 *
 * @note Individual channels can override many of these settings using
 *       TouchConfig with specific values instead of USE_GLOBAL.
 */
struct DeviceConfig {
    // Hardware interface
    uint8_t reset_pin = 255;  ///< GPIO pin for reset (255 = disabled/no reset)
    
    // Human-readable touch settings
    TouchSensitivity sensitivity = TouchSensitivity::MEDIUM;      ///< Default touch sensitivity
    TouchResponseSpeed response_speed = TouchResponseSpeed::MEDIUM; ///< Touch response speed
    TouchStability stability = TouchStability::BALANCED;           ///< Stability vs speed trade-off
    
    // Noise and filtering
    NoiseFiltering noise_filtering = NoiseFiltering::MEDIUM;    ///< Noise filtering level
    bool auto_recalibration = true;                            ///< Enable automatic recalibration
    bool digital_noise_filter = true;                          ///< Enable digital noise filtering
    bool analog_noise_filter = true;                           ///< Enable analog noise filtering
    
    // Multi-touch settings
    MultiTouchMode multi_touch = MultiTouchMode::DISABLED;    ///< Multi-touch mode
    bool multi_touch_enabled = false;                         ///< Enable multi-touch (legacy)
    
    // LED behavior
    LEDBehavior led_behavior = LEDBehavior::TOUCH_FEEDBACK;   ///< Default LED behavior
    LEDSpeed led_speed = LEDSpeed::MEDIUM;                    ///< LED animation speed
    bool led_active_high = false;                             ///< LED polarity (false = active low)
    
    // Power management
    bool interrupts_enabled = true;                           ///< Enable interrupt generation
    bool deep_sleep_enabled = false;                          ///< Enable deep sleep capability
    
    // Legacy settings (for backward compatibility)
    bool max_duration_enabled = false;                        ///< Enable maximum touch duration
    bool timeout_enabled = false;                             ///< Enable communication timeout
    Gain gain = Gain::GAIN_1X;                                ///< Direct gain control (overrides sensitivity if set)
    DeltaSense delta_sense = DeltaSense::SENSE_32X;           ///< Direct delta sense control
    uint8_t standby_sensitivity = 47;                         ///< Standby mode sensitivity (0-255, default 47)
    
    /**
     * @brief Default constructor
     * Initializes with recommended default values
     */
    DeviceConfig() = default;
};

/**
 * @struct TouchEvent
 * @brief Touch event structure for callback functions
 *
 * Contains information about a touch event, including which channel
 * was touched, whether it was a press or release, timing information,
 * and touch strength.
 *
 * @note This structure is passed to touch callback functions to provide
 *       detailed information about touch events.
 */
struct TouchEvent {
    TouchChannel channel;   ///< The channel that generated the event
    bool pressed;           ///< true = press, false = release
    uint32_t timestamp;     ///< System timestamp of event (milliseconds)
    uint8_t delta_count;    ///< Touch strength indicator (0-255)
    
    /**
     * @brief Default constructor
     */
    TouchEvent() = default;
    
    /**
     * @brief Constructor with parameters
     * @param ch Touch channel that generated the event
     * @param press true for press, false for release
     * @param time System timestamp in milliseconds
     * @param delta Touch strength indicator (0-255)
     */
    TouchEvent(TouchChannel ch, bool press, uint32_t time = 0, uint8_t delta = 0)
        : channel(ch), pressed(press), timestamp(time), delta_count(delta) {}
};

/**
 * @struct LEDConfig
 * @brief LED configuration for advanced effects
 *
 * Provides detailed control over LED behavior including timing,
 * duty cycles, and transition effects. Used for advanced LED
 * programming beyond simple on/off control.
 *
 * @note Period values are constrained to 32-2048ms by hardware.
 *       Values outside this range will be clamped.
 */
struct LEDConfig {
    LEDState state = LEDState::OFF;            ///< LED state/effect
    uint16_t pulse1_period_ms = 1000;          ///< Pulse 1 period in milliseconds (32-2048ms)
    uint16_t pulse2_period_ms = 1500;          ///< Pulse 2 period in milliseconds (32-2048ms)
    uint16_t breathe_period_ms = 2000;         ///< Breathe period in milliseconds (32-2048ms)
    uint8_t duty_cycle_percent = 50;           ///< Duty cycle as percentage (0-100%)
    uint8_t ramp_rate = 0;                     ///< Ramp rate for transitions (0-255)
    uint16_t off_delay_ms = 0;                 ///< Delay before turning off in milliseconds
    
    /**
     * @brief Default constructor
     */
    LEDConfig() = default;
    
    /**
     * @brief Constructor with LED state
     * @param st Initial LED state
     */
    LEDConfig(LEDState st) : state(st) {}
};

/**
 * @struct DeviceStatus
 * @brief Device status information
 *
 * Contains current status information about the CAP1188 device,
 * including touch detection, error conditions, and system health.
 *
 * @note This structure is populated by getStatus() and provides
 *       a snapshot of the device state at the time of the call.
 */
struct DeviceStatus {
    bool touch_detected = false;        ///< Any touch currently detected
    bool multiple_touch = false;        ///< Multiple touches detected
    bool noise_detected = false;        ///< Noise detected on inputs
    bool calibration_failed = false;    ///< Auto-calibration failure
    bool baseline_out_of_range = false; ///< Baseline count out of range
    uint8_t touched_channels = 0x00;    ///< Bitmask of touched channels (bit 0 = C1, bit 1 = C2, etc.)
    
    /**
     * @brief Default constructor
     */
    DeviceStatus() = default;
};

/**
 * @typedef TouchCallback
 * @brief Callback function type for touch events
 *
 * Function signature for touch event callbacks. Called whenever
 * a touch press or release event is detected.
 *
 * @param event TouchEvent structure containing event details
 */
using TouchCallback = void(*)(const TouchEvent& event);

/**
 * @typedef ErrorCallback
 * @brief Callback function type for error events
 *
 * Function signature for error callbacks. Called when errors
 * occur during operation.
 *
 * @param error Error code indicating the type of error
 * @param message Optional human-readable error message
 */
using ErrorCallback = void(*)(Error error, const char* message);

/**
 * @brief Check if a specific channel is touched in a status byte
 *
 * Utility function to check if a specific touch channel is active
 * in a status byte returned by getTouchedChannels().
 *
 * @param status_byte Status byte with touch channel bits
 * @param channel Touch channel to check
 * @return true if channel is touched, false otherwise
 */
constexpr bool isChannelTouched(uint8_t status_byte, TouchChannel channel) {
    return (status_byte & (1 << static_cast<uint8_t>(channel))) != 0;
}

/**
 * @brief Convert touch channel to bit mask
 *
 * Converts a TouchChannel enum value to its corresponding bit mask
 * for use in register operations.
 *
 * @param channel Touch channel to convert
 * @return Bit mask with the channel's bit set (0xFF for ALL)
 */
constexpr uint8_t channelToMask(TouchChannel channel) {
    if (channel == TouchChannel::ALL) {
        return 0xFF;
    }
    return (1 << static_cast<uint8_t>(channel));
}

/**
 * @brief Convert bit mask to touch channel
 *
 * Converts a bit mask to the first TouchChannel that has its bit set.
 * Used for processing touch status bytes.
 *
 * @param mask Bit mask to convert
 * @return First TouchChannel with its bit set (C1 as fallback)
 */
constexpr TouchChannel maskToChannel(uint8_t mask) {
    for (uint8_t i = 0; i < 8; ++i) {
        if (mask & (1 << i)) {
            return static_cast<TouchChannel>(i);
        }
    }
    return TouchChannel::C1; // Default fallback
}

/**
 * @brief Calculate threshold from base count and sensitivity percentage
 *
 * Calculates a touch threshold value as a percentage of the base count.
 * Used internally for automatic threshold calculation.
 *
 * @param base_count Base count reading from the channel
 * @param sensitivity_percent Sensitivity as percentage (0-100)
 * @return Calculated threshold value (1-255)
 */
constexpr uint8_t calculateThreshold(uint8_t base_count, uint8_t sensitivity_percent) {
    // Calculate threshold as percentage of base count
    // Clamp to valid range (1-255)
    uint16_t threshold = (static_cast<uint16_t>(base_count) * sensitivity_percent) / 100;
    if (threshold < 1) threshold = 1;
    if (threshold > 255) threshold = 255;
    return static_cast<uint8_t>(threshold);
}

/**
 * @brief Convert milliseconds to LED period register value
 *
 * Converts a time period in milliseconds to the corresponding
 * LED period register value. Hardware constraint: 32-2048ms.
 *
 * @param ms Time period in milliseconds
 * @return LED period register value (0-63)
 * @note Formula: register_value = (ms / 32) - 1
 */
constexpr uint8_t millisecondsToLEDPeriod(uint16_t ms) {
    // Convert milliseconds to LED period register value
    // Period = (register_value + 1) * 32ms
    // Clamp to valid range (32-2048ms)
    if (ms < 32) ms = 32;
    if (ms > 2048) ms = 2048;
    return static_cast<uint8_t>((ms / 32) - 1);
}

/**
 * @brief Convert LED period register value to milliseconds
 *
 * Converts an LED period register value back to milliseconds.
 * Used for reading current LED timing settings.
 *
 * @param period_reg LED period register value (0-63)
 * @return Time period in milliseconds
 * @note Formula: ms = (register_value + 1) * 32
 */
constexpr uint16_t ledPeriodToMilliseconds(uint8_t period_reg) {
    // Convert LED period register value to milliseconds
    return static_cast<uint16_t>((period_reg + 1) * 32);
}

/**
 * @brief Convert percentage to duty cycle register value
 *
 * Converts a duty cycle percentage (0-100%) to the corresponding
 * 8-bit register value (0-255).
 *
 * @param percent Duty cycle percentage (0-100)
 * @return Duty cycle register value (0-255)
 */
constexpr uint8_t percentToDutyCycle(uint8_t percent) {
    // Convert percentage (0-100) to duty cycle register value (0-255)
    if (percent > 100) percent = 100;
    return static_cast<uint8_t>((static_cast<uint16_t>(percent) * 255) / 100);
}

/**
 * @brief Convert duty cycle register value to percentage
 *
 * Converts an 8-bit duty cycle register value (0-255) back to
 * a percentage (0-100%).
 *
 * @param duty_cycle Duty cycle register value (0-255)
 * @return Duty cycle percentage (0-100)
 */
constexpr uint8_t dutyCycleToPercent(uint8_t duty_cycle) {
    // Convert duty cycle register value (0-255) to percentage (0-100)
    return static_cast<uint8_t>((static_cast<uint16_t>(duty_cycle) * 100) / 255);
}

/**
 * @struct ConfigChangeSet
 * @brief Configuration change detection structure
 *
 * Used internally to track which configuration parameters have changed
 * during runtime configuration updates. Enables efficient partial updates
 * that only modify changed settings.
 *
 * @note This structure uses bit fields to minimize memory usage and
 *       is primarily used by the smart configuration merging system.
 */
struct ConfigChangeSet {
    bool sensitivity_changed : 1;           ///< Touch sensitivity changed
    bool response_speed_changed : 1;        ///< Response speed changed
    bool stability_changed : 1;             ///< Stability setting changed
    bool noise_filtering_changed : 1;       ///< Noise filtering changed
    bool led_behavior_changed : 1;          ///< LED behavior changed
    bool led_speed_changed : 1;             ///< LED speed changed
    bool led_polarity_changed : 1;          ///< LED polarity changed
    bool multi_touch_changed : 1;           ///< Multi-touch mode changed
    bool interrupts_changed : 1;            ///< Interrupt settings changed
    bool power_settings_changed : 1;        ///< Power settings changed
    bool gain_changed : 1;                  ///< Gain setting changed
    bool delta_sense_changed : 1;           ///< Delta sense changed
    bool standby_sensitivity_changed : 1;   ///< Standby sensitivity changed
    bool auto_recalibration_changed : 1;    ///< Auto recalibration changed
    bool digital_noise_filter_changed : 1;  ///< Digital noise filter changed
    bool analog_noise_filter_changed : 1;   ///< Analog noise filter changed
    
    /**
     * @brief Default constructor
     * Initializes all change flags to false
     */
    ConfigChangeSet() : sensitivity_changed(false), response_speed_changed(false), 
                       stability_changed(false), noise_filtering_changed(false),
                       led_behavior_changed(false), led_speed_changed(false),
                       led_polarity_changed(false), multi_touch_changed(false),
                       interrupts_changed(false), power_settings_changed(false),
                       gain_changed(false), delta_sense_changed(false),
                       standby_sensitivity_changed(false), auto_recalibration_changed(false),
                       digital_noise_filter_changed(false), analog_noise_filter_changed(false) {}
    
    /**
     * @brief Check if any configuration has changed
     * @return true if any setting has changed, false otherwise
     */
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

/**
 * @example basic_touch.cpp
 * This example demonstrates basic touch detection using the CAP1188 library.
 * It shows how to initialize the device, configure touch sensitivity, and
 * monitor touch events in real-time.
 */