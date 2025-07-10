/**
 * @file cap1188.cpp
 * @brief Implementation of the CAP1188 capacitive touch sensor library
 *
 * This file contains the complete implementation of the CAP1188Device class
 * and all supporting functions. It provides comprehensive support for the
 * CAP1188 8-channel capacitive touch sensor including initialization,
 * touch detection, LED control, power management, and runtime configuration.
 *
 * @author CAP1188 Library Contributors
 * @date 2024
 * @version 0.0.1
 *
 * @note This implementation follows the Pre-Initialized I2C Pattern and
 *       requires the application to initialize I2C hardware before use.
 */

#include "cap1188/cap1188.hpp"
#include <cstring>
#include <stdio.h>

namespace CAP1188 {

/**
 * @brief Constructor for CAP1188Device
 * 
 * Initializes the device instance with the specified I2C interface and
 * device address. The device is not yet initialized - call begin() to
 * complete initialization.
 * 
 * @param i2c_instance Pointer to initialized I2C instance
 * @param device_address I2C address of the device
 * @param reset_pin GPIO pin for hardware reset (255 = no reset)
 */
CAP1188Device::CAP1188Device(i2c_inst_t* i2c_instance, uint8_t device_address,
                           uint reset_pin)
    : _i2c(i2c_instance)
    , _address(device_address)
    , _reset_pin(reset_pin)
    , _initialized(false)
    , _power_mode(PowerMode::ACTIVE)
    , _touch_callback(nullptr)
    , _error_callback(nullptr)
    , _last_touched_state(0x00)
{
    // Initialize channel configurations with defaults
    for (int i = 0; i < 8; ++i) {
        _channel_configs[i] = TouchConfig();
    }
}

/**
 * @brief Destructor for CAP1188Device
 * 
 * Cleans up the device instance. If the device was initialized,
 * it will be put into deep sleep mode to minimize power consumption.
 */
CAP1188Device::~CAP1188Device() {
    if (_initialized) {
        enterDeepSleep();
    }
}

/**
 * @brief Initialize the CAP1188 device
 * 
 * Performs complete device initialization including hardware reset
 * (if reset pin is connected), device verification, and default
 * configuration setup.
 * 
 * @return Error::SUCCESS on successful initialization
 * @return Error::INVALID_PARAMETER if I2C instance is null
 * @return Error::DEVICE_NOT_FOUND if device verification fails
 * @return Error::I2C_ERROR if I2C communication fails
 * @return Error::CONFIGURATION_ERROR if default configuration fails
 */
Error CAP1188Device::begin() {
    // Validate I2C is ready
    if (_i2c == nullptr) {
        return Error::INVALID_PARAMETER;
    }
    
    // Hardware reset if pin is connected
    if (_reset_pin < 255) {
        Error err = _hardwareReset();
        if (err != Error::SUCCESS) {
            return err;
        }
    }
    
    // Wait for device to be ready
    sleep_ms(POWER_ON_DELAY_MS);
    
    // Verify device identity
    Error err = _verifyDevice();
    if (err != Error::SUCCESS) {
        return err;
    }
    
    // Configure with defaults
    err = _configureDefaults();
    if (err != Error::SUCCESS) {
        return err;
    }
    
    _initialized = true;
    _last_update_time = get_absolute_time();
    
    return Error::SUCCESS;
}

Error CAP1188Device::reset() {
    if (_reset_pin < 255) {
        return _hardwareReset();
    } else {
        return _softwareReset();
    }
}

/**
 * @brief Check if device is connected and responding
 * 
 * Verifies device connection by reading and validating the product ID.
 * 
 * @return true if device is connected and responding
 * @return false if device is not detected or not responding
 */
bool CAP1188Device::isConnected() {
    uint8_t product_id;
    Error err = readRegister(REG_PRODUCT_ID, product_id);
    return (err == Error::SUCCESS && product_id == EXPECTED_PRODUCT_ID);
}

DeviceStatus CAP1188Device::getStatus() {
    DeviceStatus status;
    
    uint8_t general_status, touch_status, noise_status;
    
    if (readRegister(REG_GENERAL_STATUS, general_status) == Error::SUCCESS) {
        status.touch_detected = (general_status & GENERAL_STATUS_TOUCH) != 0;
        status.multiple_touch = (general_status & GENERAL_STATUS_MULT) != 0;
        status.calibration_failed = (general_status & GENERAL_STATUS_ACAL_FAIL) != 0;
        status.baseline_out_of_range = (general_status & GENERAL_STATUS_BC_OUT) != 0;
    }
    
    if (readRegister(REG_SENSOR_INPUT_STATUS, touch_status) == Error::SUCCESS) {
        status.touched_channels = touch_status;
    }
    
    if (readRegister(REG_NOISE_FLAG_STATUS, noise_status) == Error::SUCCESS) {
        status.noise_detected = (noise_status != 0);
    }
    
    return status;
}

Error CAP1188Device::setConfiguration(const DeviceConfig& config) {
    _config = config;
    
    // Apply human-readable settings by translating to register values
    
    // Configure touch sensitivity (if not using legacy gain setting)
    if (_config.gain == Gain::GAIN_1X) {  // Default gain indicates using new sensitivity
        uint8_t threshold = _sensitivityToThreshold(_config.sensitivity);
        // Apply to all channels that don't have custom thresholds
        for (int i = 0; i < 8; ++i) {
            if (_channel_configs[i].custom_threshold == 0) {
                Error err = setChannelThreshold(static_cast<TouchChannel>(i), threshold);
                if (err != Error::SUCCESS) return err;
            }
        }
    }
    
    // Configure response speed
    uint8_t repeat_rate = _responseSpeedToRepeatRate(_config.response_speed);
    Error err = writeRegister(REG_REPEAT_RATE, repeat_rate);
    if (err != Error::SUCCESS) return err;
    
    // Configure touch stability (averaging)
    uint8_t averaging = _stabilityToAveraging(_config.stability);
    err = writeRegister(REG_AVERAGING, averaging);
    if (err != Error::SUCCESS) return err;
    
    // Configure noise filtering
    uint8_t noise_config = _noiseFilteringToConfig(_config.noise_filtering);
    err = writeRegister(REG_CONFIGURATION, noise_config);
    if (err != Error::SUCCESS) return err;
    
    // Configure multi-touch
    uint8_t multi_config = _multiTouchToConfig(_config.multi_touch);
    err = writeRegister(REG_MULTIPLE_TOUCH_CONFIG, multi_config);
    if (err != Error::SUCCESS) return err;
    
    // Configure LED behavior (apply to all channels using global setting)
    uint8_t pulse1, pulse2, breathe;
    _ledBehaviorToRegisters(_config.led_behavior, _config.led_speed, pulse1, pulse2, breathe);
    
    // Apply LED timing to all channels (individual channel overrides handled in setChannelConfig)
    for (int i = 0; i < 8; ++i) {
        if (_channel_configs[i].led_behavior == LEDBehavior::USE_GLOBAL) {
            err = writeRegister(REG_LED_PULSE_1_PERIOD, pulse1);
            if (err != Error::SUCCESS) return err;
            err = writeRegister(REG_LED_PULSE_2_PERIOD, pulse2);
            if (err != Error::SUCCESS) return err;
            err = writeRegister(REG_LED_BREATHE_PERIOD, breathe);
            if (err != Error::SUCCESS) return err;
            break;  // Only need to write once for global settings
        }
    }
    
    // Update main control register
    err = _updateMainControl();
    if (err != Error::SUCCESS) return err;
    
    // Update sensor configuration  
    err = _updateSensorConfig();
    if (err != Error::SUCCESS) return err;
    
    // Configure LED polarity
    err = setLEDPolarity(_config.led_active_high);
    if (err != Error::SUCCESS) return err;
    
    // Configure interrupts
    if (_config.interrupts_enabled) {
        err = enableInterrupts(true);
    } else {
        err = disableInterrupts();
    }
    if (err != Error::SUCCESS) return err;
    
    // Configure standby sensitivity
    err = writeRegister(REG_STANDBY_SENSITIVITY, _config.standby_sensitivity);
    if (err != Error::SUCCESS) return err;
    
    return Error::SUCCESS;
}

DeviceConfig CAP1188Device::getConfiguration() const {
    return _config;
}

// Runtime configuration updates (granular)
/**
 * @brief Update touch sensitivity setting
 * 
 * Updates only the touch sensitivity setting without affecting
 * other configuration parameters. This is an atomic operation.
 * 
 * @param sensitivity New touch sensitivity level
 * @return Error::SUCCESS on successful update
 * @return Error::NOT_INITIALIZED if device is not initialized
 * @return Error::I2C_ERROR if I2C communication fails
 */
Error CAP1188Device::updateSensitivity(TouchSensitivity sensitivity) {
    if (!_initialized) return Error::NOT_INITIALIZED;
    
    Error err = _updateSensitivityRegisters(sensitivity);
    if (err == Error::SUCCESS) {
        _config.sensitivity = sensitivity;
    }
    return err;
}

Error CAP1188Device::updateResponseSpeed(TouchResponseSpeed speed) {
    if (!_initialized) return Error::NOT_INITIALIZED;
    
    Error err = _updateResponseSpeedRegisters(speed);
    if (err == Error::SUCCESS) {
        _config.response_speed = speed;
    }
    return err;
}

Error CAP1188Device::updateStability(TouchStability stability) {
    if (!_initialized) return Error::NOT_INITIALIZED;
    
    Error err = _updateStabilityRegisters(stability);
    if (err == Error::SUCCESS) {
        _config.stability = stability;
    }
    return err;
}

Error CAP1188Device::updateNoiseFiltering(NoiseFiltering filtering) {
    if (!_initialized) return Error::NOT_INITIALIZED;
    
    Error err = _updateNoiseFilteringRegisters(filtering);
    if (err == Error::SUCCESS) {
        _config.noise_filtering = filtering;
    }
    return err;
}

Error CAP1188Device::updateLEDBehavior(LEDBehavior behavior, LEDSpeed speed) {
    if (!_initialized) return Error::NOT_INITIALIZED;
    
    Error err = _updateLEDBehaviorRegisters(behavior, speed);
    if (err == Error::SUCCESS) {
        _config.led_behavior = behavior;
        _config.led_speed = speed;
    }
    return err;
}

Error CAP1188Device::updateMultiTouchMode(MultiTouchMode mode) {
    if (!_initialized) return Error::NOT_INITIALIZED;
    
    Error err = _updateMultiTouchRegisters(mode);
    if (err == Error::SUCCESS) {
        _config.multi_touch = mode;
        _config.multi_touch_enabled = (mode != MultiTouchMode::DISABLED);
    }
    return err;
}

// Runtime configuration updates (batch)
Error CAP1188Device::updateTouchSettings(TouchSensitivity sensitivity, TouchResponseSpeed speed, TouchStability stability) {
    if (!_initialized) return Error::NOT_INITIALIZED;
    
    // Apply all changes or none
    Error err = _updateSensitivityRegisters(sensitivity);
    if (err != Error::SUCCESS) return err;
    
    err = _updateResponseSpeedRegisters(speed);
    if (err != Error::SUCCESS) {
        // Rollback sensitivity
        _updateSensitivityRegisters(_config.sensitivity);
        return err;
    }
    
    err = _updateStabilityRegisters(stability);
    if (err != Error::SUCCESS) {
        // Rollback previous changes
        _updateSensitivityRegisters(_config.sensitivity);
        _updateResponseSpeedRegisters(_config.response_speed);
        return err;
    }
    
    // All successful, update config
    _config.sensitivity = sensitivity;
    _config.response_speed = speed;
    _config.stability = stability;
    
    return Error::SUCCESS;
}

Error CAP1188Device::updateLEDSettings(LEDBehavior behavior, LEDSpeed speed, bool active_high) {
    if (!_initialized) return Error::NOT_INITIALIZED;
    
    // Apply all changes or none
    Error err = _updateLEDBehaviorRegisters(behavior, speed);
    if (err != Error::SUCCESS) return err;
    
    err = setLEDPolarity(active_high);
    if (err != Error::SUCCESS) {
        // Rollback LED behavior
        _updateLEDBehaviorRegisters(_config.led_behavior, _config.led_speed);
        return err;
    }
    
    // All successful, update config
    _config.led_behavior = behavior;
    _config.led_speed = speed;
    _config.led_active_high = active_high;
    
    return Error::SUCCESS;
}

Error CAP1188Device::updateNoiseSettings(NoiseFiltering filtering, bool digital_filter, bool analog_filter) {
    if (!_initialized) return Error::NOT_INITIALIZED;
    
    // Apply all changes or none
    Error err = _updateNoiseFilteringRegisters(filtering);
    if (err != Error::SUCCESS) return err;
    
    // Update digital and analog noise filter settings
    bool prev_digital = _config.digital_noise_filter;
    bool prev_analog = _config.analog_noise_filter;
    
    _config.digital_noise_filter = digital_filter;
    _config.analog_noise_filter = analog_filter;
    
    err = _updateSensorConfig();
    if (err != Error::SUCCESS) {
        // Rollback all changes
        _updateNoiseFilteringRegisters(_config.noise_filtering);
        _config.digital_noise_filter = prev_digital;
        _config.analog_noise_filter = prev_analog;
        _updateSensorConfig();
        return err;
    }
    
    // All successful, update config
    _config.noise_filtering = filtering;
    
    return Error::SUCCESS;
}

Error CAP1188Device::updatePowerSettings(bool interrupts, bool deep_sleep) {
    if (!_initialized) return Error::NOT_INITIALIZED;
    
    // Apply interrupt setting
    Error err = enableInterrupts(interrupts);
    if (err != Error::SUCCESS) return err;
    
    // Handle deep sleep setting
    if (deep_sleep && !_config.deep_sleep_enabled) {
        err = enterDeepSleep();
    } else if (!deep_sleep && _config.deep_sleep_enabled) {
        err = exitDeepSleep();
    }
    
    if (err != Error::SUCCESS) {
        // Rollback interrupt setting
        enableInterrupts(_config.interrupts_enabled);
        return err;
    }
    
    // All successful, update config
    _config.interrupts_enabled = interrupts;
    _config.deep_sleep_enabled = deep_sleep;
    
    return Error::SUCCESS;
}

// Runtime configuration updates (smart merging)
Error CAP1188Device::updateConfiguration(const DeviceConfig& new_config, bool force_all) {
    if (!_initialized) return Error::NOT_INITIALIZED;
    
    // Validate the new configuration
    Error err = _validateDeviceConfig(new_config);
    if (err != Error::SUCCESS) return err;
    
    err = _validateConfigurationChange(_config, new_config);
    if (err != Error::SUCCESS) return err;
    
    if (force_all) {
        // Apply complete configuration (same as setConfiguration)
        return setConfiguration(new_config);
    }
    
    // Detect changes and apply only what's different
    ConfigChangeSet changes = _detectDeviceConfigChanges(_config, new_config);
    
    if (!changes.hasChanges()) {
        return Error::SUCCESS; // No changes needed
    }
    
    return _applyDeviceConfigChanges(new_config, changes);
}

// Per-channel runtime updates
Error CAP1188Device::updateChannelSensitivity(TouchChannel channel, TouchSensitivity sensitivity) {
    if (!_initialized) return Error::NOT_INITIALIZED;
    if (!_isValidChannel(channel) || channel == TouchChannel::ALL) {
        return Error::INVALID_PARAMETER;
    }
    
    uint8_t threshold = _sensitivityToThreshold(sensitivity);
    Error err = setChannelThreshold(channel, threshold);
    
    if (err == Error::SUCCESS) {
        _channel_configs[static_cast<uint8_t>(channel)].sensitivity = sensitivity;
        _channel_configs[static_cast<uint8_t>(channel)].custom_threshold = threshold;
    }
    
    return err;
}

Error CAP1188Device::updateChannelLEDBehavior(TouchChannel channel, LEDBehavior behavior) {
    if (!_initialized) return Error::NOT_INITIALIZED;
    if (!_isValidChannel(channel) || channel == TouchChannel::ALL) {
        return Error::INVALID_PARAMETER;
    }
    
    // For per-channel LED behavior, we need to configure LED linking
    // and update the channel's LED behavior setting
    Error err = Error::SUCCESS;
    
    if (behavior == LEDBehavior::OFF) {
        err = linkLEDToTouch(channel, false);
    } else {
        err = linkLEDToTouch(channel, true);
        // Note: Advanced LED behaviors would require additional register configuration
        // For now, we support basic on/off linking
    }
    
    if (err == Error::SUCCESS) {
        _channel_configs[static_cast<uint8_t>(channel)].led_behavior = behavior;
    }
    
    return err;
}

Error CAP1188Device::updateChannelConfiguration(TouchChannel channel, const TouchConfig& new_config, bool force_all) {
    if (!_initialized) return Error::NOT_INITIALIZED;
    if (!_isValidChannel(channel) || channel == TouchChannel::ALL) {
        return Error::INVALID_PARAMETER;
    }
    
    // Validate the new channel configuration
    Error err = _validateChannelConfig(channel, new_config);
    if (err != Error::SUCCESS) return err;
    
    if (force_all) {
        // Apply complete channel configuration
        return setChannelConfig(channel, new_config);
    }
    
    // Apply only changed settings
    uint8_t ch_idx = static_cast<uint8_t>(channel);
    TouchConfig& current = _channel_configs[ch_idx];
    
    // Check and update enabled state
    if (new_config.enabled != current.enabled) {
        err = enableChannel(channel, new_config.enabled);
        if (err != Error::SUCCESS) return err;
    }
    
    // Check and update threshold/sensitivity
    if (new_config.sensitivity != current.sensitivity || 
        new_config.threshold != current.threshold ||
        new_config.custom_threshold != current.custom_threshold) {
        
        uint8_t threshold = (new_config.custom_threshold > 0) ? 
                           new_config.custom_threshold : 
                           (new_config.sensitivity != TouchSensitivity::USE_GLOBAL) ?
                           _sensitivityToThreshold(new_config.sensitivity) : 
                           new_config.threshold;
        
        err = setChannelThreshold(channel, threshold);
        if (err != Error::SUCCESS) return err;
    }
    
    // Check and update LED linking
    if (new_config.linked_led != current.linked_led ||
        new_config.led_behavior != current.led_behavior) {
        err = linkLEDToTouch(channel, new_config.linked_led);
        if (err != Error::SUCCESS) return err;
    }
    
    // Update the stored configuration
    current = new_config;
    
    return Error::SUCCESS;
}

uint8_t CAP1188Device::getTouchedChannels() {
    uint8_t status = 0;
    readRegister(REG_SENSOR_INPUT_STATUS, status);
    return status;
}

bool CAP1188Device::isChannelTouched(TouchChannel channel) {
    if (!_isValidChannel(channel)) return false;
    
    uint8_t status = getTouchedChannels();
    return CAP1188::isChannelTouched(status, channel);
}

Error CAP1188Device::enableChannel(TouchChannel channel, bool enable) {
    if (!_isValidChannel(channel)) {
        return Error::INVALID_PARAMETER;
    }
    
    uint8_t enable_reg;
    Error err = readRegister(REG_SENSOR_INPUT_ENABLE, enable_reg);
    if (err != Error::SUCCESS) return err;
    
    if (channel == TouchChannel::ALL) {
        enable_reg = enable ? 0xFF : 0x00;
        for (int i = 0; i < 8; ++i) {
            _channel_configs[i].enabled = enable;
        }
    } else {
        uint8_t mask = channelToMask(channel);
        if (enable) {
            enable_reg |= mask;
        } else {
            enable_reg &= ~mask;
        }
        _channel_configs[static_cast<uint8_t>(channel)].enabled = enable;
    }
    
    return writeRegister(REG_SENSOR_INPUT_ENABLE, enable_reg);
}

Error CAP1188Device::disableChannel(TouchChannel channel) {
    return enableChannel(channel, false);
}

Error CAP1188Device::setChannelThreshold(TouchChannel channel, uint8_t threshold) {
    if (!_isValidChannel(channel) || !_isValidThreshold(threshold)) {
        return Error::INVALID_PARAMETER;
    }
    
    if (channel == TouchChannel::ALL) {
        // Set all channel thresholds
        for (int i = 0; i < 8; ++i) {
            uint8_t reg = REG_SENSOR_INPUT_1_THRESHOLD + i;
            Error err = writeRegister(reg, threshold);
            if (err != Error::SUCCESS) return err;
            _channel_configs[i].threshold = threshold;
        }
        return Error::SUCCESS;
    } else {
        uint8_t reg = _getThresholdRegister(channel);
        Error err = writeRegister(reg, threshold);
        if (err == Error::SUCCESS) {
            _channel_configs[static_cast<uint8_t>(channel)].threshold = threshold;
        }
        return err;
    }
}

uint8_t CAP1188Device::getChannelThreshold(TouchChannel channel) {
    if (!_isValidChannel(channel) || channel == TouchChannel::ALL) {
        return 0;
    }
    
    uint8_t threshold = 0;
    uint8_t reg = _getThresholdRegister(channel);
    readRegister(reg, threshold);
    return threshold;
}

Error CAP1188Device::setChannelConfig(TouchChannel channel, const TouchConfig& config) {
    if (!_isValidChannel(channel)) {
        return Error::INVALID_PARAMETER;
    }
    
    // Set threshold
    Error err = setChannelThreshold(channel, config.threshold);
    if (err != Error::SUCCESS) return err;
    
    // Set enable state
    err = enableChannel(channel, config.enabled);
    if (err != Error::SUCCESS) return err;
    
    // Set LED linking
    err = linkLEDToTouch(channel, config.linked_led);
    if (err != Error::SUCCESS) return err;
    
    // Store configuration
    if (channel == TouchChannel::ALL) {
        for (int i = 0; i < 8; ++i) {
            _channel_configs[i] = config;
        }
    } else {
        _channel_configs[static_cast<uint8_t>(channel)] = config;
    }
    
    return Error::SUCCESS;
}

TouchConfig CAP1188Device::getChannelConfig(TouchChannel channel) {
    if (!_isValidChannel(channel) || channel == TouchChannel::ALL) {
        return TouchConfig();
    }
    
    return _channel_configs[static_cast<uint8_t>(channel)];
}

Error CAP1188Device::calibrateChannel(TouchChannel channel) {
    if (!_isValidChannel(channel)) {
        return Error::INVALID_PARAMETER;
    }
    
    uint8_t cal_mask = (channel == TouchChannel::ALL) ? 0xFF : channelToMask(channel);
    return writeRegister(REG_CALIBRATION_ACTIVATE, cal_mask);
}

Error CAP1188Device::calibrateAllChannels() {
    return calibrateChannel(TouchChannel::ALL);
}

uint8_t CAP1188Device::getBaseCount(TouchChannel channel) {
    if (!_isValidChannel(channel) || channel == TouchChannel::ALL) {
        return 0;
    }
    
    uint8_t base_count = 0;
    uint8_t reg = _getBaseCountRegister(channel);
    readRegister(reg, base_count);
    return base_count;
}

Error CAP1188Device::setLEDState(TouchChannel channel, bool state) {
    if (!_isValidChannel(channel)) {
        return Error::INVALID_PARAMETER;
    }
    
    uint8_t led_control;
    Error err = readRegister(REG_LED_OUTPUT_CONTROL, led_control);
    if (err != Error::SUCCESS) return err;
    
    uint8_t mask = (channel == TouchChannel::ALL) ? 0xFF : channelToMask(channel);
    
    if (state) {
        led_control |= mask;
    } else {
        led_control &= ~mask;
    }
    
    return writeRegister(REG_LED_OUTPUT_CONTROL, led_control);
}

Error CAP1188Device::setLEDState(TouchChannel channel, LEDState state) {
    // For now, map LED states to simple on/off
    // Advanced LED effects would require additional implementation
    switch (state) {
        case LEDState::OFF:
            return setLEDState(channel, false);
        case LEDState::ON:
        case LEDState::PULSE_1:
        case LEDState::PULSE_2:
        case LEDState::BREATHE:
            return setLEDState(channel, true);
        default:
            return Error::INVALID_PARAMETER;
    }
}

Error CAP1188Device::setLEDPolarity(bool active_high) {
    uint8_t polarity = active_high ? 0xFF : 0x00;
    _config.led_active_high = active_high;
    return writeRegister(REG_LED_POLARITY, polarity);
}

Error CAP1188Device::linkLEDToTouch(TouchChannel channel, bool linked) {
    if (!_isValidChannel(channel)) {
        return Error::INVALID_PARAMETER;
    }
    
    uint8_t link_reg;
    Error err = readRegister(REG_SENSOR_INPUT_LED_LINKING, link_reg);
    if (err != Error::SUCCESS) return err;
    
    uint8_t mask = (channel == TouchChannel::ALL) ? 0xFF : channelToMask(channel);
    
    if (linked) {
        link_reg |= mask;
    } else {
        link_reg &= ~mask;
    }
    
    // Update local configuration
    if (channel == TouchChannel::ALL) {
        for (int i = 0; i < 8; ++i) {
            _channel_configs[i].linked_led = linked;
        }
    } else {
        _channel_configs[static_cast<uint8_t>(channel)].linked_led = linked;
    }
    
    return writeRegister(REG_SENSOR_INPUT_LED_LINKING, link_reg);
}

Error CAP1188Device::enterStandby() {
    uint8_t main_control;
    Error err = readRegister(REG_MAIN_CONTROL, main_control);
    if (err != Error::SUCCESS) return err;
    
    main_control |= MAIN_CONTROL_STBY;
    err = writeRegister(REG_MAIN_CONTROL, main_control);
    if (err == Error::SUCCESS) {
        _power_mode = PowerMode::STANDBY;
    }
    return err;
}

Error CAP1188Device::exitStandby() {
    uint8_t main_control;
    Error err = readRegister(REG_MAIN_CONTROL, main_control);
    if (err != Error::SUCCESS) return err;
    
    main_control &= ~MAIN_CONTROL_STBY;
    err = writeRegister(REG_MAIN_CONTROL, main_control);
    if (err == Error::SUCCESS) {
        _power_mode = PowerMode::ACTIVE;
    }
    return err;
}

Error CAP1188Device::enterDeepSleep() {
    uint8_t main_control;
    Error err = readRegister(REG_MAIN_CONTROL, main_control);
    if (err != Error::SUCCESS) return err;
    
    main_control |= MAIN_CONTROL_DSLEEP;
    err = writeRegister(REG_MAIN_CONTROL, main_control);
    if (err == Error::SUCCESS) {
        _power_mode = PowerMode::DEEP_SLEEP;
    }
    return err;
}

Error CAP1188Device::exitDeepSleep() {
    uint8_t main_control;
    Error err = readRegister(REG_MAIN_CONTROL, main_control);
    if (err != Error::SUCCESS) return err;
    
    main_control &= ~MAIN_CONTROL_DSLEEP;
    err = writeRegister(REG_MAIN_CONTROL, main_control);
    if (err == Error::SUCCESS) {
        _power_mode = PowerMode::ACTIVE;
    }
    return err;
}

PowerMode CAP1188Device::getPowerMode() const {
    return _power_mode;
}

Error CAP1188Device::enableInterrupts(bool enable) {
    uint8_t interrupt_enable = enable ? 0xFF : 0x00;
    _config.interrupts_enabled = enable;
    return writeRegister(REG_INTERRUPT_ENABLE, interrupt_enable);
}

Error CAP1188Device::disableInterrupts() {
    return enableInterrupts(false);
}

bool CAP1188Device::isInterruptPending() {
    uint8_t main_control;
    if (readRegister(REG_MAIN_CONTROL, main_control) != Error::SUCCESS) {
        return false;
    }
    return (main_control & MAIN_CONTROL_INT) != 0;
}

Error CAP1188Device::clearInterrupt() {
    uint8_t main_control;
    Error err = readRegister(REG_MAIN_CONTROL, main_control);
    if (err != Error::SUCCESS) return err;
    
    main_control &= ~MAIN_CONTROL_INT;
    return writeRegister(REG_MAIN_CONTROL, main_control);
}

void CAP1188Device::setTouchCallback(TouchCallback callback) {
    _touch_callback = callback;
}

void CAP1188Device::setErrorCallback(ErrorCallback callback) {
    _error_callback = callback;
}

Error CAP1188Device::enableMultiTouch(bool enable) {
    uint8_t multi_config = enable ? 0x00 : 0x80;  // 0x80 disables multiple touch
    _config.multi_touch_enabled = enable;
    return writeRegister(REG_MULTIPLE_TOUCH_CONFIG, multi_config);
}

Error CAP1188Device::setGain(Gain gain) {
    uint8_t main_control;
    Error err = readRegister(REG_MAIN_CONTROL, main_control);
    if (err != Error::SUCCESS) return err;
    
    // Clear existing gain bits and set new ones
    main_control &= ~0x60;  // Clear bits 5-6
    main_control |= static_cast<uint8_t>(gain);
    
    err = writeRegister(REG_MAIN_CONTROL, main_control);
    if (err == Error::SUCCESS) {
        _config.gain = gain;
    }
    return err;
}

Gain CAP1188Device::getGain() const {
    return _config.gain;
}

Error CAP1188Device::readRegister(uint8_t reg, uint8_t& value) {
    return _i2cRead(reg, value);
}

Error CAP1188Device::writeRegister(uint8_t reg, uint8_t value) {
    return _i2cWrite(reg, value);
}

uint8_t CAP1188Device::getProductID() {
    uint8_t id = 0;
    readRegister(REG_PRODUCT_ID, id);
    return id;
}

uint8_t CAP1188Device::getManufacturerID() {
    uint8_t id = 0;
    readRegister(REG_MANUFACTURER_ID, id);
    return id;
}

uint8_t CAP1188Device::getRevision() {
    uint8_t rev = 0;
    readRegister(REG_REVISION, rev);
    return rev;
}

const char* CAP1188Device::getErrorString(Error error) const {
    return errorToString(error);
}

// Configuration translation methods

uint8_t CAP1188Device::_sensitivityToThreshold(TouchSensitivity sensitivity) const {
    switch (sensitivity) {
        case TouchSensitivity::VERY_LOW:   return 0x80;  // High threshold = low sensitivity
        case TouchSensitivity::LOW:        return 0x60;
        case TouchSensitivity::MEDIUM:     return 0x40;  // Default
        case TouchSensitivity::HIGH:       return 0x20;
        case TouchSensitivity::VERY_HIGH:  return 0x10;  // Low threshold = high sensitivity
        default:                           return 0x40;  // Default to medium
    }
}

uint8_t CAP1188Device::_responseSpeedToRepeatRate(TouchResponseSpeed speed) const {
    switch (speed) {
        case TouchResponseSpeed::VERY_SLOW: return 0xFF;  // 560ms
        case TouchResponseSpeed::SLOW:      return 0x80;  // 280ms
        case TouchResponseSpeed::MEDIUM:    return 0x35;  // 140ms (default)
        case TouchResponseSpeed::FAST:      return 0x1A;  // 70ms
        case TouchResponseSpeed::VERY_FAST: return 0x0D;  // 35ms
        default:                            return 0x35;  // Default to medium
    }
}

uint8_t CAP1188Device::_stabilityToAveraging(TouchStability stability) const {
    switch (stability) {
        case TouchStability::INSTANT:     return 0x00;  // No averaging
        case TouchStability::FAST:        return 0x01;  // 2 samples
        case TouchStability::BALANCED:    return 0x03;  // 4 samples (default)
        case TouchStability::STABLE:      return 0x07;  // 8 samples
        case TouchStability::VERY_STABLE: return 0x0F;  // 16 samples
        default:                          return 0x03;  // Default to balanced
    }
}

uint8_t CAP1188Device::_noiseFilteringToConfig(NoiseFiltering filtering) const {
    switch (filtering) {
        case NoiseFiltering::DISABLED: return 0x00;  // No filtering
        case NoiseFiltering::LIGHT:    return 0x01;  // Light filtering
        case NoiseFiltering::MEDIUM:   return 0x03;  // Standard (default)
        case NoiseFiltering::HEAVY:    return 0x07;  // Heavy filtering
        case NoiseFiltering::MAXIMUM:  return 0x0F;  // Maximum filtering
        default:                       return 0x03;  // Default to medium
    }
}

uint8_t CAP1188Device::_multiTouchToConfig(MultiTouchMode mode) const {
    switch (mode) {
        case MultiTouchMode::DISABLED:       return 0x80;  // Block multiple touch
        case MultiTouchMode::ENABLED:        return 0x00;  // Allow all
        case MultiTouchMode::TWO_FINGER_MAX: return 0x00;  // Allow all (limit handled elsewhere)
        case MultiTouchMode::GESTURE_MODE:   return 0x00;  // Allow all with gesture detection
        default:                             return 0x80;  // Default to disabled
    }
}

void CAP1188Device::_ledBehaviorToRegisters(LEDBehavior behavior, LEDSpeed speed, 
                                           uint8_t& pulse1, uint8_t& pulse2, uint8_t& breathe) const {
    // Base timing values for different speeds
    uint8_t base_timing;
    switch (speed) {
        case LEDSpeed::VERY_SLOW: base_timing = 0xFF; break;  // 2 seconds
        case LEDSpeed::SLOW:      base_timing = 0x80; break;  // 1 second
        case LEDSpeed::MEDIUM:    base_timing = 0x40; break;  // 0.5 seconds (default)
        case LEDSpeed::FAST:      base_timing = 0x20; break;  // 0.25 seconds
        case LEDSpeed::VERY_FAST: base_timing = 0x10; break;  // 0.125 seconds
        default:                  base_timing = 0x40; break;  // Default to medium
    }
    
    // Configure timing based on behavior
    switch (behavior) {
        case LEDBehavior::OFF:
        case LEDBehavior::TOUCH_FEEDBACK:
            pulse1 = 0x00;
            pulse2 = 0x00;
            breathe = 0x00;
            break;
        case LEDBehavior::PULSE_ON_TOUCH:
            pulse1 = base_timing;
            pulse2 = 0x00;
            breathe = 0x00;
            break;
        case LEDBehavior::BREATHE_ON_TOUCH:
            pulse1 = 0x00;
            pulse2 = 0x00;
            breathe = base_timing;
            break;
        case LEDBehavior::FADE_ON_RELEASE:
            pulse1 = base_timing / 2;
            pulse2 = base_timing;
            breathe = 0x00;
            break;
        default:
            pulse1 = 0x00;
            pulse2 = 0x00;
            breathe = 0x00;
            break;
    }
}

// Private implementation methods

Error CAP1188Device::_verifyDevice() {
    uint8_t product_id, manufacturer_id, revision;
    
    Error err = readRegister(REG_PRODUCT_ID, product_id);
    if (err != Error::SUCCESS) return err;
    
    err = readRegister(REG_MANUFACTURER_ID, manufacturer_id);
    if (err != Error::SUCCESS) return err;
    
    err = readRegister(REG_REVISION, revision);
    if (err != Error::SUCCESS) return err;
    
    if (product_id != EXPECTED_PRODUCT_ID ||
        manufacturer_id != EXPECTED_MANUFACTURER_ID ||
        revision != EXPECTED_REVISION) {
        return Error::DEVICE_NOT_FOUND;
    }
    
    return Error::SUCCESS;
}

Error CAP1188Device::_configureDefaults() {
    // Set default configuration
    DeviceConfig default_config;
    return setConfiguration(default_config);
}


Error CAP1188Device::_hardwareReset() {
    if (_reset_pin >= 255) {
        return Error::INVALID_PARAMETER;
    }
    
    // Configure reset pin as output
    gpio_init(_reset_pin);
    gpio_set_dir(_reset_pin, GPIO_OUT);
    
    // Perform reset sequence
    gpio_put(_reset_pin, false);  // Assert reset
    sleep_ms(1);                  // Hold for 1ms minimum
    gpio_put(_reset_pin, true);   // Release reset
    sleep_ms(RESET_DELAY_MS);     // Wait for device ready
    
    return Error::SUCCESS;
}

Error CAP1188Device::_softwareReset() {
    // CAP1188 doesn't have a software reset command
    // Best we can do is reconfigure to defaults
    return _configureDefaults();
}

Error CAP1188Device::_i2cWrite(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    int result = i2c_write_blocking(_i2c, _address, data, 2, false);
    return (result == 2) ? Error::SUCCESS : Error::I2C_ERROR;
}

Error CAP1188Device::_i2cRead(uint8_t reg, uint8_t& value) {
    int result = i2c_write_blocking(_i2c, _address, &reg, 1, true);
    if (result != 1) return Error::I2C_ERROR;
    
    result = i2c_read_blocking(_i2c, _address, &value, 1, false);
    return (result == 1) ? Error::SUCCESS : Error::I2C_ERROR;
}

Error CAP1188Device::_updateMainControl() {
    uint8_t main_control = 0;
    
    // Set gain
    main_control |= static_cast<uint8_t>(_config.gain);
    
    // Set power mode bits if needed
    if (_power_mode == PowerMode::STANDBY) {
        main_control |= MAIN_CONTROL_STBY;
    } else if (_power_mode == PowerMode::DEEP_SLEEP) {
        main_control |= MAIN_CONTROL_DSLEEP;
    }
    
    return writeRegister(REG_MAIN_CONTROL, main_control);
}

Error CAP1188Device::_updateSensorConfig() {
    uint8_t config = 0;
    
    // Configure noise filtering
    if (!_config.digital_noise_filter) {
        config |= CONFIG_DIS_DIG_NOISE;
    }
    if (!_config.analog_noise_filter) {
        config |= CONFIG_DIS_ANA_NOISE;
    }
    
    // Configure timeout
    if (_config.timeout_enabled) {
        config |= CONFIG_TIMEOUT;
    }
    
    // Configure maximum duration
    if (_config.max_duration_enabled) {
        config |= CONFIG_MAX_DUR_EN;
    }
    
    return writeRegister(REG_CONFIGURATION, config);
}

uint8_t CAP1188Device::_getThresholdRegister(TouchChannel channel) const {
    return REG_SENSOR_INPUT_1_THRESHOLD + static_cast<uint8_t>(channel);
}

uint8_t CAP1188Device::_getBaseCountRegister(TouchChannel channel) const {
    return REG_SENSOR_INPUT_1_BASE_COUNT + static_cast<uint8_t>(channel);
}

bool CAP1188Device::_isValidChannel(TouchChannel channel) const {
    return (channel >= TouchChannel::C1 && channel <= TouchChannel::C8) || 
           (channel == TouchChannel::ALL);
}

bool CAP1188Device::_isValidThreshold(uint8_t threshold) const {
    return threshold > 0;  // 0 would disable touch detection
}

void CAP1188Device::_callErrorCallback(Error error, const char* message) {
    if (_error_callback) {
        _error_callback(error, message);
    }
}

// Global utility functions
const char* errorToString(Error error) {
    switch (error) {
        case Error::SUCCESS: return "Success";
        case Error::I2C_ERROR: return "I2C communication error";
        case Error::DEVICE_NOT_FOUND: return "Device not found";
        case Error::INVALID_PARAMETER: return "Invalid parameter";
        case Error::TIMEOUT: return "Operation timeout";
        case Error::NOT_INITIALIZED: return "Device not initialized";
        case Error::CONFIGURATION_ERROR: return "Configuration error";
        default: return "Unknown error";
    }
}

const char* channelToString(TouchChannel channel) {
    switch (channel) {
        case TouchChannel::C1: return "C1";
        case TouchChannel::C2: return "C2";
        case TouchChannel::C3: return "C3";
        case TouchChannel::C4: return "C4";
        case TouchChannel::C5: return "C5";
        case TouchChannel::C6: return "C6";
        case TouchChannel::C7: return "C7";
        case TouchChannel::C8: return "C8";
        case TouchChannel::ALL: return "ALL";
        default: return "Unknown";
    }
}

const char* ledStateToString(LEDState state) {
    switch (state) {
        case LEDState::OFF: return "OFF";
        case LEDState::ON: return "ON";
        case LEDState::PULSE_1: return "PULSE_1";
        case LEDState::PULSE_2: return "PULSE_2";
        case LEDState::BREATHE: return "BREATHE";
        default: return "Unknown";
    }
}

const char* powerModeToString(PowerMode mode) {
    switch (mode) {
        case PowerMode::ACTIVE: return "ACTIVE";
        case PowerMode::STANDBY: return "STANDBY";
        case PowerMode::DEEP_SLEEP: return "DEEP_SLEEP";
        default: return "Unknown";
    }
}

TouchChannel indexToChannel(uint8_t index) {
    if (index < 8) {
        return static_cast<TouchChannel>(index);
    }
    return TouchChannel::C1;
}

uint8_t channelToIndex(TouchChannel channel) {
    if (channel >= TouchChannel::C1 && channel <= TouchChannel::C8) {
        return static_cast<uint8_t>(channel);
    }
    return 0;
}

// Runtime configuration update helper methods

Error CAP1188Device::_updateSensitivityRegisters(TouchSensitivity sensitivity) {
    uint8_t threshold = _sensitivityToThreshold(sensitivity);
    
    // Apply to all channels that don't have custom thresholds
    for (int i = 0; i < 8; ++i) {
        if (_channel_configs[i].custom_threshold == 0) {
            Error err = setChannelThreshold(static_cast<TouchChannel>(i), threshold);
            if (err != Error::SUCCESS) return err;
        }
    }
    
    return Error::SUCCESS;
}

Error CAP1188Device::_updateResponseSpeedRegisters(TouchResponseSpeed speed) {
    uint8_t repeat_rate = _responseSpeedToRepeatRate(speed);
    return writeRegister(REG_REPEAT_RATE, repeat_rate);
}

Error CAP1188Device::_updateStabilityRegisters(TouchStability stability) {
    uint8_t averaging = _stabilityToAveraging(stability);
    return writeRegister(REG_AVERAGING, averaging);
}

Error CAP1188Device::_updateNoiseFilteringRegisters(NoiseFiltering filtering) {
    uint8_t noise_config = _noiseFilteringToConfig(filtering);
    return writeRegister(REG_CONFIGURATION, noise_config);
}

Error CAP1188Device::_updateLEDBehaviorRegisters(LEDBehavior behavior, LEDSpeed speed) {
    uint8_t pulse1, pulse2, breathe;
    _ledBehaviorToRegisters(behavior, speed, pulse1, pulse2, breathe);
    
    // Write LED timing registers
    Error err = writeRegister(REG_LED_PULSE_1_PERIOD, pulse1);
    if (err != Error::SUCCESS) return err;
    
    err = writeRegister(REG_LED_PULSE_2_PERIOD, pulse2);
    if (err != Error::SUCCESS) return err;
    
    err = writeRegister(REG_LED_BREATHE_PERIOD, breathe);
    if (err != Error::SUCCESS) return err;
    
    return Error::SUCCESS;
}

Error CAP1188Device::_updateMultiTouchRegisters(MultiTouchMode mode) {
    uint8_t multi_config = _multiTouchToConfig(mode);
    return writeRegister(REG_MULTIPLE_TOUCH_CONFIG, multi_config);
}

// Configuration validation methods

Error CAP1188Device::_validateDeviceConfig(const DeviceConfig& config) const {
    // Check for valid enum values and logical combinations
    // reset_pin is uint8_t so valid range is 0-255, with 255 meaning disabled
    
    // Add more validation as needed
    return Error::SUCCESS;
}

Error CAP1188Device::_validateChannelConfig(TouchChannel channel, const TouchConfig& config) const {
    if (!_isValidChannel(channel)) {
        return Error::INVALID_PARAMETER;
    }
    
    if (!_isValidThreshold(config.threshold)) {
        return Error::INVALID_PARAMETER;
    }
    
    return Error::SUCCESS;
}

Error CAP1188Device::_validateConfigurationChange(const DeviceConfig& old_config, const DeviceConfig& new_config) const {
    // Check for potentially disruptive changes
    if (old_config.reset_pin != new_config.reset_pin) {
        // Changing reset pin configuration during runtime is not recommended
        return Error::CONFIGURATION_ERROR;
    }
    
    return Error::SUCCESS;
}

// Change detection methods

ConfigChangeSet CAP1188Device::_detectDeviceConfigChanges(const DeviceConfig& old_config, const DeviceConfig& new_config) const {
    ConfigChangeSet changes;
    
    changes.sensitivity_changed = (old_config.sensitivity != new_config.sensitivity);
    changes.response_speed_changed = (old_config.response_speed != new_config.response_speed);
    changes.stability_changed = (old_config.stability != new_config.stability);
    changes.noise_filtering_changed = (old_config.noise_filtering != new_config.noise_filtering);
    changes.led_behavior_changed = (old_config.led_behavior != new_config.led_behavior);
    changes.led_speed_changed = (old_config.led_speed != new_config.led_speed);
    changes.led_polarity_changed = (old_config.led_active_high != new_config.led_active_high);
    changes.multi_touch_changed = (old_config.multi_touch != new_config.multi_touch);
    changes.interrupts_changed = (old_config.interrupts_enabled != new_config.interrupts_enabled);
    changes.power_settings_changed = (old_config.deep_sleep_enabled != new_config.deep_sleep_enabled);
    changes.gain_changed = (old_config.gain != new_config.gain);
    changes.delta_sense_changed = (old_config.delta_sense != new_config.delta_sense);
    changes.standby_sensitivity_changed = (old_config.standby_sensitivity != new_config.standby_sensitivity);
    changes.auto_recalibration_changed = (old_config.auto_recalibration != new_config.auto_recalibration);
    changes.digital_noise_filter_changed = (old_config.digital_noise_filter != new_config.digital_noise_filter);
    changes.analog_noise_filter_changed = (old_config.analog_noise_filter != new_config.analog_noise_filter);
    
    return changes;
}

Error CAP1188Device::_applyDeviceConfigChanges(const DeviceConfig& config, const ConfigChangeSet& changes) {
    Error err = Error::SUCCESS;
    
    // Apply changes in logical order to minimize disruption
    
    if (changes.sensitivity_changed) {
        err = _updateSensitivityRegisters(config.sensitivity);
        if (err != Error::SUCCESS) return err;
        _config.sensitivity = config.sensitivity;
    }
    
    if (changes.response_speed_changed) {
        err = _updateResponseSpeedRegisters(config.response_speed);
        if (err != Error::SUCCESS) return err;
        _config.response_speed = config.response_speed;
    }
    
    if (changes.stability_changed) {
        err = _updateStabilityRegisters(config.stability);
        if (err != Error::SUCCESS) return err;
        _config.stability = config.stability;
    }
    
    if (changes.noise_filtering_changed || changes.digital_noise_filter_changed || 
        changes.analog_noise_filter_changed) {
        _config.noise_filtering = config.noise_filtering;
        _config.digital_noise_filter = config.digital_noise_filter;
        _config.analog_noise_filter = config.analog_noise_filter;
        err = _updateSensorConfig();
        if (err != Error::SUCCESS) return err;
    }
    
    if (changes.led_behavior_changed || changes.led_speed_changed) {
        err = _updateLEDBehaviorRegisters(config.led_behavior, config.led_speed);
        if (err != Error::SUCCESS) return err;
        _config.led_behavior = config.led_behavior;
        _config.led_speed = config.led_speed;
    }
    
    if (changes.led_polarity_changed) {
        err = setLEDPolarity(config.led_active_high);
        if (err != Error::SUCCESS) return err;
        _config.led_active_high = config.led_active_high;
    }
    
    if (changes.multi_touch_changed) {
        err = _updateMultiTouchRegisters(config.multi_touch);
        if (err != Error::SUCCESS) return err;
        _config.multi_touch = config.multi_touch;
        _config.multi_touch_enabled = (config.multi_touch != MultiTouchMode::DISABLED);
    }
    
    if (changes.interrupts_changed) {
        err = enableInterrupts(config.interrupts_enabled);
        if (err != Error::SUCCESS) return err;
        _config.interrupts_enabled = config.interrupts_enabled;
    }
    
    if (changes.power_settings_changed) {
        if (config.deep_sleep_enabled && !_config.deep_sleep_enabled) {
            err = enterDeepSleep();
        } else if (!config.deep_sleep_enabled && _config.deep_sleep_enabled) {
            err = exitDeepSleep();
        }
        if (err != Error::SUCCESS) return err;
        _config.deep_sleep_enabled = config.deep_sleep_enabled;
    }
    
    if (changes.gain_changed) {
        err = setGain(config.gain);
        if (err != Error::SUCCESS) return err;
        _config.gain = config.gain;
    }
    
    if (changes.standby_sensitivity_changed) {
        err = writeRegister(REG_STANDBY_SENSITIVITY, config.standby_sensitivity);
        if (err != Error::SUCCESS) return err;
        _config.standby_sensitivity = config.standby_sensitivity;
    }
    
    // Update remaining config values that don't require register writes
    if (changes.auto_recalibration_changed) {
        _config.auto_recalibration = config.auto_recalibration;
    }
    
    return Error::SUCCESS;
}

} // namespace CAP1188