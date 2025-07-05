#include "cap1188/cap1188.hpp"
#include <cstring>
#include <stdio.h>

namespace CAP1188 {

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

CAP1188Device::~CAP1188Device() {
    if (_initialized) {
        enterDeepSleep();
    }
}

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
    
    // Update main control register
    Error err = _updateMainControl();
    if (err != Error::SUCCESS) return err;
    
    // Update sensor configuration
    err = _updateSensorConfig();
    if (err != Error::SUCCESS) return err;
    
    // Configure LED polarity
    err = setLEDPolarity(_config.led_active_high);
    if (err != Error::SUCCESS) return err;
    
    // Configure multiple touch
    err = enableMultiTouch(_config.multi_touch_enabled);
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

} // namespace CAP1188