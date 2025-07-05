#pragma once

#include "cap1188_registers.hpp"
#include "cap1188_types.hpp"

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

namespace CAP1188 {

class CAP1188Device {
public:
    // Constructor
    explicit CAP1188Device(i2c_inst_t* i2c_instance, 
                          uint8_t device_address = DEFAULT_I2C_ADDRESS,
                          uint reset_pin = 255); // 255 = no reset pin (disabled)

    // Destructor
    ~CAP1188Device();

    // Device initialization and management
    Error begin();
    Error reset();
    bool isConnected();
    DeviceStatus getStatus();
    Error setConfiguration(const DeviceConfig& config);
    DeviceConfig getConfiguration() const;
    
    // Runtime configuration updates (granular)
    Error updateSensitivity(TouchSensitivity sensitivity);
    Error updateResponseSpeed(TouchResponseSpeed speed);
    Error updateStability(TouchStability stability);
    Error updateNoiseFiltering(NoiseFiltering filtering);
    Error updateLEDBehavior(LEDBehavior behavior, LEDSpeed speed = LEDSpeed::MEDIUM);
    Error updateMultiTouchMode(MultiTouchMode mode);
    
    // Runtime configuration updates (batch)
    Error updateTouchSettings(TouchSensitivity sensitivity, TouchResponseSpeed speed, TouchStability stability);
    Error updateLEDSettings(LEDBehavior behavior, LEDSpeed speed, bool active_high);
    Error updateNoiseSettings(NoiseFiltering filtering, bool digital_filter, bool analog_filter);
    Error updatePowerSettings(bool interrupts, bool deep_sleep);
    
    // Runtime configuration updates (smart merging)
    Error updateConfiguration(const DeviceConfig& new_config, bool force_all = false);
    
    // Per-channel runtime updates
    Error updateChannelSensitivity(TouchChannel channel, TouchSensitivity sensitivity);
    Error updateChannelLEDBehavior(TouchChannel channel, LEDBehavior behavior);
    Error updateChannelConfiguration(TouchChannel channel, const TouchConfig& new_config, bool force_all = false);

    // Touch sensing functions
    uint8_t getTouchedChannels();
    bool isChannelTouched(TouchChannel channel);
    Error enableChannel(TouchChannel channel, bool enable = true);
    Error disableChannel(TouchChannel channel);
    
    // Threshold and sensitivity control
    Error setChannelThreshold(TouchChannel channel, uint8_t threshold);
    uint8_t getChannelThreshold(TouchChannel channel);
    Error setChannelConfig(TouchChannel channel, const TouchConfig& config);
    TouchConfig getChannelConfig(TouchChannel channel);
    
    // Calibration functions
    Error calibrateChannel(TouchChannel channel);
    Error calibrateAllChannels();
    uint8_t getBaseCount(TouchChannel channel);
    
    // LED control functions
    Error setLEDState(TouchChannel channel, bool state);
    Error setLEDState(TouchChannel channel, LEDState state);
    Error setLEDConfig(TouchChannel channel, const LEDConfig& config);
    Error setLEDPolarity(bool active_high);
    Error linkLEDToTouch(TouchChannel channel, bool linked = true);
    
    // Advanced LED effects
    Error setLEDPulse(TouchChannel channel, uint8_t period, uint8_t duty_cycle);
    Error setLEDBreathe(TouchChannel channel, uint8_t period, uint8_t duty_cycle);
    Error setLEDDirectControl(TouchChannel channel, uint8_t brightness);
    
    // Power management
    Error enterStandby();
    Error exitStandby();
    Error enterDeepSleep();
    Error exitDeepSleep();
    PowerMode getPowerMode() const;
    
    // Interrupt handling
    Error enableInterrupts(bool enable = true);
    Error disableInterrupts();
    bool isInterruptPending();
    Error clearInterrupt();
    void setTouchCallback(TouchCallback callback);
    void setErrorCallback(ErrorCallback callback);
    
    // Multiple touch configuration
    Error enableMultiTouch(bool enable = true);
    Error setMultiTouchConfig(uint8_t config);
    
    // Noise filtering
    Error enableDigitalNoiseFilter(bool enable = true);
    Error enableAnalogNoiseFilter(bool enable = true);
    Error setNoiseThreshold(uint8_t threshold);
    
    // Gain and sensitivity
    Error setGain(Gain gain);
    Gain getGain() const;
    Error setDeltaSense(DeltaSense sense);
    DeltaSense getDeltaSense() const;
    
    // Low-level register access
    Error readRegister(uint8_t reg, uint8_t& value);
    Error writeRegister(uint8_t reg, uint8_t value);
    Error readMultipleRegisters(uint8_t start_reg, uint8_t* buffer, size_t length);
    Error writeMultipleRegisters(uint8_t start_reg, const uint8_t* data, size_t length);
    
    // Utility functions
    const char* getErrorString(Error error) const;
    void printStatus() const;
    void printConfiguration() const;
    
    // Device information
    uint8_t getProductID();
    uint8_t getManufacturerID();
    uint8_t getRevision();
    
private:
    // Hardware interface
    i2c_inst_t* _i2c;
    uint8_t _address;
    uint _reset_pin;
    
    // State tracking
    bool _initialized;
    DeviceConfig _config;
    TouchConfig _channel_configs[8];
    PowerMode _power_mode;
    
    // Callbacks
    TouchCallback _touch_callback;
    ErrorCallback _error_callback;
    
    // Last known state for change detection
    uint8_t _last_touched_state;
    absolute_time_t _last_update_time;
    
    // Internal helper functions
    Error _verifyDevice();
    Error _configureDefaults();
    Error _hardwareReset();
    Error _softwareReset();
    
    // I2C communication helpers
    Error _i2cWrite(uint8_t reg, uint8_t value);
    Error _i2cRead(uint8_t reg, uint8_t& value);
    Error _i2cWriteMultiple(uint8_t reg, const uint8_t* data, size_t length);
    Error _i2cReadMultiple(uint8_t reg, uint8_t* buffer, size_t length);
    
    // Configuration helpers
    uint8_t _getThresholdRegister(TouchChannel channel) const;
    uint8_t _getBaseCountRegister(TouchChannel channel) const;
    Error _updateMainControl();
    Error _updateSensorConfig();
    
    // Runtime configuration update helpers
    Error _updateSensitivityRegisters(TouchSensitivity sensitivity);
    Error _updateResponseSpeedRegisters(TouchResponseSpeed speed);
    Error _updateStabilityRegisters(TouchStability stability);
    Error _updateNoiseFilteringRegisters(NoiseFiltering filtering);
    Error _updateLEDBehaviorRegisters(LEDBehavior behavior, LEDSpeed speed);
    Error _updateMultiTouchRegisters(MultiTouchMode mode);
    
    // Configuration validation
    Error _validateDeviceConfig(const DeviceConfig& config) const;
    Error _validateChannelConfig(TouchChannel channel, const TouchConfig& config) const;
    Error _validateConfigurationChange(const DeviceConfig& old_config, const DeviceConfig& new_config) const;
    
    // Change detection
    ConfigChangeSet _detectDeviceConfigChanges(const DeviceConfig& old_config, const DeviceConfig& new_config) const;
    Error _applyDeviceConfigChanges(const DeviceConfig& config, const ConfigChangeSet& changes);
    
    // Configuration translation methods (human-readable to register values)
    uint8_t _sensitivityToThreshold(TouchSensitivity sensitivity) const;
    uint8_t _responseSpeedToRepeatRate(TouchResponseSpeed speed) const;
    uint8_t _stabilityToAveraging(TouchStability stability) const;
    uint8_t _noiseFilteringToConfig(NoiseFiltering filtering) const;
    uint8_t _multiTouchToConfig(MultiTouchMode mode) const;
    void _ledBehaviorToRegisters(LEDBehavior behavior, LEDSpeed speed, uint8_t& pulse1, uint8_t& pulse2, uint8_t& breathe) const;
    
    // State management
    void _updateTouchState();
    void _processTouchEvents(uint8_t current_state);
    void _callErrorCallback(Error error, const char* message = nullptr);
    
    // Validation helpers
    bool _isValidChannel(TouchChannel channel) const;
    bool _isValidThreshold(uint8_t threshold) const;
    
    // Debug helpers
    void _debugPrint(const char* message) const;
    void _debugPrintRegister(uint8_t reg, uint8_t value) const;
};

// Global utility functions
const char* errorToString(Error error);
const char* channelToString(TouchChannel channel);
const char* ledStateToString(LEDState state);
const char* powerModeToString(PowerMode mode);

// Conversion helpers
TouchChannel indexToChannel(uint8_t index);
uint8_t channelToIndex(TouchChannel channel);

} // namespace CAP1188