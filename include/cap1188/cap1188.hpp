/**
 * @file cap1188.hpp
 * @brief Main header file for the CAP1188 capacitive touch sensor library
 *
 * This is the primary header file that provides the CAP1188Device class
 * and all necessary functionality for interfacing with the CAP1188
 * 8-channel capacitive touch sensor.
 *
 * @author CAP1188 Library Contributors
 * @date 2024
 * @version 0.0.1
 *
 * @section Usage Basic Usage
 * @code
 * #include "cap1188/cap1188.hpp"
 *
 * using namespace CAP1188;
 *
 * // Initialize I2C hardware
 * i2c_init(i2c_default, 100000);
 * gpio_set_function(4, GPIO_FUNC_I2C);
 * gpio_set_function(5, GPIO_FUNC_I2C);
 * gpio_pull_up(4);
 * gpio_pull_up(5);
 *
 * // Create device instance
 * CAP1188Device touch_sensor(i2c_default);
 *
 * // Initialize device
 * if (touch_sensor.begin() == Error::SUCCESS) {
 *     // Read touch status
 *     uint8_t touched = touch_sensor.getTouchedChannels();
 *     if (touched) {
 *         printf("Touch detected on channels: 0x%02X\n", touched);
 *     }
 * }
 * @endcode
 */

#pragma once

#include "cap1188_registers.hpp"
#include "cap1188_types.hpp"

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

/**
 * @namespace CAP1188
 * @brief Main namespace for the CAP1188 capacitive touch sensor library
 */
namespace CAP1188 {

/**
 * @class CAP1188Device
 * @brief Main class for interfacing with the CAP1188 capacitive touch sensor
 *
 * This class provides a comprehensive C++ interface for the CAP1188 8-channel
 * capacitive touch sensor. It supports all major features including touch
 * detection, LED control, power management, and runtime configuration updates.
 *
 * The class follows the Pre-Initialized I2C Pattern, requiring the application
 * to initialize I2C hardware before device initialization.
 *
 * @note This class is designed to be used with the Raspberry Pi Pico SDK
 *       and requires proper I2C initialization before use.
 */
class CAP1188Device {
public:
    /**
     * @brief Constructor for CAP1188Device
     *
     * Creates a new CAP1188Device instance. The I2C interface must be
     * initialized separately before calling begin().
     *
     * @param i2c_instance Pointer to initialized I2C instance (e.g., i2c_default)
     * @param device_address I2C address of the device (default: 0x29)
     * @param reset_pin GPIO pin connected to device reset (255 = no reset pin)
     *
     * @note The reset pin parameter is optional. If not connected, pass 255.
     */
    explicit CAP1188Device(i2c_inst_t* i2c_instance, 
                          uint8_t device_address = DEFAULT_I2C_ADDRESS,
                          uint reset_pin = 255); // 255 = no reset pin (disabled)

    /**
     * @brief Destructor
     *
     * Cleans up resources and puts the device into deep sleep mode
     * if it was initialized.
     */
    ~CAP1188Device();

    /**
     * @name Device Initialization and Management
     * @{
     */
    
    /**
     * @brief Initialize the CAP1188 device
     *
     * Performs device initialization including hardware verification,
     * reset (if reset pin is connected), and default configuration setup.
     *
     * @return Error::SUCCESS on successful initialization
     * @return Error::DEVICE_NOT_FOUND if device is not detected
     * @return Error::I2C_ERROR if I2C communication fails
     * @return Error::INVALID_PARAMETER if I2C instance is null
     *
     * @note I2C hardware must be initialized before calling this function
     */
    Error begin();
    
    /**
     * @brief Reset the device
     *
     * Performs either hardware reset (if reset pin is connected) or
     * software reset via I2C command.
     *
     * @return Error::SUCCESS on successful reset
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error reset();
    
    /**
     * @brief Check if device is connected and responding
     *
     * Verifies device connection by reading and validating the product ID.
     *
     * @return true if device is connected and responding
     * @return false if device is not detected or not responding
     */
    bool isConnected();
    
    /**
     * @brief Get current device status
     *
     * Reads and returns comprehensive device status information including
     * touch detection, error conditions, and system health.
     *
     * @return DeviceStatus structure containing current status
     */
    DeviceStatus getStatus();
    
    /**
     * @brief Set device configuration
     *
     * Applies a complete device configuration. This will update all
     * device settings according to the provided configuration structure.
     *
     * @param config DeviceConfig structure with desired settings
     * @return Error::SUCCESS on successful configuration
     * @return Error::INVALID_PARAMETER if configuration is invalid
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error setConfiguration(const DeviceConfig& config);
    
    /**
     * @brief Get current device configuration
     *
     * Returns the current device configuration as stored in the device.
     *
     * @return DeviceConfig structure with current settings
     */
    DeviceConfig getConfiguration() const;
    
    /** @} */ // Device Initialization and Management
    
    /**
     * @name Runtime Configuration Updates (Granular)
     * @brief Individual setting updates without full reconfiguration
     * @{
     */
    
    /**
     * @brief Update touch sensitivity setting
     *
     * Updates only the touch sensitivity setting without affecting
     * other configuration parameters.
     *
     * @param sensitivity New touch sensitivity level
     * @return Error::SUCCESS on successful update
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error updateSensitivity(TouchSensitivity sensitivity);
    
    /**
     * @brief Update touch response speed setting
     *
     * Updates only the response speed setting without affecting
     * other configuration parameters.
     *
     * @param speed New response speed setting
     * @return Error::SUCCESS on successful update
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error updateResponseSpeed(TouchResponseSpeed speed);
    
    /**
     * @brief Update touch stability setting
     *
     * Updates only the stability setting without affecting
     * other configuration parameters.
     *
     * @param stability New stability setting
     * @return Error::SUCCESS on successful update
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error updateStability(TouchStability stability);
    
    /**
     * @brief Update noise filtering setting
     *
     * Updates only the noise filtering setting without affecting
     * other configuration parameters.
     *
     * @param filtering New noise filtering level
     * @return Error::SUCCESS on successful update
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error updateNoiseFiltering(NoiseFiltering filtering);
    
    /**
     * @brief Update LED behavior setting
     *
     * Updates LED behavior and optionally LED speed without affecting
     * other configuration parameters.
     *
     * @param behavior New LED behavior setting
     * @param speed New LED speed setting (optional)
     * @return Error::SUCCESS on successful update
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error updateLEDBehavior(LEDBehavior behavior, LEDSpeed speed = LEDSpeed::MEDIUM);
    
    /**
     * @brief Update multi-touch mode setting
     *
     * Updates only the multi-touch mode without affecting
     * other configuration parameters.
     *
     * @param mode New multi-touch mode
     * @return Error::SUCCESS on successful update
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error updateMultiTouchMode(MultiTouchMode mode);
    
    /** @} */ // Runtime Configuration Updates (Granular)
    
    /**
     * @name Runtime Configuration Updates (Batch)
     * @brief Update multiple related settings atomically
     * @{
     */
    
    /**
     * @brief Update touch-related settings together
     *
     * Updates sensitivity, response speed, and stability settings in a
     * single atomic operation for optimal performance.
     *
     * @param sensitivity New touch sensitivity level
     * @param speed New response speed setting
     * @param stability New stability setting
     * @return Error::SUCCESS on successful update
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error updateTouchSettings(TouchSensitivity sensitivity, TouchResponseSpeed speed, TouchStability stability);
    
    /**
     * @brief Update LED-related settings together
     *
     * Updates LED behavior, speed, and polarity settings in a
     * single atomic operation.
     *
     * @param behavior New LED behavior setting
     * @param speed New LED speed setting
     * @param active_high LED polarity (true = active high, false = active low)
     * @return Error::SUCCESS on successful update
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error updateLEDSettings(LEDBehavior behavior, LEDSpeed speed, bool active_high);
    
    /**
     * @brief Update noise filtering settings together
     *
     * Updates noise filtering level and digital/analog filter enables
     * in a single atomic operation.
     *
     * @param filtering New noise filtering level
     * @param digital_filter Enable digital noise filtering
     * @param analog_filter Enable analog noise filtering
     * @return Error::SUCCESS on successful update
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error updateNoiseSettings(NoiseFiltering filtering, bool digital_filter, bool analog_filter);
    
    /**
     * @brief Update power management settings together
     *
     * Updates interrupt and deep sleep settings in a single
     * atomic operation.
     *
     * @param interrupts Enable interrupt generation
     * @param deep_sleep Enable deep sleep capability
     * @return Error::SUCCESS on successful update
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error updatePowerSettings(bool interrupts, bool deep_sleep);
    
    /** @} */ // Runtime Configuration Updates (Batch)
    
    /**
     * @name Runtime Configuration Updates (Smart Merging)
     * @brief Intelligent configuration updates with change detection
     * @{
     */
    
    /**
     * @brief Update device configuration with smart change detection
     *
     * Compares the new configuration with the current configuration and
     * only updates settings that have actually changed. This provides
     * optimal performance and atomic updates.
     *
     * @param new_config New device configuration
     * @param force_all Force update of all settings (default: false)
     * @return Error::SUCCESS on successful update
     * @return Error::CONFIGURATION_ERROR if configuration is invalid
     * @return Error::I2C_ERROR if I2C communication fails
     *
     * @note When force_all is false, only changed settings are updated.
     *       When force_all is true, all settings are updated regardless of changes.
     */
    Error updateConfiguration(const DeviceConfig& new_config, bool force_all = false);
    
    /** @} */ // Runtime Configuration Updates (Smart Merging)
    
    /**
     * @name Per-Channel Runtime Updates
     * @brief Update individual channel settings
     * @{
     */
    
    /**
     * @brief Update sensitivity for a specific channel
     *
     * Updates the touch sensitivity for a single channel without
     * affecting other channels or settings.
     *
     * @param channel Channel to update
     * @param sensitivity New sensitivity level (or USE_GLOBAL)
     * @return Error::SUCCESS on successful update
     * @return Error::INVALID_PARAMETER if channel is invalid
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error updateChannelSensitivity(TouchChannel channel, TouchSensitivity sensitivity);
    
    /**
     * @brief Update LED behavior for a specific channel
     *
     * Updates the LED behavior for a single channel without
     * affecting other channels or settings.
     *
     * @param channel Channel to update
     * @param behavior New LED behavior (or USE_GLOBAL)
     * @return Error::SUCCESS on successful update
     * @return Error::INVALID_PARAMETER if channel is invalid
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error updateChannelLEDBehavior(TouchChannel channel, LEDBehavior behavior);
    
    /**
     * @brief Update complete configuration for a specific channel
     *
     * Updates the complete configuration for a single channel with
     * smart change detection.
     *
     * @param channel Channel to update
     * @param new_config New channel configuration
     * @param force_all Force update of all settings (default: false)
     * @return Error::SUCCESS on successful update
     * @return Error::INVALID_PARAMETER if channel is invalid
     * @return Error::CONFIGURATION_ERROR if configuration is invalid
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error updateChannelConfiguration(TouchChannel channel, const TouchConfig& new_config, bool force_all = false);
    
    /** @} */ // Per-Channel Runtime Updates

    /**
     * @name Touch Sensing Functions
     * @brief Core touch detection and channel control
     * @{
     */
    
    /**
     * @brief Get bitmask of currently touched channels
     *
     * Returns a bitmask indicating which channels are currently
     * detecting touch. Bit 0 = C1, bit 1 = C2, etc.
     *
     * @return Bitmask of touched channels (bit 0 = C1, bit 1 = C2, etc.)
     */
    uint8_t getTouchedChannels();
    
    /**
     * @brief Check if a specific channel is currently touched
     *
     * Checks the current touch state of a specific channel.
     *
     * @param channel Channel to check
     * @return true if channel is touched, false otherwise
     */
    bool isChannelTouched(TouchChannel channel);
    
    /**
     * @brief Enable or disable a touch channel
     *
     * Enables or disables touch detection for a specific channel.
     *
     * @param channel Channel to enable/disable
     * @param enable true to enable, false to disable
     * @return Error::SUCCESS on successful operation
     * @return Error::INVALID_PARAMETER if channel is invalid
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error enableChannel(TouchChannel channel, bool enable = true);
    
    /**
     * @brief Disable a touch channel
     *
     * Convenience function to disable touch detection for a specific channel.
     *
     * @param channel Channel to disable
     * @return Error::SUCCESS on successful operation
     * @return Error::INVALID_PARAMETER if channel is invalid
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error disableChannel(TouchChannel channel);
    
    /** @} */ // Touch Sensing Functions
    
    /**
     * @name Threshold and Sensitivity Control
     * @brief Direct threshold and channel configuration control
     * @{
     */
    
    /**
     * @brief Set touch threshold for a specific channel
     *
     * Sets the touch detection threshold for a specific channel.
     * Lower values increase sensitivity.
     *
     * @param channel Channel to configure
     * @param threshold Touch threshold value (0-255)
     * @return Error::SUCCESS on successful operation
     * @return Error::INVALID_PARAMETER if channel is invalid
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error setChannelThreshold(TouchChannel channel, uint8_t threshold);
    
    /**
     * @brief Get current touch threshold for a specific channel
     *
     * Reads the current touch threshold setting for a specific channel.
     *
     * @param channel Channel to read
     * @return Current threshold value (0-255)
     */
    uint8_t getChannelThreshold(TouchChannel channel);
    
    /**
     * @brief Set complete configuration for a specific channel
     *
     * Applies a complete configuration to a specific channel.
     *
     * @param channel Channel to configure
     * @param config Channel configuration structure
     * @return Error::SUCCESS on successful operation
     * @return Error::INVALID_PARAMETER if channel is invalid or config is invalid
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error setChannelConfig(TouchChannel channel, const TouchConfig& config);
    
    /**
     * @brief Get current configuration for a specific channel
     *
     * Reads the current configuration for a specific channel.
     *
     * @param channel Channel to read
     * @return TouchConfig structure with current channel settings
     */
    TouchConfig getChannelConfig(TouchChannel channel);
    
    /** @} */ // Threshold and Sensitivity Control
    
    /**
     * @name Calibration Functions
     * @brief Device and channel calibration control
     * @{
     */
    
    /**
     * @brief Calibrate a specific channel
     *
     * Initiates calibration for a specific channel. This sets the
     * baseline reading for "no touch" condition.
     *
     * @param channel Channel to calibrate
     * @return Error::SUCCESS on successful calibration
     * @return Error::INVALID_PARAMETER if channel is invalid
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error calibrateChannel(TouchChannel channel);
    
    /**
     * @brief Calibrate all channels
     *
     * Initiates calibration for all enabled channels simultaneously.
     * This is more efficient than calibrating channels individually.
     *
     * @return Error::SUCCESS on successful calibration
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error calibrateAllChannels();
    
    /**
     * @brief Get base count for a specific channel
     *
     * Reads the current base count (calibrated baseline) for a
     * specific channel.
     *
     * @param channel Channel to read
     * @return Base count value (0-255)
     */
    uint8_t getBaseCount(TouchChannel channel);
    
    /** @} */ // Calibration Functions
    
    /**
     * @name LED Control Functions
     * @brief LED output control and configuration
     * @{
     */
    
    /**
     * @brief Set LED state (simple on/off)
     *
     * Sets the LED for a specific channel to on or off state.
     *
     * @param channel Channel LED to control
     * @param state true for on, false for off
     * @return Error::SUCCESS on successful operation
     * @return Error::INVALID_PARAMETER if channel is invalid
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error setLEDState(TouchChannel channel, bool state);
    
    /**
     * @brief Set LED state (advanced effects)
     *
     * Sets the LED for a specific channel to a specific state including
     * advanced effects like pulsing and breathing.
     *
     * @param channel Channel LED to control
     * @param state LED state/effect
     * @return Error::SUCCESS on successful operation
     * @return Error::INVALID_PARAMETER if channel is invalid
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error setLEDState(TouchChannel channel, LEDState state);
    
    /**
     * @brief Set advanced LED configuration
     *
     * Applies advanced LED configuration including timing, duty cycles,
     * and effect parameters.
     *
     * @param channel Channel LED to configure
     * @param config LED configuration structure
     * @return Error::SUCCESS on successful operation
     * @return Error::INVALID_PARAMETER if channel is invalid or config is invalid
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error setLEDConfig(TouchChannel channel, const LEDConfig& config);
    
    /**
     * @brief Set LED polarity
     *
     * Sets the polarity for all LED outputs.
     *
     * @param active_high true for active high, false for active low
     * @return Error::SUCCESS on successful operation
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error setLEDPolarity(bool active_high);
    
    /**
     * @brief Link LED to touch detection
     *
     * Controls whether the LED for a specific channel is automatically
     * controlled by touch detection.
     *
     * @param channel Channel to configure
     * @param linked true to link LED to touch, false for manual control
     * @return Error::SUCCESS on successful operation
     * @return Error::INVALID_PARAMETER if channel is invalid
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error linkLEDToTouch(TouchChannel channel, bool linked = true);
    
    /** @} */ // LED Control Functions
    
    /**
     * @name Advanced LED Effects
     * @brief Advanced LED timing and effect control
     * @{
     */
    
    /**
     * @brief Set LED pulse effect
     *
     * Configures the LED to pulse with specified period and duty cycle.
     *
     * @param channel Channel LED to configure
     * @param period Pulse period (affects timing)
     * @param duty_cycle Pulse duty cycle (affects brightness)
     * @return Error::SUCCESS on successful operation
     * @return Error::INVALID_PARAMETER if channel is invalid
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error setLEDPulse(TouchChannel channel, uint8_t period, uint8_t duty_cycle);
    
    /**
     * @brief Set LED breathe effect
     *
     * Configures the LED to breathe (fade in/out) with specified
     * period and duty cycle.
     *
     * @param channel Channel LED to configure
     * @param period Breathe period (affects timing)
     * @param duty_cycle Breathe duty cycle (affects brightness)
     * @return Error::SUCCESS on successful operation
     * @return Error::INVALID_PARAMETER if channel is invalid
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error setLEDBreathe(TouchChannel channel, uint8_t period, uint8_t duty_cycle);
    
    /**
     * @brief Set LED direct brightness control
     *
     * Sets the LED to a specific brightness level with direct control.
     *
     * @param channel Channel LED to control
     * @param brightness Brightness level (0-255)
     * @return Error::SUCCESS on successful operation
     * @return Error::INVALID_PARAMETER if channel is invalid
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error setLEDDirectControl(TouchChannel channel, uint8_t brightness);
    
    /** @} */ // Advanced LED Effects
    
    /**
     * @name Power Management
     * @brief Device power mode control
     * @{
     */
    
    /**
     * @brief Enter standby mode
     *
     * Puts the device into standby mode for reduced power consumption.
     * Touch detection may still be active depending on configuration.
     *
     * @return Error::SUCCESS on successful operation
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error enterStandby();
    
    /**
     * @brief Exit standby mode
     *
     * Returns the device to normal operation from standby mode.
     *
     * @return Error::SUCCESS on successful operation
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error exitStandby();
    
    /**
     * @brief Enter deep sleep mode
     *
     * Puts the device into deep sleep mode for minimum power consumption.
     * Wake-up typically requires reset or specific touch events.
     *
     * @return Error::SUCCESS on successful operation
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error enterDeepSleep();
    
    /**
     * @brief Exit deep sleep mode
     *
     * Returns the device to normal operation from deep sleep mode.
     *
     * @return Error::SUCCESS on successful operation
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error exitDeepSleep();
    
    /**
     * @brief Get current power mode
     *
     * Returns the current power mode of the device.
     *
     * @return Current PowerMode
     */
    PowerMode getPowerMode() const;
    
    /** @} */ // Power Management
    
    /**
     * @name Interrupt Handling
     * @brief Interrupt configuration and callback management
     * @{
     */
    
    /**
     * @brief Enable or disable interrupts
     *
     * Controls whether the device generates interrupts for touch events.
     *
     * @param enable true to enable interrupts, false to disable
     * @return Error::SUCCESS on successful operation
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error enableInterrupts(bool enable = true);
    
    /**
     * @brief Disable interrupts
     *
     * Convenience function to disable interrupt generation.
     *
     * @return Error::SUCCESS on successful operation
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error disableInterrupts();
    
    /**
     * @brief Check if interrupt is pending
     *
     * Checks whether the device has a pending interrupt condition.
     *
     * @return true if interrupt is pending, false otherwise
     */
    bool isInterruptPending();
    
    /**
     * @brief Clear pending interrupt
     *
     * Clears any pending interrupt condition.
     *
     * @return Error::SUCCESS on successful operation
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error clearInterrupt();
    
    /**
     * @brief Set touch event callback function
     *
     * Sets a callback function to be called when touch events occur.
     *
     * @param callback Function to call for touch events (nullptr to disable)
     */
    void setTouchCallback(TouchCallback callback);
    
    /**
     * @brief Set error callback function
     *
     * Sets a callback function to be called when errors occur.
     *
     * @param callback Function to call for errors (nullptr to disable)
     */
    void setErrorCallback(ErrorCallback callback);
    
    /** @} */ // Interrupt Handling
    
    /**
     * @name Multiple Touch Configuration
     * @brief Multi-touch detection configuration
     * @{
     */
    
    /**
     * @brief Enable or disable multi-touch detection
     *
     * Controls whether the device can detect multiple simultaneous touches.
     *
     * @param enable true to enable multi-touch, false to disable
     * @return Error::SUCCESS on successful operation
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error enableMultiTouch(bool enable = true);
    
    /**
     * @brief Set multi-touch configuration
     *
     * Sets the low-level multi-touch configuration register.
     *
     * @param config Multi-touch configuration value
     * @return Error::SUCCESS on successful operation
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error setMultiTouchConfig(uint8_t config);
    
    /** @} */ // Multiple Touch Configuration
    
    /**
     * @name Noise Filtering
     * @brief Noise detection and filtering control
     * @{
     */
    
    /**
     * @brief Enable or disable digital noise filtering
     *
     * Controls the digital noise filtering feature.
     *
     * @param enable true to enable digital noise filtering, false to disable
     * @return Error::SUCCESS on successful operation
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error enableDigitalNoiseFilter(bool enable = true);
    
    /**
     * @brief Enable or disable analog noise filtering
     *
     * Controls the analog noise filtering feature.
     *
     * @param enable true to enable analog noise filtering, false to disable
     * @return Error::SUCCESS on successful operation
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error enableAnalogNoiseFilter(bool enable = true);
    
    /**
     * @brief Set noise detection threshold
     *
     * Sets the threshold for noise detection on all channels.
     *
     * @param threshold Noise threshold value (0-255)
     * @return Error::SUCCESS on successful operation
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error setNoiseThreshold(uint8_t threshold);
    
    /** @} */ // Noise Filtering
    
    /**
     * @name Gain and Sensitivity (Low-Level)
     * @brief Direct hardware gain and sensitivity control
     * @{
     */
    
    /**
     * @brief Set analog gain
     *
     * Sets the analog gain for touch detection. Higher gain increases
     * sensitivity but may also increase noise susceptibility.
     *
     * @param gain Analog gain setting
     * @return Error::SUCCESS on successful operation
     * @return Error::I2C_ERROR if I2C communication fails
     * @note Most users should use TouchSensitivity instead
     */
    Error setGain(Gain gain);
    
    /**
     * @brief Get current analog gain
     *
     * Returns the current analog gain setting.
     *
     * @return Current Gain setting
     */
    Gain getGain() const;
    
    /**
     * @brief Set delta sense
     *
     * Sets the delta sense for touch detection algorithm.
     *
     * @param sense Delta sense setting
     * @return Error::SUCCESS on successful operation
     * @return Error::I2C_ERROR if I2C communication fails
     * @note Most users should use TouchSensitivity instead
     */
    Error setDeltaSense(DeltaSense sense);
    
    /**
     * @brief Get current delta sense
     *
     * Returns the current delta sense setting.
     *
     * @return Current DeltaSense setting
     */
    DeltaSense getDeltaSense() const;
    
    /** @} */ // Gain and Sensitivity (Low-Level)
    
    /**
     * @name Low-Level Register Access
     * @brief Direct register read/write access
     * @{
     */
    
    /**
     * @brief Read a single register
     *
     * Reads a single register value from the device.
     *
     * @param reg Register address to read
     * @param value Reference to store the read value
     * @return Error::SUCCESS on successful read
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error readRegister(uint8_t reg, uint8_t& value);
    
    /**
     * @brief Write a single register
     *
     * Writes a single register value to the device.
     *
     * @param reg Register address to write
     * @param value Value to write
     * @return Error::SUCCESS on successful write
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error writeRegister(uint8_t reg, uint8_t value);
    
    /**
     * @brief Read multiple consecutive registers
     *
     * Reads multiple consecutive registers from the device.
     *
     * @param start_reg Starting register address
     * @param buffer Buffer to store read values
     * @param length Number of registers to read
     * @return Error::SUCCESS on successful read
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error readMultipleRegisters(uint8_t start_reg, uint8_t* buffer, size_t length);
    
    /**
     * @brief Write multiple consecutive registers
     *
     * Writes multiple consecutive registers to the device.
     *
     * @param start_reg Starting register address
     * @param data Data to write
     * @param length Number of registers to write
     * @return Error::SUCCESS on successful write
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error writeMultipleRegisters(uint8_t start_reg, const uint8_t* data, size_t length);
    
    /** @} */ // Low-Level Register Access
    
    /**
     * @name Utility Functions
     * @brief Helper and debugging functions
     * @{
     */
    
    /**
     * @brief Get human-readable error string
     *
     * Converts an Error code to a human-readable string.
     *
     * @param error Error code to convert
     * @return Pointer to error string
     */
    const char* getErrorString(Error error) const;
    
    /**
     * @brief Print current device status
     *
     * Prints comprehensive device status information to stdout.
     * Useful for debugging and monitoring.
     */
    void printStatus() const;
    
    /**
     * @brief Print current device configuration
     *
     * Prints complete device configuration to stdout.
     * Useful for debugging and verification.
     */
    void printConfiguration() const;
    
    /** @} */ // Utility Functions
    
    /**
     * @name Device Information
     * @brief Device identification and version information
     * @{
     */
    
    /**
     * @brief Get product ID
     *
     * Reads the product ID register. Should return 0x50 for CAP1188.
     *
     * @return Product ID value (0x50 for CAP1188)
     */
    uint8_t getProductID();
    
    /**
     * @brief Get manufacturer ID
     *
     * Reads the manufacturer ID register. Should return 0x5D for Microchip.
     *
     * @return Manufacturer ID value (0x5D for Microchip)
     */
    uint8_t getManufacturerID();
    
    /**
     * @brief Get device revision
     *
     * Reads the device revision register.
     *
     * @return Device revision value (0x82 for current revision)
     */
    uint8_t getRevision();
    
    /** @} */ // Device Information
    
private:
    /**
     * @name Private Member Variables
     * @{
     */
    
    // Hardware interface
    i2c_inst_t* _i2c;          ///< I2C instance pointer
    uint8_t _address;          ///< Device I2C address
    uint _reset_pin;           ///< Reset pin GPIO number (255 = not connected)
    
    // State tracking
    bool _initialized;                ///< Device initialization state
    DeviceConfig _config;            ///< Current device configuration
    TouchConfig _channel_configs[8]; ///< Per-channel configurations
    PowerMode _power_mode;           ///< Current power mode
    
    // Callbacks
    TouchCallback _touch_callback;   ///< Touch event callback function
    ErrorCallback _error_callback;   ///< Error callback function
    
    // Last known state for change detection
    uint8_t _last_touched_state;     ///< Last touch state for change detection
    absolute_time_t _last_update_time; ///< Last update timestamp
    
    /** @} */ // Private Member Variables
    
    /**
     * @name Internal Helper Functions
     * @{
     */
    
    /**
     * @brief Verify device identity
     * 
     * Verifies that the connected device is a CAP1188 by checking
     * the product ID, manufacturer ID, and revision.
     * 
     * @return Error::SUCCESS if device is verified
     * @return Error::DEVICE_NOT_FOUND if device ID doesn't match
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error _verifyDevice();
    
    /**
     * @brief Configure device with default settings
     * 
     * Applies default configuration settings to the device after
     * initialization.
     * 
     * @return Error::SUCCESS on successful configuration
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error _configureDefaults();
    
    /**
     * @brief Perform hardware reset
     * 
     * Performs a hardware reset using the reset pin if connected.
     * 
     * @return Error::SUCCESS on successful reset
     * @return Error::INVALID_PARAMETER if reset pin is not connected
     */
    Error _hardwareReset();
    
    /**
     * @brief Perform software reset
     * 
     * Performs a software reset via I2C command.
     * 
     * @return Error::SUCCESS on successful reset
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error _softwareReset();
    
    /** @} */ // Internal Helper Functions
    
    /**
     * @name I2C Communication Helpers
     * @{
     */
    
    /**
     * @brief Write single register via I2C
     * 
     * Low-level I2C write function with error handling.
     * 
     * @param reg Register address
     * @param value Value to write
     * @return Error::SUCCESS on successful write
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error _i2cWrite(uint8_t reg, uint8_t value);
    
    /**
     * @brief Read single register via I2C
     * 
     * Low-level I2C read function with error handling.
     * 
     * @param reg Register address
     * @param value Reference to store read value
     * @return Error::SUCCESS on successful read
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error _i2cRead(uint8_t reg, uint8_t& value);
    
    /**
     * @brief Write multiple registers via I2C
     * 
     * Low-level I2C write function for multiple consecutive registers.
     * 
     * @param reg Starting register address
     * @param data Data to write
     * @param length Number of bytes to write
     * @return Error::SUCCESS on successful write
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error _i2cWriteMultiple(uint8_t reg, const uint8_t* data, size_t length);
    
    /**
     * @brief Read multiple registers via I2C
     * 
     * Low-level I2C read function for multiple consecutive registers.
     * 
     * @param reg Starting register address
     * @param buffer Buffer to store read data
     * @param length Number of bytes to read
     * @return Error::SUCCESS on successful read
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error _i2cReadMultiple(uint8_t reg, uint8_t* buffer, size_t length);
    
    /** @} */ // I2C Communication Helpers
    
    /**
     * @name Configuration Helpers
     * @{
     */
    
    /**
     * @brief Get threshold register address for channel
     * 
     * Returns the register address for the threshold setting of
     * a specific channel.
     * 
     * @param channel Channel to get register for
     * @return Register address for channel threshold
     */
    uint8_t _getThresholdRegister(TouchChannel channel) const;
    
    /**
     * @brief Get base count register address for channel
     * 
     * Returns the register address for the base count of
     * a specific channel.
     * 
     * @param channel Channel to get register for
     * @return Register address for channel base count
     */
    uint8_t _getBaseCountRegister(TouchChannel channel) const;
    
    /**
     * @brief Update main control register
     * 
     * Updates the main control register based on current configuration.
     * 
     * @return Error::SUCCESS on successful update
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error _updateMainControl();
    
    /**
     * @brief Update sensor configuration registers
     * 
     * Updates sensor configuration registers based on current configuration.
     * 
     * @return Error::SUCCESS on successful update
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error _updateSensorConfig();
    
    /** @} */ // Configuration Helpers
    
    /**
     * @name Runtime Configuration Update Helpers
     * @{
     */
    
    /**
     * @brief Update sensitivity-related registers
     * 
     * Updates hardware registers to implement the specified sensitivity.
     * 
     * @param sensitivity Sensitivity level to apply
     * @return Error::SUCCESS on successful update
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error _updateSensitivityRegisters(TouchSensitivity sensitivity);
    
    /**
     * @brief Update response speed-related registers
     * 
     * Updates hardware registers to implement the specified response speed.
     * 
     * @param speed Response speed to apply
     * @return Error::SUCCESS on successful update
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error _updateResponseSpeedRegisters(TouchResponseSpeed speed);
    
    /**
     * @brief Update stability-related registers
     * 
     * Updates hardware registers to implement the specified stability.
     * 
     * @param stability Stability level to apply
     * @return Error::SUCCESS on successful update
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error _updateStabilityRegisters(TouchStability stability);
    
    /**
     * @brief Update noise filtering-related registers
     * 
     * Updates hardware registers to implement the specified noise filtering.
     * 
     * @param filtering Noise filtering level to apply
     * @return Error::SUCCESS on successful update
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error _updateNoiseFilteringRegisters(NoiseFiltering filtering);
    
    /**
     * @brief Update LED behavior-related registers
     * 
     * Updates hardware registers to implement the specified LED behavior.
     * 
     * @param behavior LED behavior to apply
     * @param speed LED speed to apply
     * @return Error::SUCCESS on successful update
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error _updateLEDBehaviorRegisters(LEDBehavior behavior, LEDSpeed speed);
    
    /**
     * @brief Update multi-touch-related registers
     * 
     * Updates hardware registers to implement the specified multi-touch mode.
     * 
     * @param mode Multi-touch mode to apply
     * @return Error::SUCCESS on successful update
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error _updateMultiTouchRegisters(MultiTouchMode mode);
    
    /** @} */ // Runtime Configuration Update Helpers
    
    /**
     * @name Configuration Validation
     * @{
     */
    
    /**
     * @brief Validate device configuration
     * 
     * Validates that a device configuration is valid and safe to apply.
     * 
     * @param config Configuration to validate
     * @return Error::SUCCESS if configuration is valid
     * @return Error::INVALID_PARAMETER if configuration is invalid
     */
    Error _validateDeviceConfig(const DeviceConfig& config) const;
    
    /**
     * @brief Validate channel configuration
     * 
     * Validates that a channel configuration is valid and safe to apply.
     * 
     * @param channel Channel being configured
     * @param config Configuration to validate
     * @return Error::SUCCESS if configuration is valid
     * @return Error::INVALID_PARAMETER if configuration is invalid
     */
    Error _validateChannelConfig(TouchChannel channel, const TouchConfig& config) const;
    
    /**
     * @brief Validate configuration change
     * 
     * Validates that a configuration change is safe and won't cause
     * disruptive behavior.
     * 
     * @param old_config Current configuration
     * @param new_config Proposed new configuration
     * @return Error::SUCCESS if change is valid
     * @return Error::CONFIGURATION_ERROR if change is invalid
     */
    Error _validateConfigurationChange(const DeviceConfig& old_config, const DeviceConfig& new_config) const;
    
    /** @} */ // Configuration Validation
    
    /**
     * @name Change Detection
     * @{
     */
    
    /**
     * @brief Detect configuration changes
     * 
     * Compares two device configurations and returns a set of changes
     * that need to be applied.
     * 
     * @param old_config Current configuration
     * @param new_config Proposed new configuration
     * @return ConfigChangeSet indicating what has changed
     */
    ConfigChangeSet _detectDeviceConfigChanges(const DeviceConfig& old_config, const DeviceConfig& new_config) const;
    
    /**
     * @brief Apply configuration changes
     * 
     * Applies only the changes specified in the ConfigChangeSet.
     * 
     * @param config Configuration to apply
     * @param changes Set of changes to apply
     * @return Error::SUCCESS on successful application
     * @return Error::I2C_ERROR if I2C communication fails
     */
    Error _applyDeviceConfigChanges(const DeviceConfig& config, const ConfigChangeSet& changes);
    
    /** @} */ // Change Detection
    
    /**
     * @name Configuration Translation Methods
     * @brief Convert human-readable settings to register values
     * @{
     */
    
    /**
     * @brief Convert sensitivity to threshold value
     * 
     * Converts human-readable sensitivity setting to register threshold value.
     * 
     * @param sensitivity Sensitivity level
     * @return Threshold register value
     */
    uint8_t _sensitivityToThreshold(TouchSensitivity sensitivity) const;
    
    /**
     * @brief Convert response speed to repeat rate
     * 
     * Converts human-readable response speed to register repeat rate value.
     * 
     * @param speed Response speed
     * @return Repeat rate register value
     */
    uint8_t _responseSpeedToRepeatRate(TouchResponseSpeed speed) const;
    
    /**
     * @brief Convert stability to averaging value
     * 
     * Converts human-readable stability setting to register averaging value.
     * 
     * @param stability Stability level
     * @return Averaging register value
     */
    uint8_t _stabilityToAveraging(TouchStability stability) const;
    
    /**
     * @brief Convert noise filtering to config value
     * 
     * Converts human-readable noise filtering to register configuration value.
     * 
     * @param filtering Noise filtering level
     * @return Configuration register value
     */
    uint8_t _noiseFilteringToConfig(NoiseFiltering filtering) const;
    
    /**
     * @brief Convert multi-touch mode to config value
     * 
     * Converts human-readable multi-touch mode to register configuration value.
     * 
     * @param mode Multi-touch mode
     * @return Configuration register value
     */
    uint8_t _multiTouchToConfig(MultiTouchMode mode) const;
    
    /**
     * @brief Convert LED behavior to register values
     * 
     * Converts human-readable LED behavior and speed to register values.
     * 
     * @param behavior LED behavior
     * @param speed LED speed
     * @param pulse1 Output: pulse 1 register value
     * @param pulse2 Output: pulse 2 register value
     * @param breathe Output: breathe register value
     */
    void _ledBehaviorToRegisters(LEDBehavior behavior, LEDSpeed speed, uint8_t& pulse1, uint8_t& pulse2, uint8_t& breathe) const;
    
    /** @} */ // Configuration Translation Methods
    
    /**
     * @name State Management
     * @{
     */
    
    /**
     * @brief Update internal touch state
     * 
     * Updates the internal touch state tracking and triggers
     * callbacks if needed.
     */
    void _updateTouchState();
    
    /**
     * @brief Process touch events
     * 
     * Processes touch events and calls appropriate callbacks.
     * 
     * @param current_state Current touch state bitmask
     */
    void _processTouchEvents(uint8_t current_state);
    
    /**
     * @brief Call error callback
     * 
     * Calls the error callback function if one is registered.
     * 
     * @param error Error code
     * @param message Optional error message
     */
    void _callErrorCallback(Error error, const char* message = nullptr);
    
    /** @} */ // State Management
    
    /**
     * @name Validation Helpers
     * @{
     */
    
    /**
     * @brief Check if channel is valid
     * 
     * Validates that a channel number is within valid range.
     * 
     * @param channel Channel to validate
     * @return true if channel is valid, false otherwise
     */
    bool _isValidChannel(TouchChannel channel) const;
    
    /**
     * @brief Check if threshold is valid
     * 
     * Validates that a threshold value is within valid range.
     * 
     * @param threshold Threshold to validate
     * @return true if threshold is valid, false otherwise
     */
    bool _isValidThreshold(uint8_t threshold) const;
    
    /** @} */ // Validation Helpers
    
    /**
     * @name Debug Helpers
     * @{
     */
    
    /**
     * @brief Print debug message
     * 
     * Prints a debug message if debug output is enabled.
     * 
     * @param message Debug message to print
     */
    void _debugPrint(const char* message) const;
    
    /**
     * @brief Print register value for debugging
     * 
     * Prints a register address and value for debugging.
     * 
     * @param reg Register address
     * @param value Register value
     */
    void _debugPrintRegister(uint8_t reg, uint8_t value) const;
    
    /** @} */ // Debug Helpers
};

/**
 * @name Global Utility Functions
 * @brief Utility functions for the CAP1188 library
 * @{
 */

/**
 * @brief Convert error code to string
 * 
 * Converts an Error enum value to a human-readable string.
 * 
 * @param error Error code to convert
 * @return Pointer to error string
 */
const char* errorToString(Error error);

/**
 * @brief Convert touch channel to string
 * 
 * Converts a TouchChannel enum value to a human-readable string.
 * 
 * @param channel Channel to convert
 * @return Pointer to channel string (e.g., "C1", "C2")
 */
const char* channelToString(TouchChannel channel);

/**
 * @brief Convert LED state to string
 * 
 * Converts an LEDState enum value to a human-readable string.
 * 
 * @param state LED state to convert
 * @return Pointer to state string
 */
const char* ledStateToString(LEDState state);

/**
 * @brief Convert power mode to string
 * 
 * Converts a PowerMode enum value to a human-readable string.
 * 
 * @param mode Power mode to convert
 * @return Pointer to mode string
 */
const char* powerModeToString(PowerMode mode);

/** @} */ // Global Utility Functions

/**
 * @name Conversion Helpers
 * @brief Helper functions for channel index conversion
 * @{
 */

/**
 * @brief Convert array index to TouchChannel
 * 
 * Converts a zero-based array index to the corresponding TouchChannel.
 * 
 * @param index Array index (0-7)
 * @return TouchChannel enum value
 */
TouchChannel indexToChannel(uint8_t index);

/**
 * @brief Convert TouchChannel to array index
 * 
 * Converts a TouchChannel enum value to a zero-based array index.
 * 
 * @param channel TouchChannel to convert
 * @return Array index (0-7)
 */
uint8_t channelToIndex(TouchChannel channel);

/** @} */ // Conversion Helpers

} // namespace CAP1188