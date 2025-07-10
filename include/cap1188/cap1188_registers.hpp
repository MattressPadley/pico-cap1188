/**
 * @file cap1188_registers.hpp
 * @brief CAP1188 register definitions and hardware constants
 *
 * This header contains all register addresses, bit field definitions,
 * and hardware constants for the CAP1188 capacitive touch sensor.
 * These definitions provide direct access to the hardware registers
 * for advanced users and internal library implementation.
 *
 * @author CAP1188 Library Contributors
 * @date 2024
 * @version 0.0.1
 *
 * @note Register values are based on the official CAP1188 datasheet.
 *       Most users should use the high-level API instead of direct
 *       register access.
 */

#pragma once

#include <cstdint>

/**
 * @namespace CAP1188
 * @brief Main namespace for the CAP1188 capacitive touch sensor library
 */
namespace CAP1188 {

/**
 * @defgroup I2C_Addresses I2C Address Constants
 * @brief Available I2C addresses for the CAP1188 device
 *
 * The CAP1188 supports multiple I2C addresses, selectable via hardware
 * configuration. This allows multiple devices on the same I2C bus.
 *
 * @note Address selection is typically done via the ADDR_SEL pin.
 * @{
 */
constexpr uint8_t DEFAULT_I2C_ADDRESS = 0x29; ///< Default I2C address
constexpr uint8_t I2C_ADDRESS_0x28 = 0x28;    ///< I2C address option 1
constexpr uint8_t I2C_ADDRESS_0x29 = 0x29;    ///< I2C address option 2 (default)
constexpr uint8_t I2C_ADDRESS_0x2A = 0x2A;    ///< I2C address option 3
constexpr uint8_t I2C_ADDRESS_0x2B = 0x2B;    ///< I2C address option 4
constexpr uint8_t I2C_ADDRESS_0x2C = 0x2C;    ///< I2C address option 5
constexpr uint8_t I2C_ADDRESS_0x2D = 0x2D;    ///< I2C address option 6
/** @} */

/**
 * @defgroup Device_ID Device Identification
 * @brief Device identification registers and expected values
 *
 * These registers contain fixed values that identify the device type,
 * manufacturer, and revision. Used for device detection and validation.
 * @{
 */
constexpr uint8_t REG_PRODUCT_ID = 0xFD;        ///< Product ID register
constexpr uint8_t REG_MANUFACTURER_ID = 0xFE;   ///< Manufacturer ID register
constexpr uint8_t REG_REVISION = 0xFF;          ///< Revision register

/// Expected product ID value (0x50 for CAP1188)
constexpr uint8_t EXPECTED_PRODUCT_ID = 0x50;
/// Expected manufacturer ID value (0x5D for Microchip)
constexpr uint8_t EXPECTED_MANUFACTURER_ID = 0x5D;
/// Expected revision value (0x82 for current revision)
constexpr uint8_t EXPECTED_REVISION = 0x82;
/** @} */

/**
 * @defgroup Core_Registers Core Control Registers
 * @brief Primary control and status registers
 *
 * These registers control the main device operation and provide
 * status information about touch detection and device state.
 * @{
 */
constexpr uint8_t REG_MAIN_CONTROL = 0x00;        ///< Main control register
constexpr uint8_t REG_GENERAL_STATUS = 0x02;      ///< General status register
constexpr uint8_t REG_SENSOR_INPUT_STATUS = 0x03; ///< Sensor input status register
constexpr uint8_t REG_NOISE_FLAG_STATUS = 0x0A;   ///< Noise flag status register
/** @} */

/**
 * @defgroup Threshold_Registers Threshold Registers
 * @brief Touch threshold and noise threshold registers
 *
 * These registers control the sensitivity thresholds for touch detection
 * and noise filtering. Each channel has its own threshold register.
 * @{
 */
constexpr uint8_t REG_SENSOR_INPUT_1_THRESHOLD = 0x30; ///< Channel 1 touch threshold
constexpr uint8_t REG_SENSOR_INPUT_2_THRESHOLD = 0x31; ///< Channel 2 touch threshold
constexpr uint8_t REG_SENSOR_INPUT_3_THRESHOLD = 0x32; ///< Channel 3 touch threshold
constexpr uint8_t REG_SENSOR_INPUT_4_THRESHOLD = 0x33; ///< Channel 4 touch threshold
constexpr uint8_t REG_SENSOR_INPUT_5_THRESHOLD = 0x34; ///< Channel 5 touch threshold
constexpr uint8_t REG_SENSOR_INPUT_6_THRESHOLD = 0x35; ///< Channel 6 touch threshold
constexpr uint8_t REG_SENSOR_INPUT_7_THRESHOLD = 0x36; ///< Channel 7 touch threshold
constexpr uint8_t REG_SENSOR_INPUT_8_THRESHOLD = 0x37; ///< Channel 8 touch threshold

constexpr uint8_t REG_SENSOR_INPUT_NOISE_THRESHOLD = 0x38; ///< Noise threshold register
/** @} */

/**
 * @defgroup Standby_Registers Standby Mode Registers
 * @brief Standby mode configuration and sensitivity control
 *
 * These registers control the device behavior in standby mode,
 * including which channels remain active and their sensitivity.
 * @{
 */
constexpr uint8_t REG_STANDBY_CHANNEL = 0x40;       ///< Standby channel selection
constexpr uint8_t REG_STANDBY_CONFIGURATION = 0x41; ///< Standby configuration
constexpr uint8_t REG_STANDBY_SENSITIVITY = 0x42;   ///< Standby sensitivity
constexpr uint8_t REG_STANDBY_THRESHOLD = 0x43;     ///< Standby threshold
/** @} */

/**
 * @defgroup Config_Registers Configuration Registers
 * @brief Device configuration and behavior control registers
 *
 * These registers control various aspects of device operation including
 * channel enable/disable, sensitivity settings, averaging, and interrupts.
 * @{
 */
constexpr uint8_t REG_CONFIGURATION = 0x20;                ///< Main configuration register
constexpr uint8_t REG_SENSOR_INPUT_ENABLE = 0x21;          ///< Sensor input enable register
constexpr uint8_t REG_SENSOR_INPUT_CONFIGURATION = 0x22;   ///< Sensor input configuration
constexpr uint8_t REG_SENSOR_INPUT_CONFIGURATION_2 = 0x23; ///< Sensor input configuration 2
constexpr uint8_t REG_AVERAGING_AND_SAMPLING_CONFIG = 0x24; ///< Averaging and sampling config
constexpr uint8_t REG_AVERAGING = 0x24;                    ///< Alias for averaging configuration
constexpr uint8_t REG_CALIBRATION_ACTIVATE = 0x26;        ///< Calibration activation register
constexpr uint8_t REG_INTERRUPT_ENABLE = 0x27;            ///< Interrupt enable register
constexpr uint8_t REG_REPEAT_RATE_ENABLE = 0x28;          ///< Repeat rate enable register
constexpr uint8_t REG_REPEAT_RATE = 0x28;                 ///< Alias for repeat rate configuration
/** @} */

/**
 * @defgroup MultiTouch_Registers Multiple Touch Registers
 * @brief Multiple touch detection configuration
 *
 * These registers control the multiple touch detection feature,
 * including pattern recognition and simultaneous touch handling.
 * @{
 */
constexpr uint8_t REG_MULTIPLE_TOUCH_CONFIG = 0x2A;        ///< Multiple touch configuration
constexpr uint8_t REG_MULTIPLE_TOUCH_PATTERN_CONFIG = 0x2B; ///< Multiple touch pattern config
constexpr uint8_t REG_MULTIPLE_TOUCH_PATTERN = 0x2D;       ///< Multiple touch pattern register
/** @} */

/**
 * @defgroup BaseCount_Registers Base Count Registers
 * @brief Channel base count (calibration) registers
 *
 * These registers contain the base count values for each channel,
 * which represent the calibrated "no touch" state. Used for
 * threshold calculations and touch detection.
 * @{
 */
constexpr uint8_t REG_SENSOR_INPUT_1_BASE_COUNT = 0x50; ///< Channel 1 base count
constexpr uint8_t REG_SENSOR_INPUT_2_BASE_COUNT = 0x51; ///< Channel 2 base count
constexpr uint8_t REG_SENSOR_INPUT_3_BASE_COUNT = 0x52; ///< Channel 3 base count
constexpr uint8_t REG_SENSOR_INPUT_4_BASE_COUNT = 0x53; ///< Channel 4 base count
constexpr uint8_t REG_SENSOR_INPUT_5_BASE_COUNT = 0x54; ///< Channel 5 base count
constexpr uint8_t REG_SENSOR_INPUT_6_BASE_COUNT = 0x55; ///< Channel 6 base count
constexpr uint8_t REG_SENSOR_INPUT_7_BASE_COUNT = 0x56; ///< Channel 7 base count
constexpr uint8_t REG_SENSOR_INPUT_8_BASE_COUNT = 0x57; ///< Channel 8 base count
/** @} */

/**
 * @defgroup LED_Control_Registers LED Control Registers
 * @brief LED output control and configuration registers
 *
 * These registers control the LED drivers including output type,
 * polarity, linking to touch channels, and direct control.
 * @{
 */
constexpr uint8_t REG_LED_OUTPUT_TYPE = 0x71;         ///< LED output type register
constexpr uint8_t REG_SENSOR_INPUT_LED_LINKING = 0x72; ///< Sensor to LED linking register
constexpr uint8_t REG_LED_POLARITY = 0x73;            ///< LED polarity register
constexpr uint8_t REG_LED_OUTPUT_CONTROL = 0x74;      ///< LED output control register
/** @} */

/**
 * @defgroup LED_Behavior_Registers LED Behavior Registers
 * @brief LED timing and effect configuration registers
 *
 * These registers control LED effects including pulse periods,
 * duty cycles, breathe effects, and ramp rates for smooth transitions.
 * @{
 */
constexpr uint8_t REG_LED_PULSE_1_PERIOD = 0x84;     ///< LED pulse 1 period register
constexpr uint8_t REG_LED_PULSE_2_PERIOD = 0x85;     ///< LED pulse 2 period register
constexpr uint8_t REG_LED_BREATHE_PERIOD = 0x86;     ///< LED breathe period register
constexpr uint8_t REG_LED_CONFIG = 0x88;             ///< LED configuration register
constexpr uint8_t REG_LED_PULSE_1_DUTY_CYCLE = 0x90; ///< LED pulse 1 duty cycle
constexpr uint8_t REG_LED_PULSE_2_DUTY_CYCLE = 0x91; ///< LED pulse 2 duty cycle
constexpr uint8_t REG_LED_BREATHE_DUTY_CYCLE = 0x92; ///< LED breathe duty cycle
constexpr uint8_t REG_LED_DIRECT_DUTY_CYCLE = 0x93;  ///< LED direct duty cycle
constexpr uint8_t REG_LED_DIRECT_RAMP_RATES = 0x94;  ///< LED direct ramp rates
constexpr uint8_t REG_LED_OFF_DELAY = 0x95;          ///< LED off delay register
/** @} */

/**
 * @defgroup Main_Control_Bits Main Control Register Bits
 * @brief Bit definitions for the main control register (0x00)
 *
 * These bits control the primary device operation modes including
 * power management, gain settings, and interrupt control.
 * @{
 */
constexpr uint8_t MAIN_CONTROL_INT = 0x01;    ///< Interrupt enable bit
constexpr uint8_t MAIN_CONTROL_DSLEEP = 0x02; ///< Deep sleep mode bit
constexpr uint8_t MAIN_CONTROL_STBY = 0x04;   ///< Standby mode bit
constexpr uint8_t MAIN_CONTROL_GAIN_1 = 0x00; ///< Gain setting: 1x
constexpr uint8_t MAIN_CONTROL_GAIN_2 = 0x20; ///< Gain setting: 2x
constexpr uint8_t MAIN_CONTROL_GAIN_4 = 0x40; ///< Gain setting: 4x
constexpr uint8_t MAIN_CONTROL_GAIN_8 = 0x60; ///< Gain setting: 8x
/** @} */

/**
 * @defgroup General_Status_Bits General Status Register Bits
 * @brief Bit definitions for the general status register (0x02)
 *
 * These bits provide status information about touch detection,
 * calibration, and various device conditions.
 * @{
 */
constexpr uint8_t GENERAL_STATUS_TOUCH = 0x01;      ///< Touch detected bit
constexpr uint8_t GENERAL_STATUS_MTP = 0x02;        ///< Multiple touch pattern bit
constexpr uint8_t GENERAL_STATUS_MULT = 0x04;       ///< Multiple touch detected bit
constexpr uint8_t GENERAL_STATUS_PWR = 0x08;        ///< Power status bit
constexpr uint8_t GENERAL_STATUS_ACAL_FAIL = 0x10;  ///< Auto-calibration failed bit
constexpr uint8_t GENERAL_STATUS_BC_OUT = 0x20;     ///< Base count out of range bit
/** @} */

/**
 * @defgroup Config_Bits Configuration Register Bits
 * @brief Bit definitions for the configuration register (0x20)
 *
 * These bits control various device configuration options including
 * timeout settings, noise filtering, and maximum duration control.
 * @{
 */
constexpr uint8_t CONFIG_TIMEOUT = 0x01;         ///< Timeout enable bit
constexpr uint8_t CONFIG_DIS_DIG_NOISE = 0x20;   ///< Disable digital noise filter bit
constexpr uint8_t CONFIG_DIS_ANA_NOISE = 0x40;   ///< Disable analog noise filter bit
constexpr uint8_t CONFIG_MAX_DUR_EN = 0x80;      ///< Maximum duration enable bit
/** @} */

/**
 * @defgroup Sensor_Config_Bits Sensor Configuration Register Bits
 * @brief Bit definitions for sensor configuration registers
 *
 * These bits control sensor-specific settings including delta sense
 * and other sensitivity-related parameters.
 * @{
 */
constexpr uint8_t SENSOR_CONFIG_DELTA_SENSE = 0x70; ///< Delta sense setting mask (bits 6:4)
/** @} */

/**
 * @defgroup Default_Values Default Configuration Values
 * @brief Recommended default values for device configuration
 *
 * These values provide good starting points for device configuration
 * and are used by the library's default initialization.
 * @{
 */
constexpr uint8_t DEFAULT_TOUCH_THRESHOLD = 0x40;      ///< Default touch threshold (64)
constexpr uint8_t DEFAULT_NOISE_THRESHOLD = 0x25;      ///< Default noise threshold (37)
constexpr uint8_t DEFAULT_STANDBY_CONFIG = 0x30;       ///< Default standby configuration
constexpr uint8_t DEFAULT_SENSITIVITY = 0x2F;          ///< Default sensitivity (47)
constexpr uint8_t DEFAULT_MULTI_TOUCH_CONFIG = 0x00;   ///< Default multi-touch config (disabled)
constexpr uint8_t DEFAULT_LED_LINKING = 0xFF;          ///< Default LED linking (all LEDs linked)
constexpr uint8_t DEFAULT_LED_POLARITY = 0x00;         ///< Default LED polarity (active low)
/** @} */

/**
 * @defgroup Timing_Constants Timing Constants
 * @brief Hardware timing requirements and timeouts
 *
 * These constants define the timing requirements for proper device
 * operation including power-on delays and I2C communication timeouts.
 * @{
 */
constexpr uint16_t POWER_ON_DELAY_MS = 15;  ///< Power-on delay in milliseconds
constexpr uint16_t RESET_DELAY_MS = 15;     ///< Reset delay in milliseconds
constexpr uint8_t I2C_TIMEOUT_MS = 100;     ///< I2C communication timeout in milliseconds
/** @} */

} // namespace CAP1188

/**
 * @page RegisterMap CAP1188 Register Map
 *
 * @section RegisterOverview Register Overview
 *
 * The CAP1188 contains numerous registers for configuration and status.
 * This page provides a comprehensive overview of the register organization.
 *
 * @subsection CoreRegs Core Registers
 * - 0x00: Main Control - Primary device control
 * - 0x02: General Status - Overall device status
 * - 0x03: Sensor Input Status - Touch detection status
 * - 0x0A: Noise Flag Status - Noise detection status
 *
 * @subsection ThresholdRegs Threshold Registers (0x30-0x38)
 * - 0x30-0x37: Individual channel thresholds (C1-C8)
 * - 0x38: Noise threshold
 *
 * @subsection ConfigRegs Configuration Registers (0x20-0x2D)
 * - 0x20: Main configuration
 * - 0x21: Sensor input enable
 * - 0x22-0x23: Sensor input configuration
 * - 0x24: Averaging and sampling
 * - 0x26: Calibration activation
 * - 0x27: Interrupt enable
 * - 0x28: Repeat rate
 * - 0x2A-0x2D: Multiple touch configuration
 *
 * @subsection BaseCountRegs Base Count Registers (0x50-0x57)
 * - 0x50-0x57: Channel base counts (C1-C8)
 *
 * @subsection LEDRegs LED Control Registers (0x71-0x95)
 * - 0x71-0x74: LED control and linking
 * - 0x84-0x86: LED timing (pulse/breathe periods)
 * - 0x88: LED configuration
 * - 0x90-0x95: LED duty cycles and effects
 *
 * @subsection StandbyRegs Standby Registers (0x40-0x43)
 * - 0x40-0x43: Standby mode configuration
 *
 * @subsection DeviceIDRegs Device ID Registers (0xFD-0xFF)
 * - 0xFD: Product ID (0x50)
 * - 0xFE: Manufacturer ID (0x5D)
 * - 0xFF: Revision (0x82)
 */