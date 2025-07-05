#pragma once

#include <cstdint>

namespace CAP1188 {

// Default I2C address
constexpr uint8_t DEFAULT_I2C_ADDRESS = 0x29;

// Available I2C addresses
constexpr uint8_t I2C_ADDRESS_0x28 = 0x28;
constexpr uint8_t I2C_ADDRESS_0x29 = 0x29;
constexpr uint8_t I2C_ADDRESS_0x2A = 0x2A;
constexpr uint8_t I2C_ADDRESS_0x2B = 0x2B;
constexpr uint8_t I2C_ADDRESS_0x2C = 0x2C;
constexpr uint8_t I2C_ADDRESS_0x2D = 0x2D;

// Device identification registers
constexpr uint8_t REG_PRODUCT_ID = 0xFD;
constexpr uint8_t REG_MANUFACTURER_ID = 0xFE;
constexpr uint8_t REG_REVISION = 0xFF;

// Expected device identification values
constexpr uint8_t EXPECTED_PRODUCT_ID = 0x50;
constexpr uint8_t EXPECTED_MANUFACTURER_ID = 0x5D;
constexpr uint8_t EXPECTED_REVISION = 0x82;

// Core control registers
constexpr uint8_t REG_MAIN_CONTROL = 0x00;
constexpr uint8_t REG_GENERAL_STATUS = 0x02;
constexpr uint8_t REG_SENSOR_INPUT_STATUS = 0x03;
constexpr uint8_t REG_NOISE_FLAG_STATUS = 0x0A;

// Sensor input threshold registers (C1-C8)
constexpr uint8_t REG_SENSOR_INPUT_1_THRESHOLD = 0x30;
constexpr uint8_t REG_SENSOR_INPUT_2_THRESHOLD = 0x31;
constexpr uint8_t REG_SENSOR_INPUT_3_THRESHOLD = 0x32;
constexpr uint8_t REG_SENSOR_INPUT_4_THRESHOLD = 0x33;
constexpr uint8_t REG_SENSOR_INPUT_5_THRESHOLD = 0x34;
constexpr uint8_t REG_SENSOR_INPUT_6_THRESHOLD = 0x35;
constexpr uint8_t REG_SENSOR_INPUT_7_THRESHOLD = 0x36;
constexpr uint8_t REG_SENSOR_INPUT_8_THRESHOLD = 0x37;

// Sensor input noise threshold registers
constexpr uint8_t REG_SENSOR_INPUT_NOISE_THRESHOLD = 0x38;

// Standby and sensitivity control
constexpr uint8_t REG_STANDBY_CHANNEL = 0x40;
constexpr uint8_t REG_STANDBY_CONFIGURATION = 0x41;
constexpr uint8_t REG_STANDBY_SENSITIVITY = 0x42;
constexpr uint8_t REG_STANDBY_THRESHOLD = 0x43;

// Configuration registers
constexpr uint8_t REG_CONFIGURATION = 0x20;
constexpr uint8_t REG_SENSOR_INPUT_ENABLE = 0x21;
constexpr uint8_t REG_SENSOR_INPUT_CONFIGURATION = 0x22;
constexpr uint8_t REG_SENSOR_INPUT_CONFIGURATION_2 = 0x23;
constexpr uint8_t REG_AVERAGING_AND_SAMPLING_CONFIG = 0x24;
constexpr uint8_t REG_AVERAGING = 0x24;  // Alias for averaging configuration
constexpr uint8_t REG_CALIBRATION_ACTIVATE = 0x26;
constexpr uint8_t REG_INTERRUPT_ENABLE = 0x27;
constexpr uint8_t REG_REPEAT_RATE_ENABLE = 0x28;
constexpr uint8_t REG_REPEAT_RATE = 0x28;  // Alias for repeat rate configuration

// Multiple touch configuration
constexpr uint8_t REG_MULTIPLE_TOUCH_CONFIG = 0x2A;
constexpr uint8_t REG_MULTIPLE_TOUCH_PATTERN_CONFIG = 0x2B;
constexpr uint8_t REG_MULTIPLE_TOUCH_PATTERN = 0x2D;

// Base count registers (C1-C8)
constexpr uint8_t REG_SENSOR_INPUT_1_BASE_COUNT = 0x50;
constexpr uint8_t REG_SENSOR_INPUT_2_BASE_COUNT = 0x51;
constexpr uint8_t REG_SENSOR_INPUT_3_BASE_COUNT = 0x52;
constexpr uint8_t REG_SENSOR_INPUT_4_BASE_COUNT = 0x53;
constexpr uint8_t REG_SENSOR_INPUT_5_BASE_COUNT = 0x54;
constexpr uint8_t REG_SENSOR_INPUT_6_BASE_COUNT = 0x55;
constexpr uint8_t REG_SENSOR_INPUT_7_BASE_COUNT = 0x56;
constexpr uint8_t REG_SENSOR_INPUT_8_BASE_COUNT = 0x57;

// LED output control registers
constexpr uint8_t REG_LED_OUTPUT_TYPE = 0x71;
constexpr uint8_t REG_SENSOR_INPUT_LED_LINKING = 0x72;
constexpr uint8_t REG_LED_POLARITY = 0x73;
constexpr uint8_t REG_LED_OUTPUT_CONTROL = 0x74;

// LED behavior registers
constexpr uint8_t REG_LED_PULSE_1_PERIOD = 0x84;
constexpr uint8_t REG_LED_PULSE_2_PERIOD = 0x85;
constexpr uint8_t REG_LED_BREATHE_PERIOD = 0x86;
constexpr uint8_t REG_LED_CONFIG = 0x88;
constexpr uint8_t REG_LED_PULSE_1_DUTY_CYCLE = 0x90;
constexpr uint8_t REG_LED_PULSE_2_DUTY_CYCLE = 0x91;
constexpr uint8_t REG_LED_BREATHE_DUTY_CYCLE = 0x92;
constexpr uint8_t REG_LED_DIRECT_DUTY_CYCLE = 0x93;
constexpr uint8_t REG_LED_DIRECT_RAMP_RATES = 0x94;
constexpr uint8_t REG_LED_OFF_DELAY = 0x95;

// Main Control Register bit definitions
constexpr uint8_t MAIN_CONTROL_INT = 0x01;
constexpr uint8_t MAIN_CONTROL_DSLEEP = 0x02;
constexpr uint8_t MAIN_CONTROL_STBY = 0x04;
constexpr uint8_t MAIN_CONTROL_GAIN_1 = 0x00;
constexpr uint8_t MAIN_CONTROL_GAIN_2 = 0x20;
constexpr uint8_t MAIN_CONTROL_GAIN_4 = 0x40;
constexpr uint8_t MAIN_CONTROL_GAIN_8 = 0x60;

// General Status Register bit definitions
constexpr uint8_t GENERAL_STATUS_TOUCH = 0x01;
constexpr uint8_t GENERAL_STATUS_MTP = 0x02;
constexpr uint8_t GENERAL_STATUS_MULT = 0x04;
constexpr uint8_t GENERAL_STATUS_PWR = 0x08;
constexpr uint8_t GENERAL_STATUS_ACAL_FAIL = 0x10;
constexpr uint8_t GENERAL_STATUS_BC_OUT = 0x20;

// Configuration Register bit definitions
constexpr uint8_t CONFIG_TIMEOUT = 0x01;
constexpr uint8_t CONFIG_DIS_DIG_NOISE = 0x20;
constexpr uint8_t CONFIG_DIS_ANA_NOISE = 0x40;
constexpr uint8_t CONFIG_MAX_DUR_EN = 0x80;

// Sensor Input Configuration bit definitions
constexpr uint8_t SENSOR_CONFIG_DELTA_SENSE = 0x70;

// Default configuration values
constexpr uint8_t DEFAULT_TOUCH_THRESHOLD = 0x40;
constexpr uint8_t DEFAULT_NOISE_THRESHOLD = 0x25;
constexpr uint8_t DEFAULT_STANDBY_CONFIG = 0x30;
constexpr uint8_t DEFAULT_SENSITIVITY = 0x2F;
constexpr uint8_t DEFAULT_MULTI_TOUCH_CONFIG = 0x00; // Disabled
constexpr uint8_t DEFAULT_LED_LINKING = 0xFF; // All LEDs linked
constexpr uint8_t DEFAULT_LED_POLARITY = 0x00; // Active low

// Timing constants (in milliseconds)
constexpr uint16_t POWER_ON_DELAY_MS = 15;
constexpr uint16_t RESET_DELAY_MS = 15;
constexpr uint8_t I2C_TIMEOUT_MS = 100;

} // namespace CAP1188