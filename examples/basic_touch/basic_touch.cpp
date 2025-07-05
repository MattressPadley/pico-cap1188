#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "cap1188/cap1188.hpp"

using namespace CAP1188;

// Configuration
constexpr uint8_t CAP1188_ADDRESS = DEFAULT_I2C_ADDRESS;  // 0x29
constexpr uint SDA_PIN = PICO_DEFAULT_I2C_SDA_PIN;        // GPIO 4
constexpr uint SCL_PIN = PICO_DEFAULT_I2C_SCL_PIN;        // GPIO 5
// Reset pin disabled by default (no GPIO interference)
constexpr uint BAUDRATE = 100000;                         // 100 kHz I2C

// Note: Using CAP1188's built-in LEDs instead of external GPIO LEDs

// Global CAP1188 device instance
CAP1188Device touch_sensor(i2c_default, CAP1188_ADDRESS);  // No reset pin

// Track previous touch state to detect changes
uint8_t previous_touched = 0x00;

// I2C scan function
void i2c_scan() {
    printf("\nScanning I2C bus for devices...\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
    
    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }
        
        // Skip reserved addresses
        if ((addr & 0x78) == 0 || (addr & 0x78) == 0x78) {
            printf("   ");
        } else {
            uint8_t rxdata;
            int ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);
            if (ret >= 0) {
                printf("%02x ", addr);
            } else {
                printf("-- ");
            }
        }
        
        if (addr % 16 == 15) {
            printf("\n");
        }
    }
    printf("I2C scan complete.\n\n");
}

// Touch event callback function
void on_touch_event(const TouchEvent& event) {
    printf("Touch %s on channel %s (C%d)\n",
           event.pressed ? "PRESS" : "RELEASE",
           channelToString(event.channel),
           channelToIndex(event.channel) + 1);
}

// Error callback function
void on_error(Error error, const char* message) {
    printf("CAP1188 Error: %s", errorToString(error));
    if (message) {
        printf(" - %s", message);
    }
    printf("\n");
}

// External LEDs removed - using CAP1188's built-in LEDs only

// External LED update removed - using CAP1188's built-in LEDs only

// Configure the CAP1188 device
Error configure_device() {
    printf("Configuring CAP1188...\n");
    
    // Create device configuration using human-readable settings
    DeviceConfig config;
    config.sensitivity = TouchSensitivity::MEDIUM;           // Balanced touch sensitivity
    config.response_speed = TouchResponseSpeed::MEDIUM;      // Standard response speed
    config.stability = TouchStability::BALANCED;             // Good balance of speed vs stability
    config.noise_filtering = NoiseFiltering::MEDIUM;         // Standard noise filtering
    config.multi_touch = MultiTouchMode::ENABLED;           // Allow multiple touches
    config.led_behavior = LEDBehavior::TOUCH_FEEDBACK;       // LEDs on while touched
    config.led_speed = LEDSpeed::MEDIUM;                     // Standard LED timing
    config.led_active_high = false;                         // CAP1188 LEDs are active low
    config.interrupts_enabled = false;                      // Disable interrupts (not using INT pin)
    config.auto_recalibration = true;                       // Enable auto-calibration
    
    Error err = touch_sensor.setConfiguration(config);
    if (err != Error::SUCCESS) {
        printf("Failed to set device configuration: %s\n", errorToString(err));
        return err;
    }
    
    // Configure individual channels using human-readable settings
    TouchConfig channel_config;
    channel_config.enabled = true;
    channel_config.sensitivity = TouchSensitivity::USE_GLOBAL;  // Use device-wide sensitivity
    channel_config.led_behavior = LEDBehavior::USE_GLOBAL;      // Use device-wide LED behavior
    channel_config.repeat_enabled = false;                      // No auto-repeat
    channel_config.noise_threshold = 0x25;
    
    err = touch_sensor.setChannelConfig(TouchChannel::ALL, channel_config);
    if (err != Error::SUCCESS) {
        printf("Failed to set channel configuration: %s\n", errorToString(err));
        return err;
    }
    
    // Set LED polarity to match our configuration
    err = touch_sensor.setLEDPolarity(config.led_active_high);
    if (err != Error::SUCCESS) {
        printf("Failed to set LED polarity: %s\n", errorToString(err));
        return err;
    }
    
    // Calibrate all channels
    printf("Calibrating touch sensors...\n");
    err = touch_sensor.calibrateAllChannels();
    if (err != Error::SUCCESS) {
        printf("Failed to calibrate channels: %s\n", errorToString(err));
        return err;
    }
    
    // Wait for calibration to complete
    sleep_ms(100);
    
    printf("CAP1188 configuration complete\n");
    return Error::SUCCESS;
}

// Print device information
void print_device_info() {
    printf("\n=== CAP1188 Device Information ===\n");
    printf("Product ID: 0x%02X (expected: 0x%02X)\n", 
           touch_sensor.getProductID(), EXPECTED_PRODUCT_ID);
    printf("Manufacturer ID: 0x%02X (expected: 0x%02X)\n", 
           touch_sensor.getManufacturerID(), EXPECTED_MANUFACTURER_ID);
    printf("Revision: 0x%02X (expected: 0x%02X)\n", 
           touch_sensor.getRevision(), EXPECTED_REVISION);
    
    // Print base counts for each channel
    printf("\nChannel Base Counts:\n");
    for (int i = 0; i < 8; ++i) {
        TouchChannel channel = static_cast<TouchChannel>(i);
        uint8_t base_count = touch_sensor.getBaseCount(channel);
        uint8_t threshold = touch_sensor.getChannelThreshold(channel);
        printf("  C%d: Base=%3d, Threshold=%3d\n", i + 1, base_count, threshold);
    }
    
    // Print device status
    DeviceStatus status = touch_sensor.getStatus();
    printf("\nDevice Status:\n");
    printf("  Touch Detected: %s\n", status.touch_detected ? "YES" : "NO");
    printf("  Multiple Touch: %s\n", status.multiple_touch ? "YES" : "NO");
    printf("  Noise Detected: %s\n", status.noise_detected ? "YES" : "NO");
    printf("  Calibration Failed: %s\n", status.calibration_failed ? "YES" : "NO");
    printf("  Baseline Out of Range: %s\n", status.baseline_out_of_range ? "YES" : "NO");
    printf("=================================\n\n");
}

// Main touch detection loop
void touch_loop() {
    printf("Starting touch detection loop...\n");
    printf("Touch the capacitive pads to see detection events.\n");
    printf("Watch for runtime configuration demonstrations every 15 seconds.\n");
    printf("Press Ctrl+C to exit.\n\n");
    
    while (true) {
        // Read current touch state
        uint8_t current_touched = touch_sensor.getTouchedChannels();
        
        // Clear interrupt flag for proper release detection
        touch_sensor.clearInterrupt();
        
        // Check for changes in touch state
        if (current_touched != previous_touched) {
            // Determine which channels changed
            uint8_t changed = current_touched ^ previous_touched;
            
            for (int i = 0; i < 8; ++i) {
                if (changed & (1 << i)) {
                    TouchChannel channel = static_cast<TouchChannel>(i);
                    bool pressed = (current_touched & (1 << i)) != 0;
                    
                    // Create and send touch event
                    TouchEvent event(channel, pressed, to_us_since_boot(get_absolute_time()));
                    on_touch_event(event);
                }
            }
            
            // CAP1188's built-in LEDs will update automatically via linked_led configuration
            
            previous_touched = current_touched;
        }
        
        // Check device status periodically
        static uint32_t last_status_check = 0;
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_status_check > 5000) {  // Check every 5 seconds
            DeviceStatus status = touch_sensor.getStatus();
            if (status.calibration_failed) {
                printf("Warning: Calibration failed, attempting recalibration...\n");
                touch_sensor.calibrateAllChannels();
            }
            if (status.baseline_out_of_range) {
                printf("Warning: Baseline out of range detected\n");
            }
            last_status_check = now;
        }
        
        // Demonstrate runtime configuration changes every 15 seconds
        static uint32_t last_config_demo = 0;
        static int demo_stage = 0;
        if (now - last_config_demo > 15000) {  // Every 15 seconds
            switch (demo_stage) {
                case 0:
                    printf("\n--- Runtime Config Demo: Increasing sensitivity ---\n");
                    touch_sensor.updateSensitivity(TouchSensitivity::HIGH);
                    printf("Sensitivity updated to HIGH\n\n");
                    break;
                case 1:
                    printf("\n--- Runtime Config Demo: Faster response ---\n");
                    touch_sensor.updateResponseSpeed(TouchResponseSpeed::FAST);
                    printf("Response speed updated to FAST\n\n");
                    break;
                case 2:
                    printf("\n--- Runtime Config Demo: LED pulse effect ---\n");
                    touch_sensor.updateLEDBehavior(LEDBehavior::PULSE_ON_TOUCH, LEDSpeed::FAST);
                    printf("LED behavior updated to PULSE_ON_TOUCH with FAST speed\n\n");
                    break;
                case 3:
                    printf("\n--- Runtime Config Demo: Batch touch settings ---\n");
                    touch_sensor.updateTouchSettings(
                        TouchSensitivity::MEDIUM,
                        TouchResponseSpeed::MEDIUM,
                        TouchStability::BALANCED
                    );
                    printf("Touch settings updated to balanced defaults\n\n");
                    break;
                case 4:
                    printf("\n--- Runtime Config Demo: Smart configuration merge ---\n");
                    {
                        DeviceConfig new_config = touch_sensor.getConfiguration();
                        new_config.sensitivity = TouchSensitivity::LOW;
                        new_config.led_behavior = LEDBehavior::TOUCH_FEEDBACK;
                        new_config.noise_filtering = NoiseFiltering::HEAVY;
                        // Only changed settings will be applied
                        touch_sensor.updateConfiguration(new_config);
                        printf("Configuration merged: LOW sensitivity, TOUCH_FEEDBACK LEDs, HEAVY noise filtering\n\n");
                    }
                    break;
                case 5:
                    printf("\n--- Runtime Config Demo: Per-channel sensitivity ---\n");
                    touch_sensor.updateChannelSensitivity(TouchChannel::C1, TouchSensitivity::VERY_HIGH);
                    touch_sensor.updateChannelSensitivity(TouchChannel::C2, TouchSensitivity::HIGH);
                    printf("Channel C1 set to VERY_HIGH sensitivity, C2 set to HIGH sensitivity\n\n");
                    break;
                default:
                    printf("\n--- Runtime Config Demo: Reset to defaults ---\n");
                    touch_sensor.updateTouchSettings(
                        TouchSensitivity::MEDIUM,
                        TouchResponseSpeed::MEDIUM,
                        TouchStability::BALANCED
                    );
                    touch_sensor.updateLEDBehavior(LEDBehavior::TOUCH_FEEDBACK, LEDSpeed::MEDIUM);
                    touch_sensor.updateNoiseFiltering(NoiseFiltering::MEDIUM);
                    printf("All settings reset to defaults\n\n");
                    demo_stage = -1; // Will wrap to 0 on increment
                    break;
            }
            demo_stage++;
            last_config_demo = now;
        }
        
        // Small delay to prevent excessive polling
        sleep_ms(10);
    }
}

int main() {
    // Initialize stdio for console output
    stdio_init_all();
    
    // Wait for USB serial connection
    printf("Waiting for USB serial connection...\n");
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    
    // Wait a moment for console to initialize
    sleep_ms(1000);
    
    printf("CAP1188 Basic Touch Example with Runtime Configuration\n");
    printf("======================================================\n");
    printf("This example demonstrates:\n");
    printf("- Human-readable CAP1188 configuration system\n");
    printf("- Runtime configuration changes without reinitialization\n");
    printf("- No GPIO interference (reset pin disabled by default)\n");
    printf("- Pre-Initialized I2C Pattern for multi-device support\n\n");
    
    // Initialize I2C hardware (pre-initialized pattern)
    printf("Initializing I2C hardware...\n");
    i2c_init(i2c_default, BAUDRATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    
    // Perform I2C scan to verify devices
    i2c_scan();
    
    // Debug: Read device identity registers directly
    printf("Reading device identity registers...\n");
    uint8_t product_id, manufacturer_id, revision;
    uint8_t reg_data;
    
    // Read Product ID (register 0xFD)
    int ret = i2c_write_blocking(i2c_default, CAP1188_ADDRESS, (uint8_t[]){0xFD}, 1, true);
    if (ret >= 0) {
        ret = i2c_read_blocking(i2c_default, CAP1188_ADDRESS, &product_id, 1, false);
        if (ret >= 0) {
            printf("Product ID: 0x%02X (expected: 0x50)\n", product_id);
        } else {
            printf("Failed to read Product ID\n");
        }
    } else {
        printf("Failed to write Product ID register address\n");
    }
    
    // Read Manufacturer ID (register 0xFE)
    ret = i2c_write_blocking(i2c_default, CAP1188_ADDRESS, (uint8_t[]){0xFE}, 1, true);
    if (ret >= 0) {
        ret = i2c_read_blocking(i2c_default, CAP1188_ADDRESS, &manufacturer_id, 1, false);
        if (ret >= 0) {
            printf("Manufacturer ID: 0x%02X (expected: 0x5D)\n", manufacturer_id);
        } else {
            printf("Failed to read Manufacturer ID\n");
        }
    } else {
        printf("Failed to write Manufacturer ID register address\n");
    }
    
    // Read Revision (register 0xFF)
    ret = i2c_write_blocking(i2c_default, CAP1188_ADDRESS, (uint8_t[]){0xFF}, 1, true);
    if (ret >= 0) {
        ret = i2c_read_blocking(i2c_default, CAP1188_ADDRESS, &revision, 1, false);
        if (ret >= 0) {
            printf("Revision: 0x%02X (expected: 0x83)\n", revision);
        } else {
            printf("Failed to read Revision\n");
        }
    } else {
        printf("Failed to write Revision register address\n");
    }
    
    printf("\n");
    
    // Using CAP1188's built-in LEDs - no external LED initialization needed
    
    // Set up error callback
    touch_sensor.setErrorCallback(on_error);
    
    // Initialize the CAP1188 device (no baudrate parameter)
    printf("Initializing CAP1188...\n");
    Error err = touch_sensor.begin();
    if (err != Error::SUCCESS) {
        printf("Failed to initialize CAP1188: %s\n", errorToString(err));
        printf("Check I2C connections and device address.\n");
        return -1;
    }
    
    printf("CAP1188 initialized successfully!\n");
    
    // Check if device is connected
    if (!touch_sensor.isConnected()) {
        printf("Warning: Device connection check failed\n");
    }
    
    // Configure the device
    err = configure_device();
    if (err != Error::SUCCESS) {
        printf("Failed to configure device, continuing anyway...\n");
    }
    
    // Print device information
    print_device_info();
    
    // Run the main touch detection loop
    touch_loop();
    
    return 0;
}