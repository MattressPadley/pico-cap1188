#include <stdio.h>
#include "pico/stdlib.h"
#include "cap1188/cap1188.hpp"

using namespace CAP1188;

// Configuration
constexpr uint8_t CAP1188_ADDRESS = DEFAULT_I2C_ADDRESS;  // 0x29
constexpr uint SDA_PIN = PICO_DEFAULT_I2C_SDA_PIN;        // GPIO 4
constexpr uint SCL_PIN = PICO_DEFAULT_I2C_SCL_PIN;        // GPIO 5
constexpr uint RESET_PIN = 22;                            // Optional: GPIO 22 for reset
constexpr uint BAUDRATE = 100000;                         // 100 kHz I2C

// Optional: LED pins for visual feedback (connect LEDs with current limiting resistors)
constexpr uint LED_PINS[8] = {10, 11, 12, 13, 14, 15, 16, 17};

// Global CAP1188 device instance
CAP1188Device touch_sensor(i2c_default, CAP1188_ADDRESS, SDA_PIN, SCL_PIN, RESET_PIN);

// Track previous touch state to detect changes
uint8_t previous_touched = 0x00;

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

// Initialize external LEDs (optional)
void init_external_leds() {
    for (int i = 0; i < 8; ++i) {
        gpio_init(LED_PINS[i]);
        gpio_set_dir(LED_PINS[i], GPIO_OUT);
        gpio_put(LED_PINS[i], false);  // Start with LEDs off
    }
}

// Update external LEDs based on touch state
void update_external_leds(uint8_t touched_channels) {
    for (int i = 0; i < 8; ++i) {
        bool touched = (touched_channels & (1 << i)) != 0;
        gpio_put(LED_PINS[i], touched);
    }
}

// Configure the CAP1188 device
Error configure_device() {
    printf("Configuring CAP1188...\n");
    
    // Create device configuration
    DeviceConfig config;
    config.multi_touch_enabled = true;        // Allow multiple simultaneous touches
    config.interrupts_enabled = true;         // Enable interrupt generation
    config.digital_noise_filter = true;       // Enable noise filtering
    config.analog_noise_filter = true;
    config.led_active_high = false;           // CAP1188 LEDs are typically active low
    config.gain = Gain::GAIN_1X;              // Start with 1x gain
    
    Error err = touch_sensor.setConfiguration(config);
    if (err != Error::SUCCESS) {
        printf("Failed to set device configuration: %s\n", errorToString(err));
        return err;
    }
    
    // Configure individual channels
    TouchConfig channel_config;
    channel_config.threshold = 0x40;          // Moderate sensitivity
    channel_config.enabled = true;
    channel_config.linked_led = true;         // Link LEDs to touch detection
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
    printf("Press Ctrl+C to exit.\n\n");
    
    while (true) {
        // Read current touch state
        uint8_t current_touched = touch_sensor.getTouchedChannels();
        
        // Check for changes in touch state
        if (current_touched != previous_touched) {
            // Determine which channels changed
            uint8_t changed = current_touched ^ previous_touched;
            
            for (int i = 0; i < 8; ++i) {
                if (changed & (1 << i)) {
                    TouchChannel channel = static_cast<TouchChannel>(i);
                    bool pressed = (current_touched & (1 << i)) != 0;
                    
                    // Create and send touch event
                    TouchEvent event(channel, pressed, get_absolute_time()._private_us_since_boot);
                    on_touch_event(event);
                }
            }
            
            // Update external LEDs
            update_external_leds(current_touched);
            
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
        
        // Small delay to prevent excessive polling
        sleep_ms(10);
    }
}

int main() {
    // Initialize stdio for console output
    stdio_init_all();
    
    // Wait a moment for console to initialize
    sleep_ms(2000);
    
    printf("CAP1188 Basic Touch Example\n");
    printf("===========================\n");
    
    // Initialize external LEDs (optional)
    init_external_leds();
    
    // Set up error callback
    touch_sensor.setErrorCallback(on_error);
    
    // Initialize the CAP1188 device
    printf("Initializing CAP1188...\n");
    Error err = touch_sensor.begin(BAUDRATE);
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