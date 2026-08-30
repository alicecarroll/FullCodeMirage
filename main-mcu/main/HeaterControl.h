#pragma once
#include <stdint.h>
#include <stdbool.h>

// Heater control modes - must match thermal-mcu definitions
typedef enum {
    HEATER_MODE_BANGBANG = 0,    // Hysteresis/Bang-Bang control
    HEATER_MODE_PID = 1,          // PID control
    HEATER_MODE_MANUAL = 155      // Manual control (155-255 can be manual with different duty cycles)
} HeaterControlMode;

// Heater control configuration for a single heater
struct HeaterConfig {
    uint8_t heater_id;           // 0-7
    HeaterControlMode mode;      // Current control mode
    int16_t target_temp;         // Target temperature (in 0.01°C units, e.g., 5000 = 50.00°C)
    uint8_t manual_duty_cycle;   // Only used when mode is HEATER_MODE_MANUAL (0-100)
    bool enabled;                // Whether heater is enabled
};

// Heater system state - manages all 8 heaters
struct HeaterSystem {
    HeaterConfig heaters[8];
};

// Initialize heater system
void heater_system_init(HeaterSystem* system);

// Set heater control mode
void heater_set_mode(HeaterSystem* system, uint8_t heater_id, HeaterControlMode mode);

// Set heater target temperature
void heater_set_target(HeaterSystem* system, uint8_t heater_id, int16_t target_temp);

// Set heater manual duty cycle (only effective in HEATER_MODE_MANUAL)
void heater_set_manual_duty(HeaterSystem* system, uint8_t heater_id, uint8_t duty_cycle);

// Enable/disable heater
void heater_set_enabled(HeaterSystem* system, uint8_t heater_id, bool enabled);

// Get heater configuration
HeaterConfig* heater_get_config(HeaterSystem* system, uint8_t heater_id);

// Send heater configuration to thermal MCU (requires i2c_master_write_to_device)
bool heater_send_config_to_thermal(uint8_t heater_id, int16_t current_temp);

// Get global heater system instance
HeaterSystem* heater_system_get_global();
