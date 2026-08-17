#include "HeaterControl.h"
#include "Slaves.h"
#include "Settings.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "HeaterControl";

// Global heater system instance
static HeaterSystem global_heater_system;

// Initialize heater system with default values
void heater_system_init(HeaterSystem* system)
{
    // Initialize all 8 heaters with default settings
    for (int i = 0; i < 8; i++) {
        system->heaters[i].heater_id = i;
        system->heaters[i].mode = HEATER_MODE_BANGBANG;  // Default to bang-bang control
        system->heaters[i].target_temp = 2000;           // Default to 20.00°C
        system->heaters[i].manual_duty_cycle = 0;
        system->heaters[i].enabled = false;
    }
    ESP_LOGI(TAG, "Heater system initialized");
}

// Set heater control mode
void heater_set_mode(HeaterSystem* system, uint8_t heater_id, HeaterControlMode mode)
{
    if (heater_id >= 8) {
        ESP_LOGW(TAG, "Invalid heater ID: %d", heater_id);
        return;
    }
    
    system->heaters[heater_id].mode = mode;
    
    const char* mode_str = "";
    switch (mode) {
        case HEATER_MODE_BANGBANG:
            mode_str = "Bang-Bang";
            break;
        case HEATER_MODE_PID:
            mode_str = "PID";
            break;
        case HEATER_MODE_MANUAL:
            mode_str = "Manual";
            break;
        default:
            mode_str = "Unknown";
    }
    
    ESP_LOGI(TAG, "Heater %d set to %s control mode", heater_id, mode_str);
}

// Set heater target temperature
void heater_set_target(HeaterSystem* system, uint8_t heater_id, int16_t target_temp)
{
    if (heater_id >= 8) {
        ESP_LOGW(TAG, "Invalid heater ID: %d", heater_id);
        return;
    }
    
    system->heaters[heater_id].target_temp = target_temp;
    ESP_LOGI(TAG, "Heater %d target set to %.2f°C", heater_id, target_temp / 100.0f);
}

// Set heater manual duty cycle
void heater_set_manual_duty(HeaterSystem* system, uint8_t heater_id, uint8_t duty_cycle)
{
    if (heater_id >= 8) {
        ESP_LOGW(TAG, "Invalid heater ID: %d. Must be 1-8", heater_id);
        return;
    }
    
    if (duty_cycle > 100) {
        ESP_LOGW(TAG, "Invalid duty cycle: %d. Must be 0-100", duty_cycle);
        return;
    }
    
    system->heaters[heater_id].manual_duty_cycle = duty_cycle;
    ESP_LOGI(TAG, "Heater %d manual duty cycle set to %d%%", heater_id, duty_cycle);
}

// Enable/disable heater
void heater_set_enabled(HeaterSystem* system, uint8_t heater_id, bool enabled)
{
    if (heater_id >= 8) {
        ESP_LOGW(TAG, "Invalid heater ID: %d", heater_id);
        return;
    }
    
    system->heaters[heater_id].enabled = enabled;
    ESP_LOGI(TAG, "Heater %d is now %s", heater_id, enabled ? "ENABLED" : "DISABLED");
}

// Get heater configuration
HeaterConfig* heater_get_config(HeaterSystem* system, uint8_t heater_id)
{
    if (heater_id >= 8) {
        ESP_LOGW(TAG, "Invalid heater ID: %d", heater_id);
        return nullptr;
    }
    
    return &system->heaters[heater_id];
}

// Send heater configuration to thermal MCU
// This wraps the thermal_test_send_package function from Slaves.cpp
bool heater_send_config_to_thermal(uint8_t heater_id, int16_t current_temp)
{
    if (heater_id >= 8) {
        ESP_LOGW(TAG, "Invalid heater ID: %d", heater_id);
        return false;
    }
    
    HeaterConfig* config = heater_get_config(&global_heater_system, heater_id);
    if (!config) {
        return false;
    }
    
    // Convert mode to appropriate value for thermal MCU
    // If manual mode, send the duty cycle value (200-255), otherwise send mode
    uint8_t mode_to_send = (config->mode == HEATER_MODE_MANUAL) 
                          ? config->manual_duty_cycle 
                          : static_cast<uint8_t>(config->mode);
    
    // Send to thermal MCU via I2C
    return thermal_test_send_package(
        thermal_mcu,
        heater_id,
        mode_to_send,
        current_temp,
        config->target_temp
    );
}

// Get global heater system instance
HeaterSystem* heater_system_get_global()
{
    return &global_heater_system;
}
