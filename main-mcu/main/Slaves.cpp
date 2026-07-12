#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "Settings.h"
#include "Multiplexer.h"
#include "Initialize.h"
//#include "communication.h"
#include "Slaves.h"
#include "read_sensors.h"

// Shared slave address
#define Slave_MCU_addr  0x42

// Internal helper
static bool select_slave(
    SlaveDevice slave,
    uint8_t* mux_channel,
    gpio_num_t* reset_pin)
{
    switch(slave)
    {
        case thermal_mcu:
            *mux_channel = multiplex_Tt3_devP;
            *reset_pin = Thermal_reset_PIN;
            return true;

        case pressure_mcu:
            *mux_channel = multiplex_Tp1_devT;
            *reset_pin = Preassure_reset_PIN;
            return true;
    }
    return false;
}

// Send telemetry/data
bool slave_send_data(
    SlaveDevice slave,
    SlaveData data_id,
    float value
)
{
    uint8_t mux_channel;
    gpio_num_t reset_pin;

    if (!select_slave(slave, &mux_channel, &reset_pin))
    {
        return false;
    }

    // Select correct mux channel
    sel_mux_channel(mux_channel);

    uint8_t packet[6];
    packet[0] = Slave_packet_data;
    packet[1] = data_id;
    memcpy(&packet[2], &value, sizeof(float));

    esp_err_t err = i2c_master_write_to_device(
        I2C_master,
        Slave_MCU_addr,
        packet,
        sizeof(packet),
        100 / portTICK_PERIOD_MS
    );

    return (err == ESP_OK);
}

// Send command
bool slave_send_command(
    SlaveDevice slave,
    SlaveCommand command
)
{
    uint8_t mux_channel;
    gpio_num_t reset_pin;

    if (!select_slave(slave, &mux_channel, &reset_pin))
    {
        return false;
    }

    sel_mux_channel(mux_channel);

    uint8_t packet[2];
    packet[0] = Slave_packet_command;
    packet[1] = command;

    esp_err_t err = i2c_master_write_to_device(
        I2C_master,
        Slave_MCU_addr,
        packet,
        sizeof(packet),
        100 / portTICK_PERIOD_MS
    );

    return (err == ESP_OK);
}

/**
 * Sends combined operating state parameters to a slave via the standard command packet system.
 * Packs Mode, Emergency status, Pressure Status, and an 8-bit heater selection mask.
 */
bool slave_send_complex_state(
    SlaveDevice slave,
    bool emergency_stop,
    bool autonomous_mode,
    bool pressure_system_on,
    uint8_t heater_mask
)
{
    uint8_t mux_channel;
    gpio_num_t reset_pin;

    if (!select_slave(slave, &mux_channel, &reset_pin))
    {
        return false;
    }

    sel_mux_channel(mux_channel);

    // Byte 0: Packet identifier
    // Byte 1: Control flags (Bit 0: Emergency, Bit 1: Auto/Manual, Bit 2: Pressure System)
    // Byte 2: Heater Activation Bitmask (Bit 0 = Heater 1, Bit 1 = Heater 2...)
    uint8_t packet[3];
    packet[0] = Slave_packet_command; 
    
    packet[1] = 0;
    if (emergency_stop)      packet[1] |= (1 << 0);
    if (autonomous_mode)     packet[1] |= (1 << 1);
    if (pressure_system_on)  packet[1] |= (1 << 2);

    packet[2] = heater_mask;

    esp_err_t err = i2c_master_write_to_device(
        I2C_master,
        Slave_MCU_addr,
        packet,
        sizeof(packet),
        100 / portTICK_PERIOD_MS
    );

    return (err == ESP_OK);
}

// Update persistent setting
bool slave_update_setting(
    SlaveDevice slave,
    uint8_t setting,
    float value)
{
    uint8_t mux_channel;
    gpio_num_t reset_pin;

    if (!select_slave(slave, &mux_channel, &reset_pin))
    {
        return false;
    }

    uint8_t packet[6];
    packet[0] = Slave_packet_setting;
    packet[1] = setting;
    memcpy(&packet[2], &value, sizeof(float));

    esp_err_t err = i2c_master_write_to_device(
        I2C_master,
        Slave_MCU_addr,
        packet,
        sizeof(packet),
        100 / portTICK_PERIOD_MS
    );

    return (err == ESP_OK);
}

void recover_i2c_driver() {
    ESP_LOGW("I2C_DEBUG", "Hard resetting I2C peripheral...");
    
    // 1. Delete the current driver instance
    i2c_driver_delete(I2C_master);
    
    // 2. Wait for pending operations to clear
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 3. Re-install the driver using your existing initialization
    init_i2c(); 
}

// Read slave status (Modified to capture the ping counter byte)
bool slave_read_status(
    SlaveDevice slave,
    SlaveStatus* status)
{
    uint8_t mux_channel;
    gpio_num_t reset_pin;

    if (!select_slave(slave, &mux_channel, &reset_pin))
    {
        return false;
    }

    sel_mux_channel(mux_channel);

    // Expecting 3 bytes from slave: [State, Error Code, Rolling Ping Counter]
    uint8_t data[3];

    esp_err_t err = i2c_master_read_from_device(
        I2C_master,
        Slave_MCU_addr,
        data,
        sizeof(data),
        100 / portTICK_PERIOD_MS
    );

    if (err != ESP_OK)
    {
        status->online = false;
        if (err != ESP_OK) {
            ESP_LOGE("I2C_DEBUG", "I2C Transaction Failed: 0x%02X", err);
            if (err==0xFFFFFFFF) {
                ESP_LOGE("I2C_DEBUG", "I2C bus may be stuck. Attempting recovery...");
                recover_i2c_driver();
            }
        }
        return false; // Main loop will catch this false and eventually trigger reset
    }

    status->online = true;
    status->state = data[0];
    status->error = data[1];
    
    return true;
}

// Reset slave MCU
void slave_reset(SlaveDevice slave)
{
    uint8_t mux_channel;
    gpio_num_t reset_pin;

    if (!select_slave(slave, &mux_channel, &reset_pin))
    {
        return;
    }

    // REMOVED: gpio_config() calls that were causing the conflict.
    // Instead, just perform the pulse on the existing, pre-configured pin.
    
    gpio_set_level(reset_pin, 0); // Pulse low
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(reset_pin, 1); // Release
}