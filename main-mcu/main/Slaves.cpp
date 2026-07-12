#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "Settings.h"
#include "Multiplexer.h"
//#include "communication.h"
#include "Slaves.h"
#include "read_sensors.h"

// Shared slave address
#define Slave_MCU_addr  0x42

// Watchdog tracking metrics
#define WATCHDOG_TIMEOUT_MS 5000
static uint32_t last_thermal_ping = 0;
static uint32_t last_pressure_ping = 0;
static uint8_t last_thermal_ping_val = 0;
static uint8_t last_pressure_ping_val = 0;

// Internal helper
static bool select_slave(
    SlaveDevice slave,
    uint8_t* mux_channel,
    gpio_num_t* reset_pin)
{
    switch(slave)
    {
        case thermal_mcu:
            *mux_channel = multiplex_Tp1_devT;
            *reset_pin = Thermal_reset_PIN;
            return true;

        case pressure_mcu:
            *mux_channel = multiplex_Tt3_devP;
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

    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (err != ESP_OK)
    {
        status->online = false;
        return false;
    }

    status->online = true;
    status->state = data[0];
    status->error = data[1];

    // Read the rolling ping index byte to reset the local watchdog clocks
    uint8_t received_ping = data[2];
    if (slave == thermal_mcu)
    {
        if (received_ping != last_thermal_ping_val)
        {
            last_thermal_ping = current_time;
            last_thermal_ping_val = received_ping;
        }
    }
    else if (slave == pressure_mcu)
    {
        if (received_ping != last_pressure_ping_val)
        {
            last_pressure_ping = current_time;
            last_pressure_ping_val = received_ping;
        }
    }

    return true;
}

// Reset slave MCU
void slave_reset(
    SlaveDevice slave)
{
    uint8_t mux_channel;
    gpio_num_t reset_pin;

    if (!select_slave(slave, &mux_channel, &reset_pin))
    {
        return;
    }

    // Configure pin explicitly to guarantee output state
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << reset_pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // Pulse BOOT0 Low to trigger restart
    gpio_set_level(reset_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Release back to high/input mode so it enters execution mode naturally
    gpio_set_level(reset_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_direction(reset_pin, GPIO_MODE_INPUT);
}

/**
 * FreeRTOS Background Watchdog Task.
 * Spun up in application initialization to verify both MCUs are responsive.
 */
void slave_watchdog_task(void *pvParameters)
{
    SlaveStatus temp_status;
    uint32_t current_time;
    
    // Initialize the timestamps
    last_thermal_ping = xTaskGetTickCount() * portTICK_PERIOD_MS;
    last_pressure_ping = last_thermal_ping;

    while (1)
    {
        current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Check Thermal Slave
        slave_read_status(thermal_mcu, &temp_status);
        if ((current_time - last_thermal_ping) > WATCHDOG_TIMEOUT_MS)
        {
            slave_reset(thermal_mcu);
            last_thermal_ping = xTaskGetTickCount() * portTICK_PERIOD_MS; // Give time to boot
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // Stagger multiplexer readings

        // Check Pressure Slave
        slave_read_status(pressure_mcu, &temp_status);
        if ((current_time - last_pressure_ping) > WATCHDOG_TIMEOUT_MS)
        {
            slave_reset(pressure_mcu);
            last_pressure_ping = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Poll once per second
    }
}