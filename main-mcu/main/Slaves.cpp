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

// Internal helper
static bool select_slave(
    SlaveDevice slave,
    uint8_t* mux_channel,
    gpio_num_t* reset_pin)
{
    switch(slave)
    {
        case thermal_mcu:

            *mux_channel =
                multiplex_Tp1_devT;

            *reset_pin =
                Thermal_reset_PIN;

            return true;


        case pressure_mcu:

            *mux_channel =
                multiplex_Tt3_devP;

            *reset_pin =
                Preassure_reset_PIN;

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

    if (!select_slave(
            slave,
            &mux_channel,
            &reset_pin))
    {
        return false;
    }

    // Select correct mux channel
    sel_mux_channel(
        mux_channel
    );

    uint8_t packet[6];

    packet[0] =
        Slave_packet_data;

    packet[1] =
        data_id;

    memcpy(
        &packet[2],
        &value,
        sizeof(float)
    );

    esp_err_t err =
        i2c_master_write_to_device(
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

    if (!select_slave(
            slave,
            &mux_channel,
            &reset_pin))
    {
        return false;
    }

    sel_mux_channel(
        mux_channel
    );

    uint8_t packet[2];

    packet[0] =
        PACKET_COMMAND;

    packet[1] =
        command;

    esp_err_t err =
        i2c_master_write_to_device(
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

    if (!select_slave(
            slave,
            &mux_channel,
            &reset_pin))
    {
        return false;
    }

    uint8_t packet[6];

    packet[0] = Slave_packet_setting;

    packet[1] = setting;

    memcpy(
        &packet[2],
        &value,
        sizeof(float)
    );

    esp_err_t err =
        i2c_master_write_to_device(
            I2C_master,
            Slave_MCU_addr,
            packet,
            sizeof(packet),
            100 / portTICK_PERIOD_MS
        );

    return (err == ESP_OK);
}

// Read slave status
bool slave_read_status(
    SlaveDevice slave,
    SlaveStatus* status)
{
    uint8_t mux_channel;
    gpio_num_t reset_pin;

    if (!select_slave(
            slave,
            &mux_channel,
            &reset_pin))
    {
        return false;
    }

    sel_mux_channel(
        mux_channel
    );

    uint8_t data[2];

    esp_err_t err =
        i2c_master_read_from_device(
            I2C_master,
            Slave_MCU_addr,
            data,
            sizeof(data),
            100 / portTICK_PERIOD_MS
        );

    if (err != ESP_OK)
    {
        status->online = false;

        return false;
    }

    status->online = true;

    status->state =
        data[0];

    status->error =
        data[1];

    return true;
}

// Reset slave MCU
void slave_reset(
    SlaveDevice slave)
{
    uint8_t mux_channel;
    gpio_num_t reset_pin;

    if (!select_slave(
            slave,
            &mux_channel,
            &reset_pin))
    {
        return;
    }

    sel_mux_channel(
        mux_channel
    );

    gpio_set_level(
        reset_pin,
        0
    );

    vTaskDelay(
        pdMS_TO_TICKS(100)
    );

    gpio_set_level(
        reset_pin,
        1
    );
}