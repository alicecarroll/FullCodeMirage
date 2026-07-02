#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "Settings.h"
#include "read_sensors.h"
#include "uart.h"
#include <math.h>

void K96_on()
{
    gpio_set_level(K96_EN_PIN, 1);
}

void K96_off()
{
    gpio_set_level(K96_EN_PIN, 0);
}

// Creates checksum for Modbus, used by K96 for error checking
static uint16_t modbus_crc16(
    const uint8_t *data,
    uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

// Reads one data point from K96 RAM by calling adress and number of bytes to read, returns true if successful
static bool K96_read_ram(
    uint16_t ram_address,
    uint8_t num_bytes,
    uint8_t *response)
{
    uint8_t frame[7];

    // Device address (K96 default is 0x68)
    frame[0] = 0x68;

    // Function code (0x44 for reading RAM)
    frame[1] = 0x44;

    // RAM address
    frame[2] =
        (ram_address >> 8) & 0xFF;

    frame[3] =
        ram_address & 0xFF;

    // Number of bytes to read
    frame[4] = num_bytes;

    // CRC16
    uint16_t crc =
        modbus_crc16(frame, 5);

    frame[5] =
        crc & 0xFF;

    frame[6] =
        (crc >> 8) & 0xFF;

    // Clear old UART RX data
    uart_flush(UART_PORT);

    // Send MODBUS frame
    uart_write_bytes(
        UART_PORT,
        (const char *)frame,
        sizeof(frame));

    // Read response
    int len =
        uart_read_bytes(
            UART_PORT,
            response,
            num_bytes + 5,
            pdMS_TO_TICKS(1000));

    return (len == (num_bytes + 5));
}

void read_k96()
{
    uint8_t response[16];

    // CO2 concentration
    if (K96_read_ram(0x038C, 2, response))
    {
        int16_t raw =
            (response[3] << 8) |
            response[4];

        sensor_data.K96_CO2 =
            (float)raw;
    }
    else
    {
        sensor_data.K96_CO2 = NAN;
    }

    // Pressure
    if (K96_read_ram(0x01D0, 2, response))
    {
        int16_t raw =
            (response[3] << 8) |
            response[4];

        sensor_data.K96_pressure =
            raw * 0.1f; // hPa
    }
    else
    {
        sensor_data.K96_pressure = NAN;
    }

    // Humidity
    if (K96_read_ram(0x01F0, 2, response))
    {
        int16_t raw =
            (response[3] << 8) |
            response[4];

        sensor_data.K96_humidity =
            raw * 0.01f;
    }
    else
    {
        sensor_data.K96_humidity = NAN;
    }

    // Temperature
    if (K96_read_ram(0x01F8, 2, response))
    {
        int16_t raw =
            (response[3] << 8) |
            response[4];

        sensor_data.K96_temperature =
            raw * 0.01f;
    }
    else
    {
        sensor_data.K96_temperature = NAN;
    }

    // Error status
    if (K96_read_ram(0x001C, 2, response))
    {
        sensor_data.K96_error =
            (response[3] << 8) |
            response[4];
    }
    else
    {
        sensor_data.K96_error = 0xFFFF;
    }
}