#include <stdio.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Settings.h"
#include "Multiplexer.h"
#include "read_sensors.h"

SensorData sensor_data;

// continous sensor
static void read_tmp1075(float *temperature)
{
    uint8_t temp_reg = 0x00;
    uint8_t data[2];

    esp_err_t err = i2c_master_write_read_device(
        I2C_master,
        TMP1075_addr,
        &temp_reg,
        1,
        data,
        2,
        pdMS_TO_TICKS(20));

    if (err != ESP_OK)
    {
        printf("TMP1075 read failed: %s\n", esp_err_to_name(err));
        return;
    }

    int16_t raw =
        (data[0] << 8) | data[1];

    raw >>= 4;

    if (temperature != nullptr)
    {
        *temperature =
            raw * 0.0625f;
    }
}

// Continuous pressure + temperature
static void read_abp2(float *pressure,
                      float *temperature)
{
    uint8_t data[4];

    esp_err_t err = i2c_master_read_from_device(
        I2C_master,
        ABP2_addr,
        data,
        4,
        pdMS_TO_TICKS(20));

    if (err != ESP_OK)
    {
        printf("ABP2 read failed: %s\n", esp_err_to_name(err));
        return;
    }

    // Pressure
    if (pressure != nullptr)
    {
        uint16_t raw_pressure =
            ((data[0] & 0x3F) << 8) | data[1];

        *pressure =
            ((float)(raw_pressure - 1638) * 150.0f) /
            (14745.0f - 1638.0f);
    }

    // Temperature
    if (temperature != nullptr)
    {
        uint16_t raw_temperature =
            (data[2] << 3) |
            ((data[3] & 0xE0) >> 5);

        *temperature =
            ((float)raw_temperature * 200.0f / 2047.0f) - 50.0f;
    }
}

// Continuous temperature sensor
static void read_tmp117(float *temperature)
{
    uint8_t reg = 0x00;
    uint8_t data[2];

    esp_err_t err = i2c_master_write_read_device(
        I2C_master,
        TMP117_addr,
        &reg,
        1,
        data,
        2,
        pdMS_TO_TICKS(20));

    if (err != ESP_OK)
    {
        printf("TMP117 read failed: %s\n", esp_err_to_name(err));
        return;
    }

    int16_t raw =
        (data[0] << 8) | data[1];

    if (temperature != nullptr)
    {
        *temperature =
            raw * 0.0078125f;
    }
}

/**
 * Helper function to calculate the Sensirion SHT45 CRC-8 checksum.
 * Polynomial: 0x31 (x^8 + x^5 + x^4 + 1), Initialization: 0xFF
 */
static uint8_t sensirion_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x31;
            }
            else
            {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

// SHT45 is single-shot only. Must trigger conversion, wait, then read.
static void read_sht45(float *temperature, float *humidity)
{
    uint8_t cmd = 0xFD; // High precision measurement command
    uint8_t data[6];

    // 1. Send the command to wake the sensor up and trigger a measurement
    esp_err_t err = i2c_master_write_to_device(
        I2C_master,
        SHT45_addr,
        &cmd,
        1,
        pdMS_TO_TICKS(20));

    if (err != ESP_OK)
    {
        printf("SHT45 measurement trigger failed: %s\n", esp_err_to_name(err));
        return;
    }

    // 2. Wait for the sensor to finish measuring (High-precision takes max 8.3ms)
    vTaskDelay(pdMS_TO_TICKS(10));

    // 3. Read the 6-byte result (Temp MSB/LSB/CRC + Humidity MSB/LSB/CRC)
    err = i2c_master_read_from_device(
        I2C_master,
        SHT45_addr,
        data,
        6,
        pdMS_TO_TICKS(20));

    if (err != ESP_OK)
    {
        printf("SHT45 data read failed: %s\n", esp_err_to_name(err));
        return;
    }

    // 4. Validate Temperature Checksum
    // data[0]=MSB, data[1]=LSB, data[2]=CRC
    if (sensirion_crc8(&data[0], 2) != data[2])
    {
        printf("SHT45 Temperature CRC Check Failed! Data corrupted on I2C bus.\n");
        return; // Reject corrupted data
    }

    // 5. Validate Humidity Checksum
    // data[3]=MSB, data[4]=LSB, data[5]=CRC
    if (sensirion_crc8(&data[3], 2) != data[5])
    {
        printf("SHT45 Humidity CRC Check Failed! Data corrupted on I2C bus.\n");
        return; // Reject corrupted data
    }

    // Temperature parsing (Executes only if CRC passed)
    if (temperature != NULL) // Replaced C++ nullptr with C compatible NULL
    {
        uint16_t raw_temp = (data[0] << 8) | data[1];
        *temperature = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
    }

    // Humidity parsing (Executes only if CRC passed)
    if (humidity != NULL)
    {
        uint16_t raw_humidity = (data[3] << 8) | data[4];
        *humidity = -6.0f + 125.0f * ((float)raw_humidity / 65535.0f);
    }
}

// NOT continuous, must trigger conversion every read
MS5803_Calibration pa1_cal;
MS5803_Calibration pp2_cal;

static void read_ms5803(MS5803_Calibration *cal, float *pressure)
{
    uint8_t cmd;
    uint8_t data[3];

    uint32_t D1;
    uint32_t D2;

    // Start pressure conversion (D1), OSR = 4096
    cmd = 0x48;

    esp_err_t err = i2c_master_write_to_device(
        I2C_master,
        MS5803_addr,
        &cmd,
        1,
        pdMS_TO_TICKS(20));

    if (err != ESP_OK)
    {
        printf("MS5803 command write failed: %s\n", esp_err_to_name(err));
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    cmd = 0x00;

    err = i2c_master_write_read_device(
        I2C_master,
        MS5803_addr,
        &cmd,
        1,
        data,
        3,
        pdMS_TO_TICKS(20));

    if (err != ESP_OK)
    {
        printf("MS5803 read failed: %s\n", esp_err_to_name(err));
        return;
    }

    D1 =
        ((uint32_t)data[0] << 16) |
        ((uint32_t)data[1] << 8) |
        data[2];

    cmd = 0x58;

    err = i2c_master_write_to_device(
        I2C_master,
        MS5803_addr,
        &cmd,
        1,
        pdMS_TO_TICKS(20));

    if (err != ESP_OK)
    {
        printf("MS5803 conversion command failed: %s\n", esp_err_to_name(err));
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    cmd = 0x00;

    err = i2c_master_write_read_device(
        I2C_master,
        MS5803_addr,
        &cmd,
        1,
        data,
        3,
        pdMS_TO_TICKS(20));

    if (err != ESP_OK)
    {
        printf("MS5803 read failed: %s\n", esp_err_to_name(err));
        return;
    }

    D2 =
        ((uint32_t)data[0] << 16) |
        ((uint32_t)data[1] << 8) |
        data[2];

    // Compensation calculations
    int32_t dT = D2 - ((uint32_t)cal->C[5] << 8);

    int64_t OFF = ((int64_t)cal->C[2] << 16) + (((int64_t)cal->C[4] * dT) >> 7);

    int64_t SENS = ((int64_t)cal->C[1] << 15) + (((int64_t)cal->C[3] * dT) >> 8);

    int32_t P = ((((int64_t)D1 * SENS) >> 21) - OFF) >> 15;

    if (pressure != nullptr)
    {
        *pressure = (float)P; // Pressure in pas
    }
}

// BCD - Decimal conversion for DS3231 RTC
static uint8_t bcd_to_decimal(uint8_t bcd)
{
    return ((bcd >> 4) * 10) +
           (bcd & 0x0F);
}

// DS3231 RTC READ
static void read_ds3231(uint8_t *hours, uint8_t *minutes, uint8_t *seconds)
{
    uint8_t reg = 0x00;
    uint8_t data[3];

    esp_err_t err = i2c_master_write_read_device(
        I2C_master,
        RTC_addr,
        &reg,
        1,
        data,
        3,
        pdMS_TO_TICKS(20));

    if (err != ESP_OK)
    {
        printf("RTC read failed: %s\n", esp_err_to_name(err));
        return;
    }

    // Seconds
    if (seconds != nullptr)
    {
        *seconds =
            bcd_to_decimal(data[0] & 0x7F);
    }

    // Minutes
    if (minutes != nullptr)
    {
        *minutes =
            bcd_to_decimal(data[1] & 0x7F);
    }

    // Hours
    if (hours != nullptr)
    {
        *hours =
            bcd_to_decimal(data[2] & 0x3F);
    }
}

// Collect all sensor data
void read_sensors()
{
    // Channel 0: Tp1
    sel_mux_channel(multiplex_Tp1_devT);
    printf("Switching to MUX channel: %i\n", multiplex_Tp1_devT);

    vTaskDelay(pdMS_TO_TICKS(20));

    read_tmp1075(&sensor_data.Tp1);

    // Channel 1: RTC + Tp2
    sel_mux_channel(multiplex_RTC_Tp2);
    printf("Switching to MUX channel: %i\n", multiplex_RTC_Tp2);

    vTaskDelay(pdMS_TO_TICKS(20));

    read_ds3231(
        &sensor_data.hours,
        &sensor_data.minutes,
        &sensor_data.seconds);

    read_tmp1075(&sensor_data.Tp2);

    // Channel 2: Ambient sensors (temperature, humidity, preassure)
    sel_mux_channel(multiplex_Ambient);
    printf("Switching to MUX channel: %i\n", multiplex_Ambient);

    vTaskDelay(pdMS_TO_TICKS(20));

    read_sht45(
        &sensor_data.Ta2,
        &sensor_data.Ha1);

    read_ms5803(
        &pa1_cal,
        &sensor_data.Pa1);


    read_tmp117(
        &sensor_data.Ta1);


    // Channel 3: Tp4 + Pp1 + Tp5 + Pp2
    sel_mux_channel(multiplex_Tp4_Pp1_Tp5_Pp2);
    printf("Switching to MUX channel: %i\n", multiplex_Tp4_Pp1_Tp5_Pp2);

    vTaskDelay(pdMS_TO_TICKS(20));

    // ABP2 pipe pressure + temperature
    read_abp2(
        &sensor_data.Pp1,
        &sensor_data.Tp4);

    // Chamber temperature, but currently not connected to anything
    //read_sht45(
    //    &sensor_data.Tp5,
    //    nullptr);

    // Chamber pressure
    read_ms5803(
        &pp2_cal,
        &sensor_data.Pp2);

    // Channel 4: Tp3
    sel_mux_channel(multiplex_Tp3);
    printf("Switching to MUX channel: %i\n", multiplex_Tp3);

    vTaskDelay(pdMS_TO_TICKS(20));
    read_tmp1075(&sensor_data.Tp3);

    // Channel 5: Tt1 + Tt2
    sel_mux_channel(multiplex_Outlet);
    printf("Switching to MUX channel: %i\n", multiplex_Outlet);

    vTaskDelay(pdMS_TO_TICKS(20));

    read_sht45(
        &sensor_data.Tt1,
        nullptr);
    vTaskDelay(pdMS_TO_TICKS(20));
    read_tmp1075(&sensor_data.Tt2); // The tt2 port seems to give weird readings, independent on which tmp1075 is connected.

    // Channel 6: Tp6 + Pp3
    sel_mux_channel(multiplex_Tp6_Pp3);
    printf("Switching to MUX channel: %i\n", multiplex_Tp6_Pp3);

    vTaskDelay(pdMS_TO_TICKS(20));
    
    read_abp2(
        &sensor_data.Pp3,
        &sensor_data.Tp6);

    // Channel 7: Tt3
    sel_mux_channel(multiplex_Tt3_devP);
    printf("Switching to MUX channel: %i\n", multiplex_Tt3_devP);

    vTaskDelay(pdMS_TO_TICKS(20));

    read_tmp117(
        &sensor_data.Tt3);
}