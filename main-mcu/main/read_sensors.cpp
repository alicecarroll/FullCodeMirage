#include <stdio.h>
#include <cstdio>
#include "esp_err.h"


#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Settings.h"
#include "Multiplexer.h"
#include "read_sensors.h"

SensorData sensor_data;

static const char *TAG = "read_sensors";
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

// ABP2 is single-shot only. Must trigger conversion, wait, then read.
static void read_abp2(float *pressure,
                      float *temperature)
{
    uint8_t data[7];
    uint8_t cmd[3] = {0xAA, 0x00, 0x00}; // High precision measurement command

    // 1. Send the command to wake the sensor up and trigger a measurement
    esp_err_t err_write = i2c_master_write_to_device(
        I2C_master,
        ABP2_addr,
        cmd,
        sizeof(cmd),
        pdMS_TO_TICKS(20));

    if (err_write != ESP_OK)
    {
        printf("ABP2 measurement trigger failed: %s\n", esp_err_to_name(err_write));
        return;
    }

    // 2. Wait for the sensor to finish measuring (High-precision takes max 8.3ms)
    vTaskDelay(pdMS_TO_TICKS(10));

    esp_err_t err_read = i2c_master_read_from_device(
        I2C_master,
        ABP2_addr,
        data,
        7,
        pdMS_TO_TICKS(20));

    if (err_read != ESP_OK)
    {
        printf("ABP2 read failed: %s\n", esp_err_to_name(err_read));
        return;
    }

    // Pressure
    if (pressure != nullptr) {
        const float OUTPUT_MIN = 1677722.0f;
        const float OUTPUT_MAX = 15099494.0f;

        uint32_t pressure_counts =
        ((uint32_t)data[1] << 16) |
        ((uint32_t)data[2] << 8) |
        data[3];

        *pressure =
            ((pressure_counts - OUTPUT_MIN) *
            30.0f) /
            (OUTPUT_MAX - OUTPUT_MIN);

        *pressure *= 0.0689476f;  // psi -> bar

    }

    // Temperature
    if (temperature != nullptr)
    {
        uint32_t temperature_counts =
        ((uint32_t)data[4] << 16) |
        ((uint32_t)data[5] << 8) |
        data[6];

        *temperature = temperature_counts * 200.0f / 16777215.0f - 50.0f;
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

void read_ms5803(MS5803_Calibration *cal, float *pressure, float *temperature)
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

    // First-order compensation
    int32_t dT = (int32_t)D2 - ((int32_t)cal->C[5] << 8);

    int64_t TEMP =
        2000 + (((int64_t)dT * cal->C[6]) >> 23);

    int64_t OFF =
        ((int64_t)cal->C[2] << 16) +
        (((int64_t)cal->C[4] * dT) >> 7);

    int64_t SENS =
        ((int64_t)cal->C[1] << 15) +
        (((int64_t)cal->C[3] * dT) >> 8);


    // Second-order compensation
    int64_t T2 = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;

    // The different sensor models have different second-order compensation requirements
    if (cal->model == MS5803Model::MS5803_01BA)
    {

        if (TEMP < 2000)
        {
            T2 = ((int64_t)dT * dT) >> 31;

            OFF2 = 3 * ((TEMP - 2000) * (TEMP - 2000));

            SENS2 =
                7 * ((TEMP - 2000) * (TEMP - 2000)) >> 3;

            if (TEMP < -1500)
            {
                SENS2 +=
                    2 * ((TEMP + 1500) * (TEMP + 1500));
            }
        }
        else if (TEMP >= 4500)
        {
            SENS2 =
                ((TEMP - 4500) * (TEMP - 4500)) >> 3;
        }
    } 

    else if (cal->model == MS5803Model::MS5803_14BA)
    {
        if (TEMP < 2000)
        {
            T2 = 3* ((int64_t)dT * dT) >> 33;

            OFF2 = 3 * ((TEMP - 2000) * (TEMP - 2000)) >> 1;

            SENS2 =
                5 * ((TEMP - 2000) * (TEMP - 2000)) >> 3;

            if (TEMP < -1500)
            {
                OFF2 += 7 * ((TEMP + 1500) * (TEMP + 1500));
                SENS2 +=
                    4 * ((TEMP + 1500) * (TEMP + 1500));
            }
        }
        else
        {
            T2 = 7 * ((int64_t)dT * dT) >> 37;
            OFF2 = 1* ((TEMP - 2000) * (TEMP - 2000)) >> 4;
        }
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;


    int64_t P =
        ((((int64_t)D1 * SENS) >> 21) - OFF) >> 15;

    
    // The sensors need different scaling factors to convert the raw pressure value to bar
    if (cal->model == MS5803Model::MS5803_01BA)
    {
        P /= 100000; // Convert to bar
    }
    else if (cal->model == MS5803Model::MS5803_14BA)
    {
        P /= 10000; // Convert to bar
    }

    if (pressure != nullptr)
    {
        *pressure = (float)P;
    }

    if (temperature != nullptr)
    {
        *temperature = (float)TEMP / 100.0f;
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

    // Channel 0: RTC + Tp2
    sel_mux_channel(multiplex_RTC_Tp2);
    //printf("Switching to MUX channel: %i\n", multiplex_RTC_Tp2);

    vTaskDelay(pdMS_TO_TICKS(20));

    read_ds3231(
        &sensor_data.hours,
        &sensor_data.minutes,
        &sensor_data.seconds);

    read_tmp1075(&sensor_data.Tp2);

    // Channel 1: Tp1
    sel_mux_channel(multiplex_Tp1_devT);
    //printf("Switching to MUX channel: %i\n", multiplex_Tp1_devT);

    //vTaskDelay(pdMS_TO_TICKS(20));

    read_tmp1075(&sensor_data.Tp1);

    // Channel 2: Ambient sensors (temperature, humidity, preassure)
    sel_mux_channel(multiplex_Ambient);
    //printf("Switching to MUX channel: %i\n", multiplex_Ambient);

    //vTaskDelay(pdMS_TO_TICKS(20));

    read_tmp117(&sensor_data.Ta1);

    read_sht45(
        &sensor_data.Ta3,
        &sensor_data.Ha1);

    read_ms5803(
        &pa1_cal,
        &sensor_data.Pa1,
        &sensor_data.Ta2);


    // Channel 3: Tp4 + Pp1 + Tp5 + Pp2
    sel_mux_channel(multiplex_Tp4_Pp1_Tp5_Pp2);
    //printf("Switching to MUX channel: %i\n", multiplex_Tp4_Pp1_Tp5_Pp2);

    //vTaskDelay(pdMS_TO_TICKS(20));

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
        &sensor_data.Pp2,
        &sensor_data.Tp5);

    // Channel 4: Tp3
    sel_mux_channel(multiplex_Tp3);
    //printf("Switching to MUX channel: %i\n", multiplex_Tp3);

    //vTaskDelay(pdMS_TO_TICKS(20));
    read_tmp1075(&sensor_data.Tp3);

    // Channel 5: Tt1 + Tt2
    sel_mux_channel(multiplex_Outlet);
    //printf("Switching to MUX channel: %i\n", multiplex_Outlet);

    //vTaskDelay(pdMS_TO_TICKS(20));

    read_sht45(
        &sensor_data.Tt1,
        nullptr);
    //vTaskDelay(pdMS_TO_TICKS(20));
    read_tmp1075(&sensor_data.Tt2); 

    // Channel 6: Tp6 + Pp3
    sel_mux_channel(multiplex_Tp6_Pp3);
    //printf("Switching to MUX channel: %i\n", multiplex_Tp6_Pp3);

    //vTaskDelay(pdMS_TO_TICKS(20));
    
    read_abp2(
        &sensor_data.Pp3,
        &sensor_data.Tp6);

    // Channel 7: Tt3
    sel_mux_channel(multiplex_Tt3_devP);
    //printf("Switching to MUX channel: %i\n", multiplex_Tt3_devP);

    //vTaskDelay(pdMS_TO_TICKS(20));

    read_tmp117(
        &sensor_data.Tt3);
}

void write_csv_header(FILE *f)
{
    fprintf(f, "hours,minutes,seconds,"
               "Tp1,Tp2,Tp3,Tp6,Pp3,Tp4,Pp1,Pa1,Ta1,Ta2,Ha1,Tp5,Pp2,"
               "Tt1,Tt2,Tt3,"
               "K96_CO2,K96_pressure,K96_temperature,K96_humidity,K96_error\n");
}

void write_csv_row(FILE *f, const SensorData *data)
{
    if (data == nullptr) return;

    fprintf(f, "%02u,%02u,%02u,"
               "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
               "%.2f,%.2f,%.2f,"
               "%.2f,%.2f,%.2f,%.2f,%u\n",
            data->hours, data->minutes, data->seconds,
            data->Tp1, data->Tp2, data->Tp3, data->Tp6, data->Pp3, data->Tp4,
            data->Pp1, data->Pa1, data->Ta1, data->Ta2, data->Ta3, data->Ha1, data->Tp5, data->Pp2,
            data->Tt1, data->Tt2, data->Tt3,
            data->K96_CO2, data->K96_pressure, data->K96_temperature,
            data->K96_humidity, data->K96_error);
}

void log_sensor_data(const SensorData *data, const char *filepath)
{
    // Check if file exists to decide whether to write the header
    FILE *check = fopen(filepath, "r");
    bool need_header = (check == nullptr);
    if (check) fclose(check);

    FILE *f = fopen(filepath, "a"); // append mode
    if (f == nullptr) {
        ESP_LOGE(TAG, "Failed to open %s for writing", filepath);
        return;
    }

    if (need_header) {
        write_csv_header(f);
    }
    write_csv_row(f, data);

    fclose(f);
}