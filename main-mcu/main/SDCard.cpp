//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// 100% Calude.................
//  Why use FatFS: Mainly: you can plug it in to a computer. https://docs.espressif.com/projects/esp-idf/en/latest/esp32c5/api-guides/file-system-considerations.html
//                   https://www.engineersgarage.com/esp32-sd-card-emmc-filesystems/
// Inspiration: https://github.com/espressif/esp-idf/blob/526f682397a8cfb74698c601fd2c5b30e1433837/examples/storage/sd_card/main/sd_card_example_main.c
//https://github.com/espressif/esp-idf/blob/v6.0.1/examples/storage/fatfs/getting_started/main/fatfs_getting_started_main.c

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
/**
 * SDCard.cpp
 *
 * SD card specific operations only.
 * Does NOT initialise the SPI bus — that is done once in Initialize.cpp.
 *
 * CS pin: IO9 (CS1)
 */

#include "SDCard.h"
#include "Settings.h"
#include "read_sensors.h"

#include <stdio.h>
#include <errno.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

        static const char *TAG = "SDCard";

static sdmmc_card_t *s_card = NULL;
static bool s_mounted = false;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
// hello

/*
* Buffers with immediate storing
// ===================================================================
// Option 1: Binary buffer (fastest, smallest)
// ===================================================================
void buffer_SD_data_binary_single()
{
    uint8_t buf[sizeof(sensor_data)];
    memcpy(buf, &sensor_data, sizeof(buf));
    sd_write("sensor_data.bin", buf, sizeof(buf));
    ESP_LOGI(TAG, "Buffered %zu bytes (binary)", sizeof(buf));
}

// ===================================================================
// Option 2: CSV text format (readable, larger)
// ===================================================================
void buffer_SD_data_csv_single()
{
    char line[512];
    int n = snprintf(line, sizeof(line),
        "%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u\n",
        sensor_data.hours,
        sensor_data.minutes,
        sensor_data.seconds,
        sensor_data.Tp1,
        sensor_data.Tp2,
        sensor_data.Tp3,
        sensor_data.Tp6,
        sensor_data.Pp3,
        sensor_data.Tp4,
        sensor_data.Pp1,
        sensor_data.Pa1,
        sensor_data.Ta1,
        sensor_data.Ta2,
        sensor_data.Ha1,
        sensor_data.Tp5,
        sensor_data.Pp2,
        sensor_data.Tt1,
        sensor_data.Tt2,
        sensor_data.Tt3,
        sensor_data.K96_CO2,
        sensor_data.K96_pressure,
        sensor_data.K96_temperature,
        sensor_data.K96_humidity,
        sensor_data.K96_error
    );
    sd_write("sensor_data.csv", (const uint8_t *)line, n);
    ESP_LOGI(TAG, "Buffered %d bytes (CSV)", n);
}
*/


// Buffer configuration
//#define SD_BUFFER_SIZE 4096
#define SENSOR_READING_SIZE sizeof(SensorData)
#define READINGS_PER_BUFFER (SD_BUFFER_SIZE / SENSOR_READING_SIZE)

static uint8_t SD_buffer[SD_BUFFER_SIZE];
static size_t SD_buffer_offset = 0;

// ===================================================================
// Option 1: Binary buffer large (fastest, smallest)
// ===================================================================
void buffer_SD_data_binary(const SensorData *sensor_data)
{
    // Check if there's space for another reading
    if (SD_buffer_offset + SENSOR_READING_SIZE <= SD_BUFFER_SIZE)
    {
        // Copy current sensor reading into buffer
        memcpy(&SD_buffer[SD_buffer_offset], sensor_data, SENSOR_READING_SIZE);
        SD_buffer_offset += SENSOR_READING_SIZE;
    }
    
    // Write to SD when buffer is full
    if (SD_buffer_offset >= SD_BUFFER_SIZE)
    {
        esp_err_t err = sd_write("sensor_data.bin", SD_buffer, SD_BUFFER_SIZE);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Wrote %d readings (%zu bytes) to SD", 
                     READINGS_PER_BUFFER, SD_BUFFER_SIZE);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to write buffer to SD");
        }
        SD_buffer_offset = 0;  // Reset for next batch
    }
}

void buffer_SD_data_csv(const SensorData *sensor_data)
{
    //Create temp CSV line to store
    char line[512];
    int n = snprintf(line, sizeof(line),
        "%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u\n",
        sensor_data->hours,
        sensor_data->minutes,
        sensor_data->seconds,
        sensor_data->Tp1,
        sensor_data->Tp2,
        sensor_data->Tp3,
        sensor_data->Tp6,
        sensor_data->Pp3,
        sensor_data->Tp4,
        sensor_data->Pp1,
        sensor_data->Pa1,
        sensor_data->Ta1,
        sensor_data->Ta2,
        sensor_data->Ha1,
        sensor_data->Tp5,
        sensor_data->Pp2,
        sensor_data->Tt1,
        sensor_data->Tt2,
        sensor_data->Tt3,
        sensor_data->K96_CO2,
        sensor_data->K96_pressure,
        sensor_data->K96_temperature,
        sensor_data->K96_humidity,
        sensor_data->K96_error
    );

    // If the line doesn't fit, flush current buffer first
    // Extra check since csv can be variable length and might exceed buffer size on its own,
    // in that case we should write it directly instead of trying to buffer it
    if ((size_t)n + SD_buffer_offset >= SD_BUFFER_SIZE)
    {
        esp_err_t err = sd_write("sensor_data.csv", SD_buffer, SD_buffer_offset);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Flushed %zu bytes CSV to SD", SD_buffer_offset);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to flush CSV buffer to SD");
        }
        SD_buffer_offset = 0;
    }

    // Append the line bytes into the buffer
    memcpy(&SD_buffer[SD_buffer_offset], line, (size_t)n);
    SD_buffer_offset += (size_t)n;

    // If buffer full after append, write it out
    if (SD_buffer_offset >= SD_BUFFER_SIZE)
    {
        esp_err_t err = sd_write("sensor_data.csv", SD_buffer, SD_BUFFER_SIZE);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Wrote %zu bytes CSV to SD", SD_BUFFER_SIZE);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to write CSV buffer to SD");
        }
        SD_buffer_offset = 0;  // Reset for next batch
    }
}
// Flush remaining data (call before shutdown)
void buffer_SD_data_flush()
{
    if (SD_buffer_offset > 0)
    {
        esp_err_t err = sd_write("sensor_data.bin", SD_buffer, SD_buffer_offset);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Flushed %zu bytes to SD", SD_buffer_offset);
        }
        SD_buffer_offset = 0;
    }
}


// ---------------------------------------------------------------------------
// Application-facing functions
// ---------------------------------------------------------------------------

esp_err_t sd_mount(void)
{
    if (s_mounted)
    {
        return ESP_OK;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CS_SD_PIN;
    slot_config.host_id = SPI2_HOST;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 512,
        .disk_status_check_enable = false,
        .use_one_fat = false,
    };

    esp_err_t ret = esp_vfs_fat_sdspi_mount(
        SD_MOUNT_POINT,
        &host,
        &slot_config,
        &mount_config,
        &s_card);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return ret;
    }

    s_mounted = true;
    ESP_LOGI(TAG, "SD card mounted at %s", SD_MOUNT_POINT);
    return ESP_OK;
}

void sd_unmount(void)
{
    if (!s_mounted)
    {
        return;
    }

    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
    s_card = NULL;
    s_mounted = false;
    ESP_LOGI(TAG, "SD card unmounted");
}

esp_err_t sd_write(const char *filename, const uint8_t *data, size_t length)
{
    char path[SD_MAX_PATH_LEN];
    snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, filename);

    // "ab" = append
    FILE *f = fopen(path, "ab");
    if (!f)
    {
        ESP_LOGE(TAG, "Cannot open %s (errno %d)", path, errno);
        return ESP_FAIL;
    }

    size_t written = fwrite(data, 1, length, f);
    fclose(f);

    if (written != length)
    {
        ESP_LOGE(TAG, "Wrote %zu/%zu bytes", written, length);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Wrote %zu bytes -> %s", written, path);
    return ESP_OK;
}

esp_err_t sd_read(const char *filename, uint8_t *out_buf,
                  size_t buf_size, size_t *bytes_read)
{
    char path[SD_MAX_PATH_LEN];
    snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, filename);

    FILE *f = fopen(path, "rb");
    if (!f)
    {
        ESP_LOGE(TAG, "Cannot open %s (errno %d)", path, errno);
        return ESP_FAIL;
    }

    *bytes_read = fread(out_buf, 1, buf_size, f);
    fclose(f);

    ESP_LOGI(TAG, "Read %zu bytes <- %s", *bytes_read, path);
    return ESP_OK;
}

esp_err_t sd_wipe_files(void)
{
    DIR *dir = opendir(SD_MOUNT_POINT);
    if (!dir)
    {
        ESP_LOGE(TAG, "Failed to open directory (errno %d)", errno);
        return ESP_FAIL;
    }

    struct dirent *entry;
    int failed = 0;

    while ((entry = readdir(dir)) != NULL)
    {
        // Skip directories (e.g. "." and "..")
        if (entry->d_type == DT_DIR)
            continue;

        char path[SD_MAX_PATH_LEN];
        //snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, entry->d_name);

        if (unlink(path) != 0)
        {
            ESP_LOGE(TAG, "Failed to delete %s (errno %d)", path, errno);
            failed++;
        }
        else
        {
            ESP_LOGI(TAG, "Deleted %s", path);
        }
    }

    closedir(dir);

    if (failed > 0)
    {
        ESP_LOGW(TAG, "%d file(s) could not be deleted", failed);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "All files deleted");
    return ESP_OK;
}
