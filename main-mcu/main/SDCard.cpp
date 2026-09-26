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
#include "ErrorStatus.h"
#include "Settings.h"
#include "read_sensors.h"
#include "SystemStatus.h"


#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

static const char *TAG = "SDCard";
// Buffer configuration
//#define SD_BUFFER_SIZE 4096
#define SENSOR_READING_SIZE sizeof(SensorData)
#define TOTAL_READING_SIZE sizeof(MainSystemStatusPacket)

#define READINGS_PER_BUFFER (SD_BUFFER_SIZE / TOTAL_READING_SIZE)

//#define READINGS_PER_BUFFER //(SD_BUFFER_SIZE / SENSOR_READING_SIZE)//


static uint8_t SD_buffer[SD_BUFFER_SIZE/2];
static uint8_t SD_buffer2[SD_BUFFER_SIZE/2];
static size_t SD_buffer_offset = 0;

static sdmmc_card_t *s_card = NULL;
static bool s_mounted = false;
static char current_csv_filename[56] = "";
static char current_metadata_filename[56] = "";

SensorData *sensor_datas;

static void create_timestamped_filename(const char *prefix,
                                       char *buffer,
                                       size_t buffer_size,
                                       SensorData *sensor_data,
                                       const char *extension)
{
    
    //time_t now = time(NULL);
    //struct tm timeinfo;
    //localtime_r(&now, &timeinfo);

    snprintf(buffer, buffer_size,
             "%s_%02u%02u%02u%s",
             prefix,
             sensor_data->hours,
             sensor_data->minutes,
             sensor_data->seconds,
             extension);
}

static void create_unique_csv_filename(void)
{
    create_timestamped_filename("sensor_data", current_csv_filename, sizeof(current_csv_filename), &sensor_data, ".csv");
}
static void create_unique_metadata_filename(void)
{
    create_timestamped_filename("metadata", current_metadata_filename, sizeof(current_metadata_filename), &sensor_data, ".log");
}
/*
//
static esp_err_t create_new_csv_file(void)
{
    if (current_csv_filename[0] == '\0')
    {
        create_unique_csv_filename();
    }

    char path[SD_MAX_PATH_LEN];
    snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, current_csv_filename);

    FILE *f = fopen(path, "wb");
    if (!f)
    {
        ESP_LOGE_CAPTURED(ERROR_BIT_29, TAG, "Cannot create %s (errno %d)", path, errno);
        return ESP_FAIL;
    }
    fclose(f);
    ESP_LOGI(TAG, "Created new CSV file: %s", current_csv_filename);
    return ESP_OK;
}
//
*/

static esp_err_t create_new_csv_file(void)
{
    if (current_csv_filename[0] == '\0')
    {
        create_unique_csv_filename();
    }

    char path[SD_MAX_PATH_LEN];
    snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, current_csv_filename);

    FILE *f = fopen(path, "wb");
    if (!f)
    {
        ESP_LOGE_CAPTURED(ERROR_BIT_29, TAG, "Cannot create %s (errno %d)", path, errno);
        return ESP_FAIL;
    }
    fclose(f);
    
    //write header
    const char *header = "HH,MM,SS,TP1, TP2, TP3, TP6, PP3, TP4, PP1, PA1, TA1, TA2, TA3, HA1, TP5, PP2, TT1, TT2, TT3, K96_LPL, K96_LPL_flt, K96_SPL, K96_SPL_flt, K96_MPL, K96_MPL_flt, K96_ADuCdie_Temp, K96_ADuCdie_Temp_filtered, K96_NTC0_Temp, K96_NTC0_Temp_filtered, K96_NTC1_Temp, K96_NTC1_Temp_filtered, K96_RH, K96_RH_Temp, K96_MPL_uflt_IR_Signal, K96_MPL_flt_IR_Signal, K96_MPL_uflt_Conc, K96_MPL_flt_Conc, K96_MPL_uflt_Error, K96_LPL_uflt_IR_Signal, K96_LPL_flt_IR_Signal, K96_LPL_uflt_Conc, K96_LPL_uflt_Error, K96_LPL_flt_Error, K96_SPL_uflt_IR_Signal, K96_SPL_flt_IR_Signal, K96_SPL_uflt_Conc, K96_SPL_uflt_Error,K96_SPL_flt_Error, K96_error\n";
    if (sd_write(current_csv_filename, (const uint8_t *)header, strlen(header)) != ESP_OK)
    {
        ESP_LOGW(TAG, "Sensor csv header write failed for %s", current_metadata_filename);
    }

    ESP_LOGI(TAG, "Created new CSV log: %s", current_csv_filename);
    return ESP_OK;
}

static esp_err_t create_new_metadata_log(void)
{
    if (current_metadata_filename[0] == '\0')
    {
        create_unique_metadata_filename();
    }

    char path[SD_MAX_PATH_LEN];
    snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, current_metadata_filename);

    FILE *f = fopen(path, "wb");
    if (!f)
    {
        ESP_LOGE_CAPTURED(ERROR_BIT_29, TAG, "Cannot create metadata log %s (errno %d)", path, errno);
        return ESP_FAIL;
    }
    fclose(f);

    const char *header = "HH,MM,SS,event,mode,flight_phase,heater_mask,pump_value,secondary_value,tertiary_value\n";
    if (sd_write(current_metadata_filename, (const uint8_t *)header, strlen(header)) != ESP_OK)
    {
        ESP_LOGW(TAG, "Metadata log header write failed for %s", current_metadata_filename);
    }

    ESP_LOGI(TAG, "Created metadata log: %s", current_metadata_filename);
    return ESP_OK;
}


// ===================================================================
// Option 1: Binary buffer large (fastest, smallest)
// ===================================================================
void buffer_SD_data_binary(const SensorData *sensor_data)
{
    // Check if there's space for another reading
    if (SD_buffer_offset + TOTAL_READING_SIZE <= SD_BUFFER_SIZE)
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
            ESP_LOGE_CAPTURED(ERROR_BIT_24, TAG, "Failed to write buffer to SD");
        }
        SD_buffer_offset = 0;  // Reset for next batch
    }
}

void buffer_SD_data_csv(MainSystemStatusPacket *system_status_packet)//SensorData *sensor_data)
{
    *sensor_datas = system_status_packet->sensor_data;
    if (sensor_datas == NULL) return;

    // Create temp CSV line to store (increased size to 1024 to fit all expanded sensor fields)
    char line[1024];
    char sline[1024];

    int n = snprintf(line, sizeof(line),
        "%02u,%02u,%02u,"
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
        "%.2f,%.2f,%.2f,"
        "%ld,%.4f,%ld,%.4f,"
        "%ld,%.4f,%.2f,%.2f,"
        "%.2f,%.2f,%.2f,%.2f,"
        "%.2f,%.2f,%u,%u,"
        "%.2f,%.2f,%u,"
        "%u,%u,%.2f,"
        "%u,%u,%u,"
        "%u,%.2f,%u,%u,%u\n",
        sensor_datas->hours,
        sensor_datas->minutes,
        sensor_datas->seconds,
        sensor_datas->Tp1, sensor_datas->Tp2, sensor_datas->Tp3, sensor_datas->Tp6, 
        sensor_datas->Pp3, sensor_datas->Tp4, sensor_datas->Pp1, sensor_datas->Pa1, 
        sensor_datas->Ta1, sensor_datas->Ta2, sensor_datas->Ta3, sensor_datas->Ha1, 
        sensor_datas->Tp5, sensor_datas->Pp2,
        sensor_datas->Tt1, sensor_datas->Tt2, sensor_datas->Tt3,
        // K96 fields mapping
        (long)sensor_datas->K96_LPL_Signal, sensor_datas->K96_LPL_Signal_filtered,
        (long)sensor_datas->K96_SPL_Signal, sensor_datas->K96_SPL_Signal_filtered,
        (long)sensor_datas->K96_MPL_Signal, sensor_datas->K96_MPL_Signal_filtered,
        sensor_datas->K96_ADuCdie_Temp, sensor_datas->K96_ADuCdie_Temp_filtered,
        sensor_datas->K96_NTC0_Temp, sensor_datas->K96_NTC0_Temp_filtered,
        sensor_datas->K96_NTC1_Temp, sensor_datas->K96_NTC1_Temp_filtered,
        sensor_datas->K96_RH, sensor_datas->K96_RH_Temp,
        sensor_datas->K96_MPL_uflt_IR_Signal, sensor_datas->K96_MPL_flt_IR_Signal,
        sensor_datas->K96_MPL_uflt_Conc, sensor_datas->K96_MPL_flt_Conc, sensor_datas->K96_MPL_uflt_Error,
        sensor_datas->K96_LPL_uflt_IR_Signal, sensor_datas->K96_LPL_flt_IR_Signal, sensor_datas->K96_LPL_uflt_Conc,
        sensor_datas->K96_LPL_uflt_Error, sensor_datas->K96_LPL_flt_Error,
        sensor_datas->K96_SPL_uflt_IR_Signal, sensor_datas->K96_SPL_flt_IR_Signal, sensor_datas->K96_SPL_uflt_Conc,
        sensor_datas->K96_SPL_uflt_Error, sensor_datas->K96_SPL_flt_Error, sensor_datas->K96_error
    );

    //CapturedErrors *cerr = system_status_packet->captured_errors; ---> could be nice to also save 
    int m = snprintf(sline, sizeof(sline),
        "%02u,%02u,%02u,%02u,%02u,%02u,%02u,%02u,%02u,%02u,%02u,%02u,%02u,%02u,%02u,%02u,%02u,%02u,%02u,%02u,%02u\n",
        system_status_packet->operating_mode,
        system_status_packet->command_received,
        system_status_packet->connection_lost,
        system_status_packet->status_ok,
        system_status_packet->pressure_system_on,
        system_status_packet->k96_on,
        system_status_packet->heater_mask,
        system_status_packet->thermal_online,
        system_status_packet->thermal_state,
        system_status_packet->thermal_error,
        system_status_packet->pressure_state,
        system_status_packet->pressure_error,
        system_status_packet->pressure_relay_mask,
        system_status_packet->pressure_pump1_pwm,
        system_status_packet->pressure_pump2_pwm,
        system_status_packet->pressure_compressor_pwm,
        system_status_packet->pressure_manual_override,
        system_status_packet->pressure_valve_open,
        system_status_packet->onboard_logging,
        system_status_packet->storage_free_pct,
        system_status_packet->controller_state
        //cerr.high
        );

    // Check if snprintf encountered an error or truncation
    if (n < 0 || (size_t)n >= sizeof(line))
    {
        ESP_LOGE_CAPTURED(ERROR_BIT_25, TAG, "sensor CSV line formatting failed or was truncated!");
        return;
    }
    if (m < 0 || (size_t)n >= sizeof(sline))
    {
        ESP_LOGE_CAPTURED(ERROR_BIT_25, TAG, "Meta data CSV line formatting failed or was truncated!");
        return;
    }

    // If the line doesn't fit, flush current buffer first
    // Extra check since csv can be variable length and might exceed buffer size on its own,
    // in that case we should write it directly instead of trying to buffer it
    if ((size_t)n + (size_t)m + SD_buffer_offset >= SD_BUFFER_SIZE)
    {
        esp_err_t err = sd_write(current_metadata_filename, SD_buffer2, SD_buffer_offset);
        esp_err_t err2 = sd_write(current_csv_filename, SD_buffer, SD_buffer_offset);
        if ((err == ESP_OK) and (err2 == ESP_OK))
        {
            ESP_LOGI(TAG, "Flushed %zu bytes CSV to SD", SD_buffer_offset);
        }
        else
        {
            ESP_LOGE_CAPTURED(ERROR_BIT_26, TAG, "Failed to flush CSV buffer to SD");
        }
        SD_buffer_offset = 0;
    }

    // Append the line bytes into the buffer
    memcpy(&SD_buffer[(int)SD_buffer_offset/2], line, (size_t)n);
    memcpy(&SD_buffer2[(int)SD_buffer_offset/2], sline, (size_t)m);
    SD_buffer_offset += (size_t)n + (size_t)m;

    // If buffer full after append, write it out
    if (SD_buffer_offset >= SD_BUFFER_SIZE)
    {
        //esp_err_t err = sd_write("sensor_data.csv", SD_buffer, SD_BUFFER_SIZE);
        esp_err_t err = sd_write(current_metadata_filename, SD_buffer2, SD_buffer_offset);
        esp_err_t err2 = sd_write(current_csv_filename, SD_buffer, SD_buffer_offset);
        if (err == ESP_OK and err2 == ESP_OK)
        {
            ESP_LOGI(TAG, "Wrote %zu bytes CSV to SD", SD_BUFFER_SIZE);
        }
        else
        {
            ESP_LOGE_CAPTURED(ERROR_BIT_27, TAG, "Failed to write CSV buffer to SD");
        }
        SD_buffer_offset = 0;  // Reset for next batch
    }
}
/*
void log_metadata(MetaData *meta_data)
{
    if (event_type == nullptr || current_metadata_filename[0] == '\0')
    {
        return;
    }

    time_t now = time(NULL);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    char line[256];
    int n = snprintf(line, sizeof(line),
                     "%02d,%02d,%02d,%s,%d,%d,0x%02X,%d,%d,%d\n",
                     timeinfo.tm_hour,
                     timeinfo.tm_min,
                     timeinfo.tm_sec,
                     event_type,
                     mode,
                     flight_phase,
                     heater_mask,
                     pump_value,
                     secondary_value,
                     tertiary_value);

    if (n < 0 || (size_t)n >= sizeof(line))
    {
        ESP_LOGE_CAPTURED(ERROR_BIT_25, TAG, "Metadata log line formatting failed or was truncated");
        return;
    }

    esp_err_t err = sd_write(current_metadata_filename, (const uint8_t *)line, (size_t)n);
    if (err != ESP_OK)
    {
        ESP_LOGE_CAPTURED(ERROR_BIT_26, TAG, "Failed to log metadata event %s", event_type);
    }
}

*/


// Flush remaining data (call before shutdown)
void buffer_SD_data_flush()
{
    const char *csv_filename = current_csv_filename[0] != '\0' ? current_csv_filename : "sensor_data.csv";
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
        ESP_LOGE_CAPTURED(ERROR_BIT_28, TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return ret;
    }

    s_mounted = true;
    create_unique_csv_filename();
    create_unique_metadata_filename();
    esp_err_t csv_err = create_new_csv_file();
    if (csv_err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to create initial CSV log file; continuing with mount");
    }

    esp_err_t metadata_err = create_new_metadata_log();
    if (metadata_err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to create metadata log file; continuing with mount");
    }

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

bool sd_is_mounted(void)
{
    return s_mounted;
}

bool sd_get_free_percent(uint8_t *free_percent)
{
    if (!s_mounted || free_percent == NULL)
    {
        return false;
    }

    uint64_t total_bytes = 0;
    uint64_t free_bytes = 0;
    esp_err_t err = esp_vfs_fat_info(SD_MOUNT_POINT, &total_bytes, &free_bytes);
    if (err != ESP_OK || total_bytes == 0)
    {
        ESP_LOGW(TAG, "Failed to read SD capacity: %s", esp_err_to_name(err));
        return false;
    }

    const uint64_t percentage = (free_bytes * 100U + total_bytes / 2U) / total_bytes;
    *free_percent = static_cast<uint8_t>(percentage > 100U ? 100U : percentage);
    return true;
}

esp_err_t sd_write(const char *filename, const uint8_t *data, size_t length)
{
    
    if (filename[0] == '\0')
    {
        create_new_csv_file();
    }

    char path[SD_MAX_PATH_LEN];
    snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, filename);

    // "ab" = append
    FILE *f = fopen(path, "ab");
    if (!f)
    {
        ESP_LOGE_CAPTURED(ERROR_BIT_29, TAG, "Cannot open %s (errno %d)", path, errno);
        return ESP_FAIL;
    }

    size_t written = fwrite(data, 1, length, f);
    fclose(f);

    if (written != length)
    {
        ESP_LOGE_CAPTURED(ERROR_BIT_30, TAG, "Wrote %zu/%zu bytes", written, length);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Wrote %zu bytes -> %s", written, path);
    return ESP_OK;
}

esp_err_t sd_read(const char *filename, uint8_t *out_buf,
                  size_t buf_size, size_t *bytes_read)
{
    if (filename[0] == '\0')
    {
        create_new_csv_file();
    }

    char path[SD_MAX_PATH_LEN];
    snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, filename);

    FILE *f = fopen(path, "rb");
    if (!f)
    {
        ESP_LOGE_CAPTURED(ERROR_BIT_31, TAG, "Cannot open %s (errno %d)", path, errno);
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
        ESP_LOGE_CAPTURED(ERROR_BIT_32, TAG, "Failed to open directory (errno %d)", errno);
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
            ESP_LOGE_CAPTURED(ERROR_BIT_33, TAG, "Failed to delete %s (errno %d)", path, errno);
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
