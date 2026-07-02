#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include <dirent.h>
#include "read_sensors.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief  Write a binary buffer to a file on the SD card.
     *         Data is appended if the file already exists.
     *
     * @param[in] filename  File name only (e.g. "data.bin"), not the full path.
     * @param[in] data      Pointer to the buffer to write.
     * @param[in] length    Number of bytes to write.
     *
     * @return ESP_OK on success, or an esp_err_t error code.
     */
    esp_err_t sd_write(const char *filename, const uint8_t *data, size_t length);

    /**
     * @brief  Read a binary file from the SD card into a caller-supplied buffer.
     *
     * @param[in]  filename    File name only (e.g. "data.bin"), not the full path.
     * @param[out] out_buf     Buffer to receive the data.
     * @param[in]  buf_size    Size of out_buf in bytes.
     * @param[out] bytes_read  Actual number of bytes read.
     *
     * @return ESP_OK on success, or an esp_err_t error code.
     */
    esp_err_t sd_read(const char *filename, uint8_t *out_buf,
                      size_t buf_size, size_t *bytes_read);

    esp_err_t sd_mount(void);
    void sd_unmount(void);
    void buffer_SD_data_csv(const SensorData *sensor_data);
    void buffer_SD_data_flush(void);

    /**
     * @brief Deleate all files form the SD card. Keep formating and dirs.
     *
     * @return ESP_OK on success, or an esp_err_t error code.
     */
    esp_err_t sd_wipe_files(void);

#ifdef __cplusplus
}
#endif
