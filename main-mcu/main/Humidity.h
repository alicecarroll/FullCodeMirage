// Humidity.h
#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Compares the current humidity against HUMIDITY_TARGET
     *        and updates the internal humid flag accordingly.
     *
     * @param[in] humidity Pointer to the current humidity reading.
     *
     * @return ESP_OK on success, or an esp_err_t error code.
     */
    esp_err_t compare_humidity(int *humidity);

    /**
     * @brief Checks whether humidity has been restored to an acceptable level
     *        and that the temperature requirement has been met before
     *        exiting the humidity setting mode.
     *
     * @param[in]  temp             Pointer to the current temperature reading.
     * @param[in]  humidity         Pointer to the current humidity reading.
     * @param[out] humidity_setting Pointer to the humidity setting flag,
     *                              cleared to false when conditions are met.
     *
     * @return ESP_OK on success, or an esp_err_t error code.
     */
    esp_err_t check_humidity_restored(int *temp, int *humidity, bool *humidity_setting);

    /**
     * @brief Evaluates humidity and temperature conditions to determine
     *        whether to enter, remain in, or exit humidity setting mode.
     *
     * @param[in]  temp             Pointer to the current temperature reading.
     * @param[in]  humidity         Pointer to the current humidity reading.
     * @param[out] humidity_setting Pointer to the humidity setting flag,
     *                              set to true when humidity is too high and
     *                              cleared to false when conditions are restored.
     *
     * @return ESP_OK on success, or an esp_err_t error code.
     */
    esp_err_t check_humidity_levels(int *temp, int *humidity, bool *humidity_setting);

#ifdef __cplusplus
}
#endif
