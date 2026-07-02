#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "esp_err.h"

/* ── Pin & pixel count ────────────────────────────────────────────────────── */
#define NEOPIXEL_DATA_GPIO 6 /* IO6 — change to match your wiring     */
#define NEOPIXEL_COUNT 3     /* Number of pixels on the strip          */
    /* ── Per-pixel control ────────────────────────────────────────────────────── */

    /**
     * @brief  Set one pixel to an RGB colour and push to hardware.
     * @param  index   Pixel index (0 … NEOPIXEL_COUNT-1)
     * @param  red     0–255
     * @param  green   0–255
     * @param  blue    0–255
     * @return ESP_OK, ESP_ERR_INVALID_ARG if index is out of range.
     */
    esp_err_t neopixel_set_pixel(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);

    /**
     * @brief  Turn one pixel off (black) and push to hardware.
     * @param  index   Pixel index (0 … NEOPIXEL_COUNT-1)
     */
    esp_err_t neopixel_clear_pixel(uint8_t index);

    /**
     * @brief  Turn all pixels off.
     */
    esp_err_t neopixel_clear_all(void);

    /* ── Pre-programmed states ────────────────────────────────────────────────── */

    /**
     * @brief  All pixels solid green — nominal / connected.
     */
    esp_err_t neopixel_status_nominal(void);

    /**
     * @brief  All pixels amber — connection lost, 30-min timer running.
     */
    esp_err_t neopixel_status_connection_lost(void);

    /**
     * @brief  All pixels red — experiment terminated.
     */
    esp_err_t neopixel_status_terminated(void);

#ifdef __cplusplus
}
#endif