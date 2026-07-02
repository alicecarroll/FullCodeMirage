/* Neopixel values from 0 to 255
 * r: red:   0-255
 * g: green: 0-255
 * b: blue:  0-255
 */

#include "Neopixel.h"
#include "Settings.h"
#include "Initialize.h"
#include "driver/ledc.h"
#include "led_strip.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "Neopixel";
extern led_strip_handle_t s_strip;
/* ── Helpers ──────────────────────────────────────────────────────────────── */

/** Push the current pixel buffer to the hardware. */
static esp_err_t refresh(void)
{
    if (s_strip == NULL)
    {
        ESP_LOGE(TAG, "Can't find a handle. Call neopixel_init()");
        return ESP_ERR_INVALID_STATE;
    }

    return led_strip_refresh(s_strip);
}

/** Set all pixels to the same colour and push. */
static esp_err_t fill_all(uint8_t r, uint8_t g, uint8_t b)
{
    if (s_strip == NULL)
    {
        ESP_LOGE(TAG, "Can't find a handle. Call neopixel_init()");
        return ESP_ERR_INVALID_STATE;
    }

    for (uint8_t i = 0; i < NEOPIXEL_COUNT; i++)
    {
        esp_err_t err = led_strip_set_pixel(s_strip, i, r, g, b);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "led_strip_set_pixel(%d) failed: %s", i, esp_err_to_name(err));
            return err;
        }
    }
    return refresh();
}

/* ── Pixel control ────────────────────────────────────────────────────── */

esp_err_t neopixel_set_pixel(uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    if (s_strip == NULL)
    {
        ESP_LOGE(TAG, "Can't find a handle. Call neopixel_init()");
        return ESP_ERR_INVALID_STATE;
    }
    if (index >= NEOPIXEL_COUNT)
    {
        ESP_LOGE(TAG, "Pixel index %d out of range (max %d)", index, NEOPIXEL_COUNT - 1);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = led_strip_set_pixel(s_strip, index, red, green, blue);
    if (err != ESP_OK)
    {
        return err;
    }
    return refresh();
}

esp_err_t neopixel_clear_pixel(uint8_t index)
{
    return neopixel_set_pixel(index, 0, 0, 0);
}

/* ── Pre-programmed states ────────────────────────────────────────────────── */

esp_err_t neopixel_status_nominal(void)
{
    /* Solid green — all good, connected */
    return fill_all(0, 40, 0);
}

esp_err_t neopixel_status_connection_lost(void)
{
    /* Amber — connection lost, timer is counting */
    return fill_all(40, 20, 0);
}

esp_err_t neopixel_status_terminated(void)
{
    /* Red — 30-min timeout expired, experiment terminated */
    return fill_all(40, 0, 0);
}