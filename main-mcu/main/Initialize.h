#pragma once
#include "led_strip.h"


void init_gpio_pins();
esp_err_t init_spi();
esp_err_t init_i2c();
void init_uart();
void init_sensors();
void init_neopixel();

extern led_strip_handle_t s_strip;