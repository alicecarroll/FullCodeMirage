#pragma once

#include <stdint.h>

typedef uint32_t TickType_t;

TickType_t xTaskGetTickCount();

#define portTICK_PERIOD_MS 1U
