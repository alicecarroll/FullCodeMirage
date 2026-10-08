#pragma once

#include <stdint.h>

using TickType_t = uint32_t;
constexpr uint32_t portTICK_PERIOD_MS = 10;
#define pdMS_TO_TICKS(ms) (static_cast<TickType_t>((ms) / portTICK_PERIOD_MS))
TickType_t xTaskGetTickCount();
