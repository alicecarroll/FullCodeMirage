#pragma once

#include <stdint.h>

#include "esp_log.h"

// One bit is reserved for each active ESP_LOGE call in main-mcu/main.
// Keep these values stable: they are part of the telemetry wire protocol.
enum ErrorBit : uint8_t {
#define ERROR_BIT(bit, message) ERROR_BIT_##bit = bit,
#include "ErrorBits.def"
#undef ERROR_BIT
};

// Compatibility aliases for the original zero-padded names used at call sites.
#define ERROR_BIT_00 ERROR_BIT_0
#define ERROR_BIT_01 ERROR_BIT_1
#define ERROR_BIT_02 ERROR_BIT_2
#define ERROR_BIT_03 ERROR_BIT_3
#define ERROR_BIT_04 ERROR_BIT_4
#define ERROR_BIT_05 ERROR_BIT_5
#define ERROR_BIT_06 ERROR_BIT_6
#define ERROR_BIT_07 ERROR_BIT_7
#define ERROR_BIT_08 ERROR_BIT_8
#define ERROR_BIT_09 ERROR_BIT_9

extern uint64_t captured_errors;

#define ESP_LOGE_CAPTURED(bit, tag, format, ...) do { \
    captured_errors |= (UINT64_C(1) << (bit)); \
    ESP_LOGE(tag, format, ##__VA_ARGS__); \
} while (0)
