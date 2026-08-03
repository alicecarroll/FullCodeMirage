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

struct CapturedErrors {
    uint64_t low;
    uint64_t high;
};

enum DecodedError : uint8_t {
    ERROR_OTHER = 0,
    K96_ERROR_STATUS_READ, K96_REPORTED, K96_LPL_CONCENTRATION,
    K96_SPL_CONCENTRATION, K96_MPL_CONCENTRATION, K96_ADUC_TEMPERATURE,
    K96_NTC_TEMPERATURE, K96_SPI, K96_LOGGER, K96_MEMORY,
    K96_SELF_DIAGNOSTICS, K96_CALIBRATION, K96_CONFIGURATION, K96_FATAL,
    TT1, TT2, TT3, TP1, TP2, TP3, TP4, TP5, TP6,
    PP1, PP2, PP3, TA1, TA2, TA3, HA1, PA1, RTC_READ,
};

static inline DecodedError decode_captured_error(uint8_t bit)
{
    switch (bit) {
        case ERROR_BIT_36: return K96_ERROR_STATUS_READ;
        case ERROR_BIT_37: return K96_REPORTED;
        case ERROR_BIT_38: return K96_LPL_CONCENTRATION;
        case ERROR_BIT_39: return K96_SPL_CONCENTRATION;
        case ERROR_BIT_40: return K96_MPL_CONCENTRATION;
        case ERROR_BIT_41: return K96_ADUC_TEMPERATURE;
        case ERROR_BIT_42: return K96_NTC_TEMPERATURE;
        case ERROR_BIT_43: return K96_SPI;
        case ERROR_BIT_44: return K96_LOGGER;
        case ERROR_BIT_45: return K96_MEMORY;
        case ERROR_BIT_46: return K96_SELF_DIAGNOSTICS;
        case ERROR_BIT_47: return K96_CALIBRATION;
        case ERROR_BIT_48: return K96_CONFIGURATION;
        case ERROR_BIT_49: return K96_FATAL;
        case ERROR_BIT_57: return TT1;
        case ERROR_BIT_58: return TT2;
        case ERROR_BIT_59: return TT3;
        case ERROR_BIT_60: return TP1;
        case ERROR_BIT_61: return TP2;
        case ERROR_BIT_62: return TP3;
        case ERROR_BIT_63: return TP4;
        case ERROR_BIT_64: return TP5;
        case ERROR_BIT_65: return TP6;
        case ERROR_BIT_66: return PP1;
        case ERROR_BIT_67: return PP2;
        case ERROR_BIT_68: return PP3;
        case ERROR_BIT_69: return TA1;
        case ERROR_BIT_70: return TA2;
        case ERROR_BIT_71: return TA3;
        case ERROR_BIT_72: return HA1;
        case ERROR_BIT_73: return PA1;
        case ERROR_BIT_74: return RTC_READ;
        default: return ERROR_OTHER;
    }
}

extern CapturedErrors captured_errors;

static inline bool captured_error_is_set(const CapturedErrors &errors, uint8_t bit)
{
    if (bit < 64) return (errors.low & (UINT64_C(1) << bit)) != 0;
    if (bit < 128) return (errors.high & (UINT64_C(1) << (bit - 64))) != 0;
    return false;
}

static inline void capture_error_bit(ErrorBit bit)
{
    if (bit < 64) {
        captured_errors.low |= (UINT64_C(1) << bit);
    } else {
        captured_errors.high |= (UINT64_C(1) << (bit - 64));
    }
}

#define ESP_LOGE_CAPTURED(bit, tag, format, ...) do { \
    capture_error_bit(bit); \
    ESP_LOGE(tag, format, ##__VA_ARGS__); \
} while (0)
