#include <string.h> //String and memory functions
#include <math.h> //Mathematical constants and functions
#include "freertos/FreeRTOS.h" //FreeRTOS functionality
#include "freertos/task.h" //FreeRTOS functionality
#include "driver/gpio.h" //GPIO driver functions
#include "driver/uart.h" //UART communication driver
#include "esp_log.h"
#include "Settings.h" //Pin definitions and hardware configuration
#include "read_sensors.h" //Data storage
#include "uart.h" //Initialization/configuration functions

static const char *TAG = "K96_SENSOR";

//Turn sensor on
void K96_on()
{
    gpio_set_level(K96_EN_PIN, 1);
}

//Turn sensor off
void K96_off()
{
    gpio_set_level(K96_EN_PIN, 0);
}


// ----- Compute Modbus CRC checksum for error checking communication integrity ----
static uint16_t modbus_crc16(
    const uint8_t *data,
    uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/**
 * Read data from the K96 RAM using the MODBUS protocol.
 */
static bool K96_read_ram(
    uint16_t ram_address, // Address to read
    uint8_t num_bytes, // Number of bytes requested
    uint8_t *response) // Where the reply is stored
{
    uint8_t frame[7];

    frame[0] = 0x68; // Device address
    frame[1] = 0x44; // Function code (read RAM)
    frame[2] = (ram_address >> 8) & 0xFF; // High byte
    frame[3] = ram_address & 0xFF; // Low byte
    frame[4] = num_bytes;

    uint16_t crc = modbus_crc16(frame, 5);
    frame[5] = crc & 0xFF;
    frame[6] = (crc >> 8) & 0xFF;

    // Completely clear out any stale or leftover data in the UART buffers first
    uart_flush_input(UART_PORT);

    // Send MODBUS frame
    uart_write_bytes(UART_PORT, (const char *)frame, sizeof(frame));
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(10));

    // Read expected response bytes: header (3 bytes) + payload (num_bytes) + CRC (2 bytes)
    uint16_t expected_len = num_bytes + 5;
    uint16_t len = uart_read_bytes(
        UART_PORT,
        response,
        expected_len,
        pdMS_TO_TICKS(100)); // Reduced timeout for faster recovery if a frame drops

    // Return false if expected number of bytes were not received
    if (len != expected_len)
    {
        return false;
    }

    if (response[1] == (0x44 | 0x80))
    {
        return false;
    }

    if (response[0] != 0x68 ||
        response[1] != 0x44 ||
        response[2] != num_bytes)
    {
        return false;
    }

    uint16_t received_crc = response[len-2] | (response[len-1] << 8);
    uint16_t calculated_crc = modbus_crc16(response, len-2);

    if (received_crc != calculated_crc)
    {
        return false;
    }

    return true;
}

// Describes the exact format of the K96 data
typedef enum
{
    K96_U16,
    K96_S16,
    K96_B16,
    K96_S16_8,
    K96_S32,
    K96_S32_16
} K96_DataType;

typedef struct
{
    uint16_t address;
    const char *name;
    const char *unit;
    K96_DataType type;
} K96_RAM_Item;

static const K96_RAM_Item ram_items[] =
{
    {0x0180, "LPL_Signal",             "counts", K96_S32},
    {0x0184, "LPL_Signal_filtered",    "counts", K96_S32_16},
    {0x0190, "SPL_Signal",             "counts", K96_S32},
    {0x0194, "SPL_Signal_filtered",    "counts", K96_S32_16},

    {0x01B0, "ADuCdie_Temp",           "°C",     K96_S16_8},
    {0x01B4, "ADuCdie_Temp_filtered",  "°C",     K96_S16_8},
    {0x01B8, "NTC0_Temp",              "°C",     K96_S16_8},
    {0x01BC, "NTC0_Temp_filtered",     "°C",     K96_S16_8},
    {0x01C0, "NTC1_Temp",              "°C",     K96_S16_8},
    {0x01C4, "NTC1_Temp_filtered",     "°C",     K96_S16_8},

    {0x01F0, "RH",                     "%RH",    K96_S16},
    {0x01F8, "RH_Temp",                "°C",     K96_S16},

    {0x0360, "MPL_Signal",             "counts", K96_S32},
    {0x0364, "MPL_Signal_filtered",    "counts", K96_S32_16},
    {0x0384, "MPL_uflt_IR_Signal",     "counts", K96_U16},

    {0x038E, "MPL_uflt_Error",         "-",      K96_B16},
    {0x03A4, "MPL_flt_IR_Signal",      "counts", K96_U16},
    {0x038A, "MPL_uflt_Conc",          "ppm",    K96_S16},
    {0x03AA, "MPL_flt_Conc",           "ppm",    K96_S16},

    {0x0424, "LPL_uflt_IR_Signal",     "counts", K96_U16},
    {0x042A, "LPL_uflt_Conc",          "ppm",    K96_S16},
    {0x042E, "LPL_uflt_Error",         "-",      K96_B16},
    {0x0444, "LPL_flt_IR_Signal",      "counts", K96_U16},

    {0x044E, "LPL_flt_Error",          "-",      K96_B16},
    {0x0484, "SPL_uflt_IR_Signal",     "counts", K96_U16},
    {0x048A, "SPL_uflt_Conc",          "ppm",    K96_S16},
    {0x048E, "SPL_uflt_Error",         "-",      K96_B16},
    {0x04A4, "SPL_flt_IR_Signal",      "counts", K96_U16},
    {0x04AE, "SPL_flt_Error",          "-",      K96_B16},
};

// ----- Check and log K96 hardware errors -----
void check_k96_errors(void)
{
    uint8_t response[32];
    
    if (!K96_read_ram(0x001C, 2, response))
    {
        ESP_LOGE(TAG, "Failed to read ErrorStatus register (0x001C)");
        return;
    }

    uint16_t error_status = ((uint16_t)response[3] << 8) | response[4];
    sensor_data.K96_error = error_status; // Keep structure updated

    if (error_status == 0)
    {
        return;
    }

    ESP_LOGE(TAG, "Sensor reported errors! ErrorStatus: 0x%04X", error_status);

    if (error_status & (1 << 15)) ESP_LOGE(TAG, "- LPL_Calc_Conc_error");
    if (error_status & (1 << 14)) ESP_LOGE(TAG, "- SPL_Calc_Conc_error");
    if (error_status & (1 << 13)) ESP_LOGE(TAG, "- MPL_Calc_Conc_error");
    if (error_status & (1 << 11)) ESP_LOGE(TAG, "- ADuCdie_Temp_error");
    if (error_status & (1 << 10)) ESP_LOGE(TAG, "- ADuC_NTCs_Temp_error");
    if (error_status & (1 << 9))  ESP_LOGE(TAG, "- SPI error");
    if (error_status & (1 << 8))  ESP_LOGE(TAG, "- Logger_error");
    if (error_status & (1 << 7))  ESP_LOGW(TAG, "- Warm Up"); 
    if (error_status & (1 << 6))  ESP_LOGE(TAG, "- Memory Error");
    if (error_status & (1 << 4))  ESP_LOGE(TAG, "- SelfDiag error");
    if (error_status & (1 << 3))  ESP_LOGE(TAG, "- Calibration calculation error");
    if (error_status & (1 << 2))  ESP_LOGE(TAG, "- Configuration error");
    if (error_status & (1 << 0))  ESP_LOGE(TAG, "- Fatal error");
}

//----- Read and update all sensor data in the global struct -----
void read_k96(void)
{
    check_k96_errors(); // Check for any hardware errors first
    uint8_t response[32];
    size_t num_items = sizeof(ram_items) / sizeof(ram_items[0]);

    for (size_t i = 0; i < num_items; i++)
    {
        const K96_RAM_Item *item = &ram_items[i];
        uint8_t bytes = 0;

        switch(item->type)
        {
            case K96_S16_8:  bytes = 3; break;
            case K96_S32_16: bytes = 6; break;
            case K96_S32:    bytes = 4; break;
            case K96_U16:
            case K96_S16:
            case K96_B16:
            default:         bytes = 2; break;
        }

        // Read data from K96 
        memset(response, 0, sizeof(response));
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to avoid overwhelming the sensor with requests
        if (!K96_read_ram(item->address, bytes, response))
        {
            ESP_LOGW(TAG, "Failed to read K96 RAM address 0x%04X (%s)", item->address, item->name);
        }

        double   val_f = 0.0;
        int32_t  val_i = 0;
        uint16_t val_u = 0;

        switch(item->type)
        {
            case K96_U16:
            case K96_B16:
                val_u = ((uint16_t)response[3] << 8) | response[4];
                break;

            case K96_S16:
            {
                int16_t raw = ((int16_t)response[3] << 8) | response[4];
                if (item->address == 0x01F0 || item->address == 0x01F8) {
                    val_f = raw * 0.01f;
                } else {
                    val_f = (float)raw;
                }
                break;
            }

            case K96_S16_8:
            {
                int32_t raw = ((int32_t)response[3] << 16) | ((int32_t)response[4] << 8) | response[5];
                if (raw & 0x800000) raw |= 0xFF000000;
                val_f = (raw / 256.0f) * 0.01f;
                break;
            }

            case K96_S32:
            {
                val_i = ((int32_t)response[3] << 24) | ((int32_t)response[4] << 16) | 
                        ((int32_t)response[5] << 8) | response[6];
                break;
            }

            case K96_S32_16:
            {
                int32_t int_part = ((int32_t)response[3] << 24) | ((int32_t)response[4] << 16) | 
                                   ((int32_t)response[5] << 8) | response[6];
                uint16_t frac_part = ((uint16_t)response[7] << 8) | response[8];
                val_f = int_part + (frac_part / 65536.0);
                break;
            }
            default:
                break;
        }

        switch(item->address)
        {
            case 0x0180: sensor_data.K96_LPL_Signal = val_i; break;
            case 0x0184: sensor_data.K96_LPL_Signal_filtered = val_f; break;
            case 0x0190: sensor_data.K96_SPL_Signal = val_i; break;
            case 0x0194: sensor_data.K96_SPL_Signal_filtered = val_f; break;

            case 0x01B0: sensor_data.K96_ADuCdie_Temp = val_f; break;
            case 0x01B4: sensor_data.K96_ADuCdie_Temp_filtered = val_f; break;
            case 0x01B8: sensor_data.K96_NTC0_Temp = val_f; break;
            case 0x01BC: sensor_data.K96_NTC0_Temp_filtered = val_f; break;
            case 0x01C0: sensor_data.K96_NTC1_Temp = val_f; break;
            case 0x01C4: sensor_data.K96_NTC1_Temp_filtered = val_f; break;

            case 0x01F0: sensor_data.K96_RH = val_f; break;
            case 0x01F8: sensor_data.K96_RH_Temp = val_f; break;

            case 0x0360: sensor_data.K96_MPL_Signal = val_i; break;
            case 0x0364: sensor_data.K96_MPL_Signal_filtered = val_f; break;
            case 0x0384: sensor_data.K96_MPL_uflt_IR_Signal = val_u; break;
            case 0x038A: sensor_data.K96_MPL_uflt_Conc = val_f; break;
            case 0x038E: sensor_data.K96_MPL_uflt_Error = val_u; break;
            case 0x03A4: sensor_data.K96_MPL_flt_IR_Signal = val_u; break;
            case 0x03AA: sensor_data.K96_MPL_flt_Conc = val_f; break;

            case 0x0424: sensor_data.K96_LPL_uflt_IR_Signal = val_u; break;
            case 0x042A: sensor_data.K96_LPL_uflt_Conc = val_f; break;
            case 0x042E: sensor_data.K96_LPL_uflt_Error = val_u; break;
            case 0x0444: sensor_data.K96_LPL_flt_IR_Signal = val_u; break;
            case 0x044E: sensor_data.K96_LPL_flt_Error = val_u; break;

            case 0x0484: sensor_data.K96_SPL_uflt_IR_Signal = val_u; break;
            case 0x048A: sensor_data.K96_SPL_uflt_Conc = val_f; break;
            case 0x048E: sensor_data.K96_SPL_uflt_Error = val_u; break;
            case 0x04A4: sensor_data.K96_SPL_flt_IR_Signal = val_u; break;
            case 0x04AE: sensor_data.K96_SPL_flt_Error = val_u; break;

            default:
                break;
        }
    }
}