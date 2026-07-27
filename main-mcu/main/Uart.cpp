#include <string.h> //String and memory functions
#include <math.h> //Mathematical constants and functions
#include "freertos/task.h" //FreeRTOS functionality
#include "driver/gpio.h" //GPIO driver functions
#include "driver/uart.h" //UART communication driver
#include "Settings.h" //Pin definitions and hardware configuration
#include "read_sensors.h" //Data storage
#include "uart.h" //Initialization/configuration functions


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

//data = Byte array to checksum
//length = Number of bytes
// uint16_t = 16-bit CRC value
static uint16_t modbus_crc16(
    const uint8_t *data,
    uint16_t length)
{
    // Initial  value
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        // Next input byte into the CRC calculation.
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            // If the least significant bit is set, apply the MODBUS polynomial.
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

    // Final CRC
    return crc;
}

/**
 * Read data from the K96 RAM using the MODBUS protocol.
 *
 * ram_address = RAM address to read.
 * num_bytes =   Number of data bytes requested.
 * response =    Buffer that receives the complete MODBUS response.
 *
 * @return true if the response is valid.
 * @return false if communication or CRC verification fails.
 */
static bool K96_read_ram(
    uint16_t ram_address, // Address to read
    uint8_t num_bytes, // Number of bytes requested
    uint8_t *response) // Where the reply is stored
{
    // Communication packet
    uint8_t frame[7];

    // Device address (K96 default is 0x68)
    frame[0] = 0x68;

    // Function code (0x44 for reading RAM)
    frame[1] = 0x44;

    // RAM address
    frame[2] =
        (ram_address >> 8) & 0xFF; //High bytes

    frame[3] =
        ram_address & 0xFF; //Low bytes

    // Number of bytes to read
    frame[4] = num_bytes;

    // CRC checksum stored as low bytes first
    uint16_t crc =
        modbus_crc16(frame, 5);

    frame[5] =
        crc & 0xFF;

    frame[6] =
        (crc >> 8) & 0xFF;

    // Clear old UART data
    uart_flush(UART_PORT);

    // Send MODBUS frame
    // Address, Function, RAM high, RAM low, Length, CRC low, CRC high
    uart_write_bytes(
        UART_PORT,
        (const char *)frame,
        sizeof(frame));
    int len =

    /*
    * MODBUS response format
    *
    * Byte 0 : Device address
    * Byte 1 : Function code
    * Byte 2 : Number of data bytes
    * Byte 3..N : Payload
    * Last 2 bytes : CRC (low byte first)
    */
    uart_read_bytes(
        UART_PORT,
        response,
        num_bytes + 5,
        pdMS_TO_TICKS(1000));

// ======= Changes from the original code begin from here ========

    // Return true if expected number of bytes were recieved
    // Otherwise, return false
    if (len != (num_bytes + 5))
    {
        return false;
    }

    if (response[0] != 0x68 ||
        response[1] != 0x44 ||
        response[2] != num_bytes)
    {
        return false;
    }

// Combine 4 big-endian bytes into a signed 32-bit integer.
    uint16_t received_crc =
        response[len-2] |
        (response[len-1] << 8);

    uint16_t calculated_crc =
        modbus_crc16(response, len-2);

    if (received_crc != calculated_crc)
        return false;

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

/**
 * Filter variable of interest by their RAM address
 *
 * Address      MODBUS RAM address
 * Name         Variable name.
 * Unit         Engineering unit
 * K96_DataType Interpretation of the data type.
 */
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

//----- Read and print all sensor data -----
void read_k96(void)
{
    // Checks the sensor's data type
    // 32-bit = 4 bytes
    // 16-bit = 2 bytes
    uint8_t bytes =
        (item->type == K96_S32 || item->type == K96_S32_16) ? 4 : 2;

    // Reads data from the sensor
    // Skip this variable if communication failed and continue reading the rest.
    if (!K96_read_ram(item->address, item->bytes, response))
    {
        printf("0x%04X %-25s : Read failed\n",
                item->address,
                item->name);
        continue;
    }

    printf("------------------------------------------\n");
    printf("Address : 0x%04X\n", item->address);
    printf("Name    : %s\n", item->name);

    // Combine 4 bytes into 32-bit value and prints the result
    if (bytes == 4)
    {
        int32_t raw =
            ((int32_t)response[3] << 24) |
            ((int32_t)response[4] << 16) |
            ((int32_t)response[5] << 8)  |
            response[6];

        switch(item->type)
        {
            case K96_S32:
                printf("Format  : S32\n");
                printf("Value   : %ld %s\n",
                    (long)raw,
                    item->unit);
                break;

            case K96_S32_16:
                printf("Format  : S32.16\n");
                printf("Value   : %.4f %s\n",
                    raw / 65536.0,
                    item->unit);
                break;

            default:
                break;
        }
    }

    else
    {
        // Combine 2 bytes into 16-bit value and prints the result
        uint16_t raw =
            ((uint16_t)response[3] << 8) |
            response[4];

        switch(item->type)
        {
            case K96_U16:
                printf("Format  : U16\n");
                printf("Value   : %u %s\n",
                    raw,
                    item->unit);
                break;

            case K96_S16:
                printf("Format  : S16\n");
                printf("Value   : %d %s\n",
                    (int16_t)raw,
                    item->unit);
                break;

            case K96_B16:
                printf("Format  : B16\n");
                printf("Value   : 0x%04X\n",
                    raw);
                break;

            case K96_S16_8:
                printf("Format  : S16.8\n");
                printf("Value   : %.2f %s\n",
                    ((int16_t)raw) / 256.0,
                    item->unit);
                break;

            default:
                break;
        }
    }
}
        

/*
-----Expected output-----

Address : 0x01B8
Name    : NTC0_Temp
Format  : S16.8
Value   : 24.38 °C

Address : 0x01F0
Name    : RH
Format  : S16
Value   : 43 %RH

Address : 0x0384
Name    : MPL_uflt_IR_Signal
Format  : U16
Value   : 2148 counts

Address : 0x042E
Name    : LPL_uflt_Error
Format  : B16
Value   : 0x0000

Address : 0x0184
Name    : LPL_Signal_filtered
Format  : S32.16
Value   : 153482.1250 counts
*/