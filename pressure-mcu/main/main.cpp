#include "main.h"
#include "pressure_protocol.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

namespace {
constexpr i2c_port_t I2C_PORT = I2C_NUM_1;
constexpr gpio_num_t I2C_SDA = GPIO_NUM_13;
constexpr gpio_num_t I2C_SCL = GPIO_NUM_14;
constexpr uint8_t I2C_ADDRESS = 0x10;
constexpr float SENSOR_SCALE = 100.0f;
constexpr char TAG[] = "pressure_i2c";

const uint8_t crc8_table[256] = {
    0,7,14,9,28,27,18,21,56,63,54,49,36,35,42,45,112,119,126,121,108,107,98,101,72,79,70,65,84,83,90,93,
    224,231,238,233,252,251,242,245,216,223,214,209,196,195,202,205,144,151,158,153,140,139,130,133,168,175,166,161,180,179,186,189,
    199,192,201,206,219,220,213,210,255,248,241,246,227,228,237,234,183,176,185,190,171,172,165,162,143,136,129,134,147,148,157,154,
    39,32,41,46,59,60,53,50,31,24,17,22,3,4,13,10,87,80,89,94,75,76,69,66,111,104,97,102,115,116,125,122,
    137,142,135,128,149,146,155,156,177,182,191,184,173,170,163,164,249,254,247,240,229,226,235,236,193,198,207,200,221,218,211,212,
    105,110,103,96,117,114,123,124,81,86,95,88,77,74,67,68,25,30,23,16,5,2,11,12,33,38,47,40,61,58,51,52,
    78,73,64,71,82,85,92,91,118,113,120,127,106,109,100,99,62,57,48,55,34,37,44,43,6,1,8,15,26,29,20,19,
    174,169,160,167,178,181,188,187,150,145,152,159,138,141,132,131,222,217,208,215,194,197,204,203,230,225,232,239,250,253,244,243
};

uint8_t crc8(const uint8_t *data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i) crc = crc8_table[data[i] ^ crc];
    return crc;
}

void queue_status() {
    const PressureStatus status = pressure_get_status();
    uint8_t frame[PRESSURE_STATUS_FRAME_SIZE] = {};
    frame[1] = static_cast<uint8_t>(status.state);
    frame[2] = status.error;
    // Relay bits occupy 0..3. Manual override and valve have dedicated bits.
    frame[3] = (status.relay_mask & 0x0F) |
               (status.relay_manual_override ? 0x40 : 0) |
               (status.valve_open ? 0x80 : 0);
    frame[4] = status.pump1_pwm;
    frame[5] = status.pump2_pwm;
    frame[6] = status.compressor_pwm;
    frame[7] = crc8(frame, 7);
    i2c_reset_tx_fifo(I2C_PORT);
    i2c_slave_write_buffer(I2C_PORT, frame, sizeof(frame), 0);
}

void process_frame(const uint8_t *frame, size_t length) {
    if (length == PRESSURE_SENSOR_FRAME_SIZE && frame[0] == PRESSURE_PACKET_SENSORS) {
        if (crc8(frame, 15) != frame[15]) return;
        float sensors[7] = {};
        for (size_t i = 0; i < 7; ++i) {
            const int16_t raw = static_cast<int16_t>(static_cast<uint16_t>(frame[1 + 2 * i]) << 8 | frame[2 + 2 * i]);
            sensors[i] = static_cast<float>(raw) / SENSOR_SCALE;
        }
        pressure_update_external_sensors(sensors);
        return;
    }
    if (length == PRESSURE_COMMAND_FRAME_SIZE && frame[0] == PRESSURE_PACKET_COMMANDS) {
        if (crc8(frame, 3) == frame[3]) pressure_execute_command(frame[1], frame[2]);
    }
}

void i2c_task(void *) {
    uint8_t rx[64] = {};
    size_t buffered = 0;
    while (true) {
        const int received = i2c_slave_read_buffer(I2C_PORT, rx + buffered, sizeof(rx) - buffered, pdMS_TO_TICKS(10));
        if (received > 0) buffered += static_cast<size_t>(received);
        while (buffered > 0) {
            size_t frame_length = 0;
            if (rx[0] == PRESSURE_PACKET_SENSORS) frame_length = PRESSURE_SENSOR_FRAME_SIZE;
            else if (rx[0] == PRESSURE_PACKET_COMMANDS) frame_length = PRESSURE_COMMAND_FRAME_SIZE;
            else { memmove(rx, rx + 1, --buffered); continue; }
            if (buffered < frame_length) break;
            process_frame(rx, frame_length);
            buffered -= frame_length;
            memmove(rx, rx + frame_length, buffered);
        }
        queue_status();
        pressure_update();
    }
}
}

extern "C" void app_main() {
    i2c_config_t config = {};
    config.mode = I2C_MODE_SLAVE;
    config.sda_io_num = I2C_SDA;
    config.scl_io_num = I2C_SCL;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.slave.addr_10bit_en = 0;
    config.slave.slave_addr = I2C_ADDRESS;
    config.slave.maximum_speed = 100000;
    i2c_param_config(I2C_PORT, &config);
    i2c_driver_install(I2C_PORT, I2C_MODE_SLAVE, 128, 128, 0);
    pressure_init();
    xTaskCreate(i2c_task, "pressure_i2c", 4096, nullptr, 5, nullptr);
}
