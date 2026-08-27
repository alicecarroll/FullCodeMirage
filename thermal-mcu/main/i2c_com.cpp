// libaries
#include <cstdint>
#include <cstring>

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "string.h"

// other parts of project
#include "config.h"
#include "i2c_com.h"

// numbers for functions
constexpr size_t INDIVIDUAL_SWITCH_RX_LEN = 8;
constexpr size_t INDIVIDUAL_SWITCH_TX_LEN = 8;

// Crc 8 table. Is a table to make algorithm more compute efficient DO NOT TOUCH
// OR BECOME SAD
uint8_t crc8_table[256] = {
    0,   7,   14,  9,   28,  27,  18,  21,  56,  63,  54,  49,  36,  35,  42,
    45,  112, 119, 126, 121, 108, 107, 98,  101, 72,  79,  70,  65,  84,  83,
    90,  93,  224, 231, 238, 233, 252, 251, 242, 245, 216, 223, 214, 209, 196,
    195, 202, 205, 144, 151, 158, 153, 140, 139, 130, 133, 168, 175, 166, 161,
    180, 179, 186, 189, 199, 192, 201, 206, 219, 220, 213, 210, 255, 248, 241,
    246, 227, 228, 237, 234, 183, 176, 185, 190, 171, 172, 165, 162, 143, 136,
    129, 134, 147, 148, 157, 154, 39,  32,  41,  46,  59,  60,  53,  50,  31,
    24,  17,  22,  3,   4,   13,  10,  87,  80,  89,  94,  75,  76,  69,  66,
    111, 104, 97,  102, 115, 116, 125, 122, 137, 142, 135, 128, 149, 146, 155,
    156, 177, 182, 191, 184, 173, 170, 163, 164, 249, 254, 247, 240, 229, 226,
    235, 236, 193, 198, 207, 200, 221, 218, 211, 212, 105, 110, 103, 96,  117,
    114, 123, 124, 81,  86,  95,  88,  77,  74,  67,  68,  25,  30,  23,  16,
    5,   2,   11,  12,  33,  38,  47,  40,  61,  58,  51,  52,  78,  73,  64,
    71,  82,  85,  92,  91,  118, 113, 120, 127, 106, 109, 100, 99,  62,  57,
    48,  55,  34,  37,  44,  43,  6,   1,   8,   15,  26,  29,  20,  19,  174,
    169, 160, 167, 178, 181, 188, 187, 150, 145, 152, 159, 138, 141, 132, 131,
    222, 217, 208, 215, 194, 197, 204, 203, 230, 225, 232, 239, 250, 253, 244,
    243};

// Crc8 algorithm using table above. Uses table to make algorithm more efficient
uint8_t computeCRC8(const uint8_t *data, size_t length) {
  uint8_t crc = 0x00;

  for (int i = 0; i < length; i++) {
    crc = crc8_table[data[i] ^ crc];
  }
  return crc;
}

// This one should be in its own freertos task which will run paralell to the
// controller task Fix sending stuff to main I2C Receive task
void i2c_loop_task(void *pvParameters) { // If still not working
  bool packet_error = false;
  const individual_switch_data_rx default_off_package = {
      .regist = 0x01,
      .switchID = 0x00,
      .mode = 155,
      .temperature = 20,
      .target = 20,
      .crc8 = 0}; // crc8 should get actual calue but doesnt really matter
  individual_switch_data_rx receive_indvidual_switch_packet;
  individual_switch_data_rx controllerData;
  i2c_data_evt evtData; // Data from i2c

  // send
  i2c_data_evt send;
  // i2c stuff
  uint8_t rx[rx_buffer_len] = {};
  size_t buffered = 0;

  while (1) {

    const int received = i2c_slave_read_buffer(
        I2C_PORT, rx + buffered, sizeof(rx) - buffered, pdMS_TO_TICKS(10));
    if (received > 0)
      buffered += static_cast<size_t>(received);
    while (buffered > 0) {
      // ESP_LOGI("I2C loop", " DATA RECEIVED");
      size_t send_len = 0;
      packet_error = false;
      uint8_t regi = rx[0];
      switch (regi) { // inside this switch case multiple package types will be

      // implemented
      case packet_type_indvidual_switch:
        send_len = 8;
        if (!data_unpack_indvidual_switch(rx,
                                          &receive_indvidual_switch_packet)) {
          ESP_LOGE("I2C loop:", "CRC8 failed for indvidual switch packet %d",
                   regi);
          packet_error = true;
        } else if (receive_indvidual_switch_packet.switchID >=
                   number_switches) {
          packet_error = true;
          ESP_LOGE("I2C loop:", "Incorrect switch ID %d", regi);
        } else {
          controllerData = receive_indvidual_switch_packet;
          // ESP_LOGI("I2C loop", " Switch: %d, Mode: %d",
          // controllerData.switchID, controllerData.mode);
        }
        // implement clear of databuffer after read message
        // Add timeout for specific packages ie replace crc8 with time since
        // this package has last been received

        break;
      case packet_stop_all: // Emergency stop
        send_len = 1;
        controllerData = default_off_package;
        controllerData.regist = packet_stop_all;
        break;
      case packet_resume_all: // Resume normal operations
        send_len = 1;
        controllerData = default_off_package;
        controllerData.regist = packet_resume_all;
        break;
      default:
        memmove(rx, rx + 1, --buffered);
        ESP_LOGW("I2C", "Unknown packet register: 0x%02X", regi);
        continue;
        break;
      }
      // here it should send data to another freertos task
      if (buffered < send_len) {
        break;
      }
      buffered -= send_len;
      memmove(rx, rx + send_len, buffered);

      if (!packet_error) {
        // Data to control
        // ESP_LOGI("I2C loop", " Switch: %d, Mode: %d",
        // controllerData.switchID, controllerData.mode);
        if (xQueueSend(dataQueue, &controllerData, 0) !=
            pdPASS) // If it cant queue controllerdata clear queue and que the
                    // data
        { // This makes sure that controller gets newest data but the downside
          // is that if it triggers it will delete data
          xQueueReset(dataQueue);
          xQueueSend(dataQueue, &controllerData, 0);
        }
      }
      // Add a way to send packeterrors to master
    }
    // sends stuff
    if (xQueueReceive(dataQueue_slave_tx, &send, 0)) {
      // ESP_LOGI("I2C_SLAVE","Size of send data %zu", send.length);
      i2c_reset_tx_fifo(I2C_PORT);
      i2c_slave_write_buffer(
          I2C_PORT, send.data, send.length,
          0); // can use sizeof(send.data) instead of send.length
    }
  }
}

/*
Under this is packet structures
*/

// receive packets

// thermal indvidual switch packet
bool data_unpack_indvidual_switch(
    const uint8_t *dataBuffer,
    individual_switch_data_rx *packet) { // Input uint8 arary and get a struct
  if (dataBuffer[INDIVIDUAL_SWITCH_RX_LEN - 1] !=
      computeCRC8(dataBuffer, INDIVIDUAL_SWITCH_RX_LEN - 1)) {
    return false; // Crc8 didn't pass so bad datastream
  }

  // unpacks buffer into the packet
  packet->regist = dataBuffer[0];   // regiester/command determines packet type
  packet->switchID = dataBuffer[1]; // switch id 0-7
  packet->mode = dataBuffer[2];     // mode
  packet->temperature =
      static_cast<int16_t>(static_cast<uint16_t>(dataBuffer[3]) << 8 |
                           static_cast<uint16_t>(dataBuffer[4]));
  packet->target =
      static_cast<int16_t>(static_cast<uint16_t>(dataBuffer[5]) << 8 |
                           static_cast<uint16_t>(dataBuffer[6]));
  packet->crc8 = dataBuffer[7];

  return true;
}

// send packets
// Thermal indvidual switch send packet
bool data_pack_indvidual_switch( // input a struct and get ouy uint8array
    const individual_switch_data_tx *packet,
    uint8_t *data // data should be 1 byte more than packet
) {
  int16_t target_int = static_cast<int16_t>(
      packet->target *
      100.0f); // Keep in mind that code will not work for numbers above 320C or
               // below -320C due to limits of int16
  uint16_t unsigned_target = static_cast<uint16_t>(target_int);

  data[0] = packet->switchID;
  data[1] = packet->mode;
  data[2] = packet->D_cycle;
  data[3] = static_cast<uint8_t>((unsigned_target >> 8) & 0xFF); // msb
  data[4] = static_cast<uint8_t>((unsigned_target) & 0xFF);      // lsb
  data[5] = packet->status;                                      // Errors
  data[6] = packet->global_mode; // Which global mode its in

  data[7] = computeCRC8(data, INDIVIDUAL_SWITCH_TX_LEN - 1);
  return true;
}
