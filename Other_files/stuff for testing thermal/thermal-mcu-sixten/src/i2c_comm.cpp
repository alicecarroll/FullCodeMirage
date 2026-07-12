#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "i2c_comm.h"

uint8_t calculateCRC8(const uint8_t *data, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void onI2CReceive(int howMany) {
    // We expect exactly 8 bytes for a valid Write command
    if (howMany != 8) {
        // Drain any buffer content to prevent locking the bus
        while (Wire.available()) {
            Wire.read();
        }
        lastAddressedChannel = 0xFF; // Discard invalid length packet
        return;
    }

    uint8_t rxBuffer[8];
    for (int i = 0; i < 8; i++) {
        rxBuffer[i] = Wire.read();
    }

    // Verify CRC-8 checksum
    uint8_t calcCrc = calculateCRC8(rxBuffer, 7);
    if (calcCrc != rxBuffer[7]) {
        Serial.printf("I2C Receive CRC Error: calculated 0x%02X, received 0x%02X\n", calcCrc, rxBuffer[7]);
        lastAddressedChannel = 0xFF; // Bad CRC: treat channel ID as corrupted
        return;
    }

    uint8_t ch = rxBuffer[1];
    if (ch >= 8) {
        lastAddressedChannel = 0xFF; // Invalid channel ID
        return;
    }

    // Try to acquire the mutex for this channel with a 5ms timeout.
    // ESP32 Arduino Core runs I2C callbacks in task context, making timeouts safe.
    if (xSemaphoreTake(channelMutexes[ch], pdMS_TO_TICKS(5)) == pdTRUE) {
        lastAddressedChannel = ch;

        // Parse Mode byte
        uint8_t modeByte = rxBuffer[2];
        if (modeByte == 0) {
            channelStates[ch].mode = MODE_HYSTERESIS;
        } else if (modeByte == 1) {
            channelStates[ch].mode = MODE_PID;
        } else if (modeByte >= 155 && modeByte <= 255) {
            channelStates[ch].mode = MODE_MANUAL;
            channelStates[ch].manualPower = modeByte - 155; // 155 -> 0%, 255 -> 100%
        } else {
            // Keep current mode if mode byte is out of range
        }

        // Parse 16-bit temperature fields (MSB/LSB)
        int16_t curTemp = (int16_t)((rxBuffer[3] << 8) | rxBuffer[4]);
        int16_t tgtTemp = (int16_t)((rxBuffer[5] << 8) | rxBuffer[6]);

        channelStates[ch].currentTemp = curTemp;
        channelStates[ch].targetTemp = tgtTemp;
        channelStates[ch].lastWriteTimeMs = millis(); // Reset timeout watchdog

        // Clear communication timeout flag upon successful communication
        channelStates[ch].statusByte &= ~FLAG_COMM_TIMEOUT;

        xSemaphoreGive(channelMutexes[ch]);
    } else {
        // Mutex contention (highly unlikely to hit the 5ms timeout, but handle it gracefully)
        lastAddressedChannel = 0xFF;
    }
}

void onI2CRequest() {
    uint8_t txBuffer[8] = {0};

    // Case 1: The last Write transaction was corrupted or invalid
    if (lastAddressedChannel == 0xFF) {
        txBuffer[0] = 0xFF; // Invalid Channel ID
        txBuffer[1] = 0xFF; // Invalid Mode
        txBuffer[2] = 0;    // Power = 0
        txBuffer[3] = 0;    // Target Temp MSB = 0
        txBuffer[4] = 0;    // Target Temp LSB = 0
        txBuffer[5] = FLAG_CRC_ERROR; // Status indicates CRC error
        txBuffer[6] = 0;    // Padding
        txBuffer[7] = calculateCRC8(txBuffer, 7);

        Wire.write(txBuffer, 8);
        return;
    }

    // Case 2: Normal operation, transmit status of last addressed channel
    uint8_t ch = lastAddressedChannel;
    
    // Acquire the mutex to read the channel state safely
    if (xSemaphoreTake(channelMutexes[ch], pdMS_TO_TICKS(5)) == pdTRUE) {
        ControlMode mode = channelStates[ch].mode;
        uint8_t power = channelStates[ch].outputPower;
        int16_t targetTemp = channelStates[ch].targetTemp;
        uint8_t statusByte = channelStates[ch].statusByte;

        xSemaphoreGive(channelMutexes[ch]);

        // Map internal ControlMode back to the active mode protocol byte (0, 1, 2)
        uint8_t activeModeByte = 0;
        if (mode == MODE_HYSTERESIS) {
            activeModeByte = 0;
        } else if (mode == MODE_PID) {
            activeModeByte = 1;
        } else if (mode == MODE_MANUAL) {
            activeModeByte = 2;
        }

        txBuffer[0] = ch;
        txBuffer[1] = activeModeByte;
        txBuffer[2] = power;
        txBuffer[3] = (uint8_t)(targetTemp >> 8);
        txBuffer[4] = (uint8_t)(targetTemp & 0xFF);
        txBuffer[5] = statusByte;
        txBuffer[6] = 0; // Padding
        txBuffer[7] = calculateCRC8(txBuffer, 7);
    } else {
        // Mutex timeout fallback (safety packets)
        txBuffer[0] = ch;
        txBuffer[1] = 0xFF;
        txBuffer[2] = 0;
        txBuffer[3] = 0;
        txBuffer[4] = 0;
        txBuffer[5] = FLAG_SENSOR_ERROR; // Treat lock failure as temporary error state
        txBuffer[6] = 0;
        txBuffer[7] = calculateCRC8(txBuffer, 7);
    }

    Wire.write(txBuffer, 8);
}
