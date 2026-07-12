#ifndef I2C_COMM_H
#define I2C_COMM_H

#include <Arduino.h>

// Calculates CRC-8 of a data buffer using standard polynomial 0x07 (SMBus)
uint8_t calculateCRC8(const uint8_t *data, size_t length);

// Callback triggered when the I2C Master writes to the slave
void onI2CReceive(int howMany);

// Callback triggered when the I2C Master requests data from the slave
void onI2CRequest();

#endif // I2C_COMM_H
