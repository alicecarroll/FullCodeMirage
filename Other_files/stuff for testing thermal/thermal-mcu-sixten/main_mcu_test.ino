#include <Wire.h>

// --- Configuration ---
#define I2C_SLAVE_ADDR  0x10   // Matches the default address in the Slave firmware (src/main.cpp)

// ESP32-S3 SuperMini Default I2C Pins
#define SDA_PIN         8      
#define SCL_PIN         9      

// Global Variables
int16_t targetTemperature = 3500; // 35.00 °C (scaled by 100)

// --- Function Prototypes ---
uint8_t calculateCRC8(const uint8_t *data, size_t length);

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); } // Wait for serial console to open

    Serial.println("Initializing ESP32-S3 Master...");

    // Configure I2C pins explicitly first to avoid ambiguous begin() overload compilation errors
    Wire.setPins(SDA_PIN, SCL_PIN);
    if (!Wire.begin()) {
        Serial.println("I2C initialization failed!");
    }
    Wire.setClock(400000); // Set to 400kHz Fast Mode

    Serial.println("Setup completed successfully. Starting channel loop...");
}

void loop() {
    // Loop through all 8 channels (0 to 7)
    for (uint8_t ch = 0; ch < 8; ch++) {
        // Temperature corresponding to its channel: e.g. 20.0°C + ch * 1.0°C
        int16_t currentTemp = (int16_t)((20 + ch) * 100); 

        // ==========================================
        // 1. Prepare Master Write Packet (8 Bytes)
        // ==========================================
        uint8_t writePacket[8];
        writePacket[0] = 0x01; // Command / Register: Update Channel State
        writePacket[1] = ch;   // Channel ID: 0x00 to 0x07
        writePacket[2] = 205;  // Mode / Power: 155 + 50% = 205 (Manual mode at 50% power)
        
        // Split Current Temperature into MSB and LSB
        writePacket[3] = (uint8_t)((currentTemp >> 8) & 0xFF);
        writePacket[4] = (uint8_t)(currentTemp & 0xFF);
        
        // Split Target Temperature into MSB and LSB
        writePacket[5] = (uint8_t)((targetTemperature >> 8) & 0xFF);
        writePacket[6] = (uint8_t)(targetTemperature & 0xFF);
        
        // Calculate CRC-8 for bytes 0 through 6
        writePacket[7] = calculateCRC8(writePacket, 7);

        // ==========================================
        // 2. Execute Write-then-Read Transaction
        // ==========================================
        
        // Transmit Write Packet to Slave
        Wire.beginTransmission(I2C_SLAVE_ADDR);
        Wire.write(writePacket, 8);
        uint8_t writeStatus = Wire.endTransmission();

        if (writeStatus != 0) {
            Serial.printf("Ch %d -> I2C Write Error Code: %d\n", ch, writeStatus);
            delay(100); // Short delay before next channel attempt
            continue;
        }

        // Give the slave ISR a moment to process the write packet
        delayMicroseconds(200);

        // Request 8-Byte Read Packet immediately from Slave
        uint8_t bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 8);
        
        if (bytesReceived == 8) {
            uint8_t readPacket[8];
            for (int i = 0; i < 8; i++) {
                readPacket[i] = Wire.read();
            }

            // Validate Incoming Packet CRC
            uint8_t calculatedCRC = calculateCRC8(readPacket, 7);
            if (calculatedCRC != readPacket[7]) {
                Serial.printf("Ch %d -> CRITICAL: Master-side Read CRC Mismatch! Discarding packet.\n", ch);
                continue;
            }

            // Check for "Bad CRC" Transaction Guard Packet from Slave Nano
            if (readPacket[0] == 0xFF && readPacket[1] == 0xFF && (readPacket[5] & 0x08)) {
                Serial.printf("Ch %d -> WARNING: Slave reported a CRC Error on our last write. Retrying next loop...\n", ch);
                continue;
            }

            // ==========================================
            // 3. Process Valid Status Response
            // ==========================================
            uint8_t rxChannel      = readPacket[0];
            uint8_t rxMode         = readPacket[1];
            uint8_t rxPower        = readPacket[2];
            int16_t rxTargetTemp   = (int16_t)((readPacket[3] << 8) | readPacket[4]);
            uint8_t rxStatusByte   = readPacket[5];

            Serial.printf("[Ch %d Status] Mode: %d | Power: %d%% | Target: %.2f °C | Flags: 0x%02X\n", 
                          rxChannel, rxMode, rxPower, rxTargetTemp / 100.0f, rxStatusByte);

        } else {
            Serial.printf("Ch %d -> Error: Did not receive expected 8 bytes from I2C Slave.\n", ch);
        }

        // Small inter-packet delay to keep bus communication clean
        delay(50); 
    }

    Serial.println("--------------------------------------------------\n");
    delay(2000); // Repeat the entire channel scan every 2 seconds
}

/**
 * CRC-8 Calculation Algorithm (SMBus polynomial: x^8 + x^2 + x + 1 -> 0x07)
 */
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
