#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "storage.h"
#include "i2c_comm.h"
#include "control.h"
#include "pwm.h"
#include "cli.h"

#define I2C_SLAVE_ADDR 0x10 // Default I2C slave address

// Define pin mapping with external linkage
extern const uint8_t channelPins[8] = {
    D2, // Ch 0 [sdcrd]
    D3, // Ch 1 [P_ch]
    D4, // Ch 2 [outl]
    D5, // Ch 3 [INlt 1]
    D6, // Ch 4 [INLT2]
    D7, // Ch 5 [PLT1]
    D8, // Ch 6 [PLT2]
    D9  // Ch 7 [BCKUP]
};

// Define globals
ChannelState channelStates[8];
ChannelConfig channelConfigs[8];
SemaphoreHandle_t channelMutexes[8];
volatile uint8_t lastAddressedChannel = 0xFF;

// FreeRTOS Task Functions
void controlTaskFunc(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1 Hz (1 second interval)

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        updateControlLoops();
    }
}

void pwmTaskFunc(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10 Hz (100 ms interval)

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        updateSoftwarePWM();
    }
}

void cliTaskFunc(void *pvParameters) {
    for (;;) {
        runCLI();
        vTaskDelay(pdMS_TO_TICKS(50)); // Non-blocking polling delay
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000); // Give serial monitor time to connect
    Serial.println("Initializing Thermal MCU V3...");

    // 1. Initialize pins and create mutex semaphores
    for (int i = 0; i < 8; i++) {
        pinMode(channelPins[i], OUTPUT);
        digitalWrite(channelPins[i], LOW);
        
        channelMutexes[i] = xSemaphoreCreateMutex();
        if (channelMutexes[i] == NULL) {
            Serial.printf("Fatal Error: Failed to create Mutex for Channel %d\n", i);
            while(1);
        }

        // Initialize state variables to safe startup defaults
        channelStates[i].mode = MODE_HYSTERESIS;
        channelStates[i].manualPower = 0;
        channelStates[i].currentTemp = 0;
        channelStates[i].targetTemp = 0;
        channelStates[i].outputPower = 0;
        channelStates[i].statusByte = 0;
        channelStates[i].lastWriteTimeMs = 0;
        channelStates[i].maxPowerStartTimeMs = 0;
        channelStates[i].tempAtMaxPowerStart = 0;
        channelStates[i].integralError = 0.0f;
        channelStates[i].previousTemp = 0;
    }

    // 2. Load settings from NVS
    loadConfiguration();

    // 3. Initialize I2C Slave Interface
    Wire.begin(I2C_SLAVE_ADDR);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);
    Serial.printf("I2C Slave active on address 0x%02X\n", I2C_SLAVE_ADDR);

    // 4. Spawn FreeRTOS tasks on Core 1 (Core 0 runs the communication stacks)
    // Priority levels: PWM Task (3, highest for precise timing) > Control Task (2) > CLI Task (1)
    xTaskCreatePinnedToCore(
        pwmTaskFunc,
        "PWM_Task",
        4096,
        NULL,
        3,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        controlTaskFunc,
        "Control_Task",
        4096,
        NULL,
        2,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        cliTaskFunc,
        "CLI_Task",
        4096,
        NULL,
        1,
        NULL,
        1
    );

    Serial.println("System Initialization Complete. Running tasks...");
}

void loop() {
    // Arduino main loop runs on Core 1 at priority 1 by default.
    // We let our spawned tasks manage everything and delete/idle this loop.
    vTaskDelete(NULL);
}