#include <Arduino.h>
#include "config.h"
#include "storage.h"
#include "cli.h"

// Channel names array
const char* channelNames[8] = {
    "sdcrd",
    "P_ch",
    "outl",
    "INlt 1",
    "INLT2",
    "PLT1",
    "PLT2",
    "BCKUP"
};

// Help print utility
static void printHelp() {
    Serial.println("\n--- Thermal MCU V3 CLI Help ---");
    Serial.println("Commands:");
    Serial.println("  help                             - Show this menu");
    Serial.println("  status                           - Print status table for all 8 channels");
    Serial.println("  set <ch> <m|b|p> <val>           - Override channel configuration:");
    Serial.println("                                     e.g. set 1 m 75    (Ch 1 Manual, 75% power)");
    Serial.println("                                     e.g. set 3 p 45.5  (Ch 3 PID, Target 45.5 C)");
    Serial.println("                                     e.g. set 0 b 37.0  (Ch 0 Hysteresis, Target 37.0 C)");
    Serial.println("  tune <ch> <kp> <ki> <kd>         - Tune PID coefficients (floats)");
    Serial.println("  config <ch> <h|c> <period_s>     - Config type (h=heater, c=cooler) and PWM period in seconds (10-60)");
    Serial.println("  save                             - Write current configurations to NVS (EEPROM)");
    Serial.println("  load                             - Reload configurations from NVS");
    Serial.println("  monitor                          - Periodically print status every 500ms (press any key to stop)");
    Serial.println("---------------------------------");
}

// Formatted channel status printer
static void printStatusTable() {
    Serial.println("\n-----------------------------------------------------------------------------------------------------------");
    Serial.println("| Ch | Name    | Pin | Config | Period | Mode | Power | Cur Temp | Tgt Temp | PinState | Status Flags      |");
    Serial.println("-----------------------------------------------------------------------------------------------------------");
    
    for (int ch = 0; ch < 8; ch++) {
        bool isCooler = false;
        uint32_t period = 0;
        ControlMode mode = MODE_HYSTERESIS;
        uint8_t power = 0;
        int16_t curTemp = 0;
        int16_t tgtTemp = 0;
        uint8_t statusByte = 0;
        uint8_t pinState = digitalRead(channelPins[ch]);

        // Safely lock and read state
        if (xSemaphoreTake(channelMutexes[ch], pdMS_TO_TICKS(5)) == pdTRUE) {
            isCooler = channelConfigs[ch].isCooler;
            period = channelConfigs[ch].pwmPeriodMs;
            mode = channelStates[ch].mode;
            power = channelStates[ch].outputPower;
            curTemp = channelStates[ch].currentTemp;
            tgtTemp = channelStates[ch].targetTemp;
            statusByte = channelStates[ch].statusByte;
            xSemaphoreGive(channelMutexes[ch]);
        }

        // Mode name mapping
        const char* modeStr = "HYST";
        if (mode == MODE_PID) modeStr = "PID ";
        else if (mode == MODE_MANUAL) modeStr = "MAN ";

        // Decode safety status flags to human readable warnings
        char flagStr[32] = "OK";
        if (statusByte != 0) {
            flagStr[0] = '\0';
            if (statusByte & FLAG_COMM_TIMEOUT) strcat(flagStr, "TIMEOUT ");
            if (statusByte & FLAG_THERMAL_RUNAWAY) strcat(flagStr, "RUNAWAY ");
            if (statusByte & FLAG_SENSOR_ERROR) strcat(flagStr, "SENSOR_ERR ");
            if (statusByte & FLAG_CRC_ERROR) strcat(flagStr, "CRC_ERR ");
        }

        Serial.printf("| %d  | %-7s | D%d  | %-6s | %5.1fs | %s |  %3d%% |  %6.2fC |  %6.2fC |    %-5s | %-17s |\n",
                      ch,
                      channelNames[ch],
                      channelPins[ch] - D0, // Convert Arduino ESP32 internal pin offset back to simple D index
                      isCooler ? "COOLER" : "HEATER",
                      period / 1000.0f,
                      modeStr,
                      power,
                      curTemp / 100.0f,
                      tgtTemp / 100.0f,
                      pinState ? "HIGH" : "LOW",
                      flagStr);
    }
    Serial.println("-----------------------------------------------------------------------------------------------------------");
}

void runCLI() {
    static char rxBuffer[128];
    static int rxIndex = 0;
    static bool isMonitoring = false;
    static uint32_t lastMonitorTimeMs = 0;

    if (isMonitoring) {
        if (Serial.available() > 0) {
            // Drain serial input to cancel monitoring mode
            while (Serial.available() > 0) {
                Serial.read();
            }
            isMonitoring = false;
            Serial.println("\nMonitoring stopped. CLI ready.");
            rxIndex = 0; // Reset parse index
            return;
        }

        uint32_t now = millis();
        if (now - lastMonitorTimeMs >= 500) {
            lastMonitorTimeMs = now;
            printStatusTable();
            Serial.println("Press any key to stop monitoring...");
        }
        return; // Intercept normal CLI parser during active monitoring
    }

    while (Serial.available() > 0) {
        char c = Serial.read();

        // Echo characters back to terminal
        Serial.print(c);

        if (c == '\r' || c == '\n') {
            rxBuffer[rxIndex] = '\0';
            Serial.println(); // Add new line

            if (rxIndex > 0) {
                // Command processing
                char cmd[16] = {0};
                sscanf(rxBuffer, "%15s", cmd);

                if (strcmp(cmd, "help") == 0) {
                    printHelp();
                } else if (strcmp(cmd, "status") == 0) {
                    printStatusTable();
                } else if (strcmp(cmd, "save") == 0) {
                    saveConfiguration();
                } else if (strcmp(cmd, "load") == 0) {
                    loadConfiguration();
                } else if (strcmp(cmd, "monitor") == 0) {
                    isMonitoring = true;
                    lastMonitorTimeMs = millis();
                    printStatusTable();
                    Serial.println("Press any key to stop monitoring...");
                } else if (strcmp(cmd, "set") == 0) {
                    int ch = -1;
                    char modeChar = '\0';
                    float val = 0.0f;
                    int parsed = sscanf(rxBuffer, "set %d %c %f", &ch, &modeChar, &val);

                    if (parsed == 3 && ch >= 0 && ch < 8) {
                        if (xSemaphoreTake(channelMutexes[ch], portMAX_DELAY) == pdTRUE) {
                            if (modeChar == 'm') {
                                channelStates[ch].mode = MODE_MANUAL;
                                channelStates[ch].manualPower = (uint8_t)constrain(val, 0, 100);
                                Serial.printf("Ch %d set to Manual mode with %d%% power.\n", ch, channelStates[ch].manualPower);
                            } else if (modeChar == 'b') {
                                channelStates[ch].mode = MODE_HYSTERESIS;
                                channelStates[ch].targetTemp = (int16_t)(val * 100.0f);
                                Serial.printf("Ch %d set to Hysteresis mode with target %.2f C.\n", ch, val);
                            } else if (modeChar == 'p') {
                                channelStates[ch].mode = MODE_PID;
                                channelStates[ch].targetTemp = (int16_t)(val * 100.0f);
                                Serial.printf("Ch %d set to PID mode with target %.2f C.\n", ch, val);
                            } else {
                                Serial.println("Error: Invalid mode char. Use m, b, or p.");
                            }
                            // Reset communication watchdog on user command override
                            channelStates[ch].lastWriteTimeMs = millis();
                            channelStates[ch].statusByte &= ~FLAG_COMM_TIMEOUT;
                            xSemaphoreGive(channelMutexes[ch]);
                        }
                    } else {
                        Serial.println("Syntax Error: Use 'set <ch> <m|b|p> <val>'");
                    }
                } else if (strcmp(cmd, "tune") == 0) {
                    int ch = -1;
                    float kp = 0.0f, ki = 0.0f, kd = 0.0f;
                    int parsed = sscanf(rxBuffer, "tune %d %f %f %f", &ch, &kp, &ki, &kd);

                    if (parsed == 4 && ch >= 0 && ch < 8) {
                        if (xSemaphoreTake(channelMutexes[ch], portMAX_DELAY) == pdTRUE) {
                            channelConfigs[ch].kp = kp;
                            channelConfigs[ch].ki = ki;
                            channelConfigs[ch].kd = kd;
                            xSemaphoreGive(channelMutexes[ch]);
                            Serial.printf("Ch %d PID coefficients tuned to: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", ch, kp, ki, kd);
                        }
                    } else {
                        Serial.println("Syntax Error: Use 'tune <ch> <kp> <ki> <kd>'");
                    }
                } else if (strcmp(cmd, "config") == 0) {
                    int ch = -1;
                    char typeChar = '\0';
                    int periodSec = 0;
                    int parsed = sscanf(rxBuffer, "config %d %c %d", &ch, &typeChar, &periodSec);

                    if (parsed == 3 && ch >= 0 && ch < 8) {
                        if (xSemaphoreTake(channelMutexes[ch], portMAX_DELAY) == pdTRUE) {
                            if (typeChar == 'h') {
                                channelConfigs[ch].isCooler = false;
                                Serial.printf("Ch %d configured as HEATER.\n", ch);
                            } else if (typeChar == 'c') {
                                channelConfigs[ch].isCooler = true;
                                Serial.printf("Ch %d configured as COOLER.\n", ch);
                            } else {
                                Serial.println("Error: Invalid configuration type. Use h or c.");
                            }
                            
                            channelConfigs[ch].pwmPeriodMs = constrain(periodSec, 10, 60) * 1000;
                            Serial.printf("Ch %d PWM Period set to %d seconds.\n", ch, periodSec);
                            
                            xSemaphoreGive(channelMutexes[ch]);
                        }
                    } else {
                        Serial.println("Syntax Error: Use 'config <ch> <h|c> <period_seconds>'");
                    }
                } else {
                    Serial.println("Unknown command. Type 'help' for options.");
                }
            }
            rxIndex = 0; // Clear buffer
        } else {
            // Fill buffer, ignore backspace/delete characters for simplicity
            if (c >= 32 && c <= 126 && rxIndex < (int)sizeof(rxBuffer) - 1) {
                rxBuffer[rxIndex++] = c;
            }
        }
    }
}
