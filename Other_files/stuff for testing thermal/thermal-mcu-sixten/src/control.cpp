#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "control.h"

void updateControlLoops() {
    for (int ch = 0; ch < 8; ch++) {
        // Lock this channel's mutex to perform thread-safe calculations
        if (xSemaphoreTake(channelMutexes[ch], portMAX_DELAY) == pdTRUE) {
            
            // 1. Sensor Error watchdog
            // Out of bounds checks: Temp < -50.00 C (-5000) or Temp > 250.00 C (25000), or sentinel error code -9999
            if (channelStates[ch].currentTemp == -9999 || 
                channelStates[ch].currentTemp < -30000 ||  
                channelStates[ch].currentTemp > 25000) {
                
                channelStates[ch].statusByte |= FLAG_SENSOR_ERROR;
                
                // Only force shutdown if NOT in manual mode
                if (channelStates[ch].mode != MODE_MANUAL) {
                    channelStates[ch].outputPower = 0; // Force safety state (OFF)
                    channelStates[ch].integralError = 0.0f; // Clear PID accumulator
                    xSemaphoreGive(channelMutexes[ch]);
                    continue; // Skip further control logic for this channel
                }
            } else {
                channelStates[ch].statusByte &= ~FLAG_SENSOR_ERROR;
            }

            // 2. I2C Communication Timeout watchdog
            uint32_t now = millis();
            // If we have never written or last write exceeded configured timeout
            if (channelStates[ch].lastWriteTimeMs == 0 || 
                (now - channelStates[ch].lastWriteTimeMs) > channelConfigs[ch].commTimeoutMs) {
                
                channelStates[ch].statusByte |= FLAG_COMM_TIMEOUT;
                
                // Only force shutdown if NOT in manual mode
                if (channelStates[ch].mode != MODE_MANUAL) {
                    channelStates[ch].outputPower = 0; // Force safety state (OFF)
                    channelStates[ch].integralError = 0.0f; // Clear PID accumulator
                    xSemaphoreGive(channelMutexes[ch]);
                    continue; // Skip further control logic for this channel
                }
            } else {
                channelStates[ch].statusByte &= ~FLAG_COMM_TIMEOUT;
            }

            // Initialize previousTemp on first run to prevent startup derivative kick
            if (channelStates[ch].previousTemp == 0 && channelStates[ch].currentTemp != 0) {
                channelStates[ch].previousTemp = channelStates[ch].currentTemp;
            }

            // 3. Controller execution
            if (channelStates[ch].mode == MODE_MANUAL) {
                channelStates[ch].outputPower = channelStates[ch].manualPower;
                channelStates[ch].integralError = 0.0f; // Reset integrator in manual
                
            } else if (channelStates[ch].mode == MODE_HYSTERESIS) {
                channelStates[ch].integralError = 0.0f; // Reset integrator in hysteresis
                
                int16_t current = channelStates[ch].currentTemp;
                int16_t target = channelStates[ch].targetTemp;
                int16_t delta = channelConfigs[ch].hystDelta;

                if (channelConfigs[ch].isCooler) {
                    // Cooler: Turn ON when temp is too high, turn OFF when temp is low
                    if (current > (target + delta)) {
                        channelStates[ch].outputPower = 100;
                    } else if (current < (target - delta)) {
                        channelStates[ch].outputPower = 0;
                    }
                } else {
                    // Heater: Turn ON when temp is too low, turn OFF when temp is high
                    if (current < (target - delta)) {
                        channelStates[ch].outputPower = 100;
                    } else if (current > (target + delta)) {
                        channelStates[ch].outputPower = 0;
                    }
                }

            } else if (channelStates[ch].mode == MODE_PID) {
                float error = 0.0f;
                float dTerm = 0.0f;
                float kp = channelConfigs[ch].kp;
                float ki = channelConfigs[ch].ki;
                float kd = channelConfigs[ch].kd;

                // Scaled by 100 to convert from scaled integers back to floating point Celsius
                float curTempF = channelStates[ch].currentTemp / 100.0f;
                float tgtTempF = channelStates[ch].targetTemp / 100.0f;
                float prevTempF = channelStates[ch].previousTemp / 100.0f;

                if (channelConfigs[ch].isCooler) {
                    error = curTempF - tgtTempF;
                    // Cooler: derivative is positive relative to temperature increase
                    dTerm = kd * (curTempF - prevTempF);
                } else {
                    error = tgtTempF - curTempF;
                    // Heater: derivative is negative relative to temperature increase
                    dTerm = -kd * (curTempF - prevTempF);
                }

                float pTerm = kp * error;
                float iTermCandidate = channelStates[ch].integralError + (ki * error);
                
                // PID sum calculation
                float outputVal = pTerm + iTermCandidate + dTerm;

                // Clamping and Anti-Windup Guard
                if (outputVal >= 100.0f) {
                    channelStates[ch].outputPower = 100;
                    // Only accumulate if the candidate integral isn't pushing further into saturation
                    if (error < 0.0f) {
                        channelStates[ch].integralError = iTermCandidate;
                    }
                } else if (outputVal <= 0.0f) {
                    channelStates[ch].outputPower = 0;
                    // Only accumulate if the candidate integral isn't pushing further into saturation
                    if (error > 0.0f) {
                        channelStates[ch].integralError = iTermCandidate;
                    }
                } else {
                    channelStates[ch].outputPower = (uint8_t)roundf(outputVal);
                    channelStates[ch].integralError = iTermCandidate;
                }

                // Save current temp as previous temp for next iteration
                channelStates[ch].previousTemp = channelStates[ch].currentTemp;
            }

            // 4. Thermal Runaway (High-Power Guard) monitoring
            if (channelStates[ch].outputPower >= 95) {
                if (channelStates[ch].maxPowerStartTimeMs == 0) {
                    // Start runaway check timer
                    channelStates[ch].maxPowerStartTimeMs = millis();
                    channelStates[ch].tempAtMaxPowerStart = channelStates[ch].currentTemp;
                } else {
                    uint32_t powerTimeElapsed = millis() - channelStates[ch].maxPowerStartTimeMs;
                    if (powerTimeElapsed >= RUNAWAY_CHECK_INTERVAL_MS) {
                        int16_t tempDelta = channelStates[ch].currentTemp - channelStates[ch].tempAtMaxPowerStart;
                        bool runawayTriggered = false;

                        if (channelConfigs[ch].isCooler) {
                            // Cooler should decrease temperature (delta should be negative)
                            // If temperature didn't drop by at least MIN_TEMP_RISE, trigger runaway
                            if (tempDelta > -MIN_TEMP_RISE) {
                                runawayTriggered = true;
                            }
                        } else {
                            // Heater should increase temperature (delta should be positive)
                            // If temperature didn't rise by at least MIN_TEMP_RISE, trigger runaway
                            if (tempDelta < MIN_TEMP_RISE) {
                                runawayTriggered = true;
                            }
                        }

                        if (runawayTriggered) {
                            channelStates[ch].statusByte |= FLAG_THERMAL_RUNAWAY;
                        } else {
                            // Temperature is moving as expected; reset baseline parameters to monitor the next interval
                            channelStates[ch].maxPowerStartTimeMs = millis();
                            channelStates[ch].tempAtMaxPowerStart = channelStates[ch].currentTemp;
                            channelStates[ch].statusByte &= ~FLAG_THERMAL_RUNAWAY;
                        }
                    }
                }
            } else if (channelStates[ch].outputPower < 50) {
                // Hysteresis Reset: Reset timers and clear warnings once output falls below safe threshold
                channelStates[ch].maxPowerStartTimeMs = 0;
                channelStates[ch].statusByte &= ~FLAG_THERMAL_RUNAWAY;
            }

            xSemaphoreGive(channelMutexes[ch]);
        }
    }
}
