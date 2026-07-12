#include <Arduino.h>
#include "config.h"
#include "pwm.h"

// Track the start time of the PWM cycle for each of the 8 channels
static uint32_t channelCycleStart[8] = { 0 };

void updateSoftwarePWM() {
    uint32_t now = millis();

    for (int ch = 0; ch < 8; ch++) {
        uint8_t power = 0;
        uint32_t period = 20000; // Default fallback 20s

        // Acquire lock to read configuration and output power safely.
        // We use a non-blocking timeout of 2ms to prevent stalling the PWM task.
        if (xSemaphoreTake(channelMutexes[ch], pdMS_TO_TICKS(2)) == pdTRUE) {
            power = channelStates[ch].outputPower;
            period = channelConfigs[ch].pwmPeriodMs;
            xSemaphoreGive(channelMutexes[ch]);
        } else {
            // If the lock is held (which is rare), we keep the current state of the pin 
            // for this 100ms cycle to avoid high-frequency jitter.
            continue;
        }

        // Calculate time elapsed since the start of the current PWM cycle
        uint32_t cycleElapsed = now - channelCycleStart[ch];

        // Reset the cycle start timer if the period has elapsed
        if (cycleElapsed >= period) {
            channelCycleStart[ch] = now;
            cycleElapsed = 0;
        }

        // Calculate active ON time in milliseconds
        uint32_t onDurationMs = (period * power) / 100;

        // Toggle the physical GPIO pin
        if (cycleElapsed < onDurationMs) {
            digitalWrite(channelPins[ch], HIGH);
        } else {
            digitalWrite(channelPins[ch], LOW);
        }
    }
}
