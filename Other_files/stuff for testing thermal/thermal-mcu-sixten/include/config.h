#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Control modes
enum ControlMode {
    MODE_HYSTERESIS = 0,
    MODE_PID = 1,
    MODE_MANUAL = 2
};

// Safety warning and error flags (Status Byte)
#define FLAG_COMM_TIMEOUT    0x01  // Bit 0: I2C write timeout (30s)
#define FLAG_THERMAL_RUNAWAY 0x02  // Bit 1: Temp not responding to high power (60s)
#define FLAG_SENSOR_ERROR    0x04  // Bit 2: Out of bounds or sensor disconnect reported
#define FLAG_CRC_ERROR       0x08  // Bit 3: CRC checksum mismatch on last write

// Thermal runaway configuration
const uint32_t RUNAWAY_CHECK_INTERVAL_MS = 60000; // 60 seconds
const int16_t MIN_TEMP_RISE = 50;                  // 0.50 C (scaled by 100)

// Channel configuration (stored in NVS)
struct ChannelConfig {
    bool isCooler;          // true = cooler (reverse acting), false = heater (direct acting)
    uint32_t pwmPeriodMs;   // software PWM period in ms (10,000 to 60,000)
    float kp;               // Proportional coefficient
    float ki;               // Integral coefficient
    float kd;               // Derivative coefficient
    int16_t hystDelta;      // Hysteresis deadband window (scaled by 100, e.g. 50 = 0.50 C)
    uint32_t commTimeoutMs; // Timeout for communications (default 30,000 ms)
};

// Volatile state of a channel (protected by channel-level mutex)
struct ChannelState {
    ControlMode mode;
    uint8_t manualPower;          // Manual output duty cycle (0-100%)
    int16_t currentTemp;          // Current temperature (scaled by 100)
    int16_t targetTemp;           // Target setpoint temperature (scaled by 100)
    uint8_t outputPower;          // Current output power duty cycle (0-100%)
    uint8_t statusByte;           // Aggregated status flags (Bits 0-3)
    
    uint32_t lastWriteTimeMs;     // Millis of last valid Master write
    uint32_t maxPowerStartTimeMs; // Millis when output power first hit >= 95%
    int16_t tempAtMaxPowerStart;  // Temp when output power first hit >= 95%
    
    float integralError;          // PID accumulator
    int16_t previousTemp;         // Previous temp (for Derivative-on-Measurement)
};

// Pins mapping
extern const uint8_t channelPins[8];

// Globals
extern ChannelState channelStates[8];
extern ChannelConfig channelConfigs[8];
extern SemaphoreHandle_t channelMutexes[8];

// Communication tracking
extern volatile uint8_t lastAddressedChannel;

#endif // CONFIG_H
