#pragma once

#include <stdint.h>

#include "read_sensors.h"

#pragma pack(push, 1)
typedef struct {
    SensorData sensor_data;
    uint8_t operating_mode;
    uint8_t command_received;
    uint8_t connection_lost;
    uint8_t status_ok;
    uint8_t pressure_system_on;
    uint16_t heater_mask;
    uint8_t thermal_online;
    uint8_t thermal_state;
    uint8_t thermal_error;
    uint8_t pressure_state;
    uint8_t pressure_error;
    uint8_t pressure_relay_mask;
    uint8_t pressure_pump1_pwm;
    uint8_t pressure_pump2_pwm;
    uint8_t pressure_compressor_pwm;
    uint8_t pressure_actuator_mask;
    uint8_t pressure_manual_override;
    uint8_t pressure_valve_open;
    uint64_t captured_errors;
} MainSystemStatusPacket;
#pragma pack(pop)

static_assert(sizeof(MainSystemStatusPacket) == 206, "Groundstation packet size changed");
