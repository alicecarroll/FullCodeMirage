#pragma once

#include <stdint.h>

#include "read_sensors.h"

#pragma pack(push, 1)
typedef struct
{
    SensorData sensor_data;

    uint8_t operating_mode;
    uint8_t command_received;
    uint8_t connection_lost;
    uint8_t status_ok;
    uint8_t pressure_system_on;
    uint8_t heater_mask;
    uint8_t thermal_online;
    uint8_t thermal_state;
    uint8_t thermal_error;
} MainSystemStatusPacket;
#pragma pack(pop)
