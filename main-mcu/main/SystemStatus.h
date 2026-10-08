#pragma once

#include <stddef.h>
#include <stdint.h>

#include "read_sensors.h"

enum MainControllerState : uint8_t {
    MAIN_CONTROLLER_BOOTING = 0,
    MAIN_CONTROLLER_READY = 1,
    MAIN_CONTROLLER_SAFE_SHUTDOWN = 2,
    MAIN_CONTROLLER_RESTARTING = 3,
};

#pragma pack(push, 1)
typedef struct {
    SensorData sensor_data;
    uint8_t operating_mode;
    uint8_t command_received;
    uint8_t connection_lost;
    uint8_t status_ok;
    uint8_t pressure_system_on;
    uint8_t k96_on;
    uint8_t thermal_heater_duty_cycle[8];  
    uint8_t thermal_online;
    uint8_t thermal_state; 
    uint8_t thermal_error[8]; //errorarray 
    uint8_t pressure_state; 
    uint8_t pressure_error;
    uint8_t pressure_relay_mask;
    uint8_t pressure_pump1_pwm;
    uint8_t pressure_pump2_pwm;
    uint8_t pressure_compressor_pwm;
    uint8_t pressure_manual_override;
    uint8_t pressure_valve_open;
    uint8_t onboard_logging;
    uint8_t storage_free_pct;
    uint8_t controller_state;
    CapturedErrors captured_errors;
} MainSystemStatusPacket;
#pragma pack(pop)

extern MainSystemStatusPacket system_status_packet;

// Keep this wire-size contract synchronized with groundstation_gateway.py and listener.py.
static constexpr size_t MAIN_SYSTEM_STATUS_PACKET_SIZE = 230;
static_assert(sizeof(MainSystemStatusPacket) == MAIN_SYSTEM_STATUS_PACKET_SIZE, "Groundstation packet size changed");
