#pragma once

#include <stdint.h>

// Canonical Main MCU <-> Pressure MCU protocol. All multi-byte values are
// encoded explicitly by the sender; frames are byte-oriented and CRC-protected.
enum PressurePacketType : uint8_t {
    PRESSURE_PACKET_SENSORS  = 0x01,
    PRESSURE_PACKET_COMMANDS = 0x02,
    PRESSURE_PACKET_SETTINGS = 0x03,
};

enum PressureCommand : uint8_t {
    PRESSURE_CMD_PUMP1_OFF   = 0x01,
    PRESSURE_CMD_PUMP1_ON    = 0x02,
    PRESSURE_CMD_PUMP2_OFF   = 0x03,
    PRESSURE_CMD_PUMP2_ON    = 0x04,
    PRESSURE_CMD_COMPRESSOR_OFF = 0x05,
    PRESSURE_CMD_COMPRESSOR_ON  = 0x06,
    PRESSURE_CMD_VALVE_OPEN   = 0x07,
    PRESSURE_CMD_VALVE_CLOSE  = 0x08,
    // 0x09..0x0C were legacy mode/heater commands and are intentionally
    // reserved. The supported command IDs remain wire-compatible.
    PRESSURE_CMD_SET_MODE     = 0x15,
    PRESSURE_CMD_RELAY1_ON    = 0x0D,
    PRESSURE_CMD_RELAY1_OFF   = 0x0E,
    PRESSURE_CMD_RELAY2_ON    = 0x0F,
    PRESSURE_CMD_RELAY2_OFF   = 0x10,
    PRESSURE_CMD_RELAY3_ON    = 0x11,
    PRESSURE_CMD_RELAY3_OFF   = 0x12,
    PRESSURE_CMD_RELAY4_ON    = 0x13,
    PRESSURE_CMD_RELAY4_OFF   = 0x14,
    PRESSURE_CMD_PUMP1_PWM    = 0x16,
    PRESSURE_CMD_PUMP2_PWM    = 0x17,
    PRESSURE_CMD_COMPRESSOR_PWM = 0x18,
    PRESSURE_CMD_START_PRESSURISATION = 0x19,
    PRESSURE_CMD_STOP_PRESSURISATION = 0x1A,
    PRESSURE_CMD_FLUSH_CHAMBER = 0x1B,
    PRESSURE_CMD_SAFE_SHUTDOWN = 0x1C,
};

enum PressureMode : uint8_t {
    PRESSURE_MODE_TEST_LOOP = 1,
    PRESSURE_MODE_STANDBY = 2,
    PRESSURE_MODE_MEASUREMENTS = 3,
    PRESSURE_MODE_HUMIDITY = 4,
};

enum : uint8_t {
    PRESSURE_COMMAND_FRAME_SIZE = 4,
    PRESSURE_SENSOR_FRAME_SIZE = 16,
    PRESSURE_STATUS_FRAME_SIZE = 8,
};
