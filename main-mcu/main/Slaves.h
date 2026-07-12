#pragma once
#include <stdint.h>
#include <stdbool.h>

// Slave devices
typedef enum
{
    thermal_mcu,
    pressure_mcu

} SlaveDevice;

// Packet types
typedef enum
{
    Slave_packet_data     = 0x01,
    Slave_packet_command  = 0x02,
    Slave_packet_setting  = 0x03

} PacketType;

// Telemetry/Data IDs
typedef enum
{
    // Vacuum pump temperatures
    DATA_TP1                = 0x01,
    DATA_TP2                = 0x02,

    // Compressor temperature
    DATA_TP3                = 0x03,

    // Pipe pump/pump
    DATA_TP6                = 0x04,
    DATA_PP3                = 0x05,

    // Pipe pump/compressor
    DATA_TP4                = 0x06,
    DATA_PP1                = 0x07,

    // Ambient
    DATA_PA1                = 0x08,
    DATA_TA1                = 0x09,
    DATA_TA2                = 0x0A,
    DATA_HA1                = 0x0B,

    // Measurement chamber
    DATA_TP5                = 0x0C,
    DATA_PP2                = 0x0D,

    // Outlet + inlet
    DATA_TT1                = 0x0E,
    DATA_TT2                = 0x0F,
    DATA_TT3                = 0x10

} SlaveData;

// Commands
typedef enum
{
    CMD_PUMPS_OFF       = 0x01,
    CMD_PUMPS_ON        = 0x02,

    CMD_OPEN_SHUTTERS   = 0x03,
    CMD_CLOSE_SHUTTERS  = 0x04,

    CMD_STANDBY         = 0x05,
    CMD_MEASUREMENTS    = 0x06,

    CMD_HEATER_ON       = 0x07,
    CMD_HEATER_OFF      = 0x08

} SlaveCommand;

// Settings
typedef enum
{
    SET_CHAMBER_TEMP      = 0x01,
    SET_CHAMBER_PRESSURE  = 0x02

} SlaveSetting;

// Slave MCU status
typedef struct
{
    bool online;
    uint8_t state;
    uint8_t error;
} SlaveStatus;

// API
bool slave_send_data(
    SlaveDevice slave,
    SlaveData data_id,
    float value
);

bool slave_send_command(
    SlaveDevice slave,
    SlaveCommand command
);

// Combined structural state command for full system synchronization
bool slave_send_complex_state(
    SlaveDevice slave,
    bool emergency_stop,
    bool autonomous_mode,
    bool pressure_system_on,
    uint8_t heater_mask
);

// Fixed: Unified with enum signature from implementation
bool slave_update_setting(
    SlaveDevice slave,
    SlaveSetting setting,
    float value
);

bool slave_read_status(
    SlaveDevice slave,
    SlaveStatus* status
);

void slave_reset(
    SlaveDevice slave
);

// Watchdog background worker task loop declaration
void slave_watchdog_task(void *pvParameters);