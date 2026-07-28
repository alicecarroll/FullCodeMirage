#pragma once
#include <stdint.h>
#include <stdbool.h>

struct SensorData;

// Define the scaling used to convert float values to int16_t for transmission
#define SENSOR_SCALE 100.0f


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


// Telemetry/Data IDs. I am not sure that one struct for both slaves is the way to go. It is not used atm I think (Jonathan, 26.7.)
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
    CMD_VPUMP1_OFF       = 0x01,
    CMD_VPUMP1_ON        = 0x02,

    CMD_VPUMP2_OFF       = 0x03,
    CMD_VPUMP2_ON        = 0x04,
    CMD_COMPRESSOR_OFF    = 0x05,
    CMD_COMPRESSOR_ON     = 0x06,

    CMD_OPEN_SHUTTERS   = 0x07,
    CMD_CLOSE_SHUTTERS  = 0x08,

    CMD_STANDBY         = 0x09,
    CMD_MEASUREMENTS    = 0x0A,

    CMD_HEATER_ON       = 0x0B,
    CMD_HEATER_OFF      = 0x0C,

    CMD_OPEN_RELAY1      = 0x0D,
    CMD_CLOSE_RELAY1     = 0x0E,
    CMD_OPEN_RELAY2      = 0x0F,
    CMD_CLOSE_RELAY2     = 0x10,
    CMD_OPEN_RELAY3      = 0x11,
    CMD_CLOSE_RELAY3     = 0x12,
    CMD_OPEN_RELAY4      = 0x13,
    CMD_CLOSE_RELAY4     = 0x14

} SlaveCommands;

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

// Pressure MCU status structure
struct PressureStatusData {
    uint8_t channel_id;
    uint8_t state;          // Standby, Prepressurisation, etc.
    uint8_t error_code;
};

// API
bool slave_send_data(
    SlaveDevice slave,
    SlaveData data_id,
    float value
);

bool slave_send_command(
    SlaveDevice slave,
    SlaveCommands command
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
uint8_t computeCRC8(
    const uint8_t *data, 
    size_t length);
    
bool thermal_test_send_package(  
    SlaveDevice slave, 
    uint8_t channel_id, //0x00- 0x07
    uint8_t mode, //0 bang bang 1 PID 155-255 D_cycle
    int16_t currentTemp, // 5000 = 50,0C
    int16_t target  
); 
bool thermal_test_receive_package(  //when passing variable to this one remember to pass as &channel_id for all pointer
    SlaveDevice slave,
    uint8_t* channel_id, 
    uint8_t* mode,
    uint8_t* power,
    uint16_t* target,
    uint8_t* status,
    uint8_t* error);

bool pressure_send_sensors(
    SlaveDevice slave,
    const SensorData& sensor_data
);

bool pressure_receive_package(
    SlaveDevice slave,
    PressureStatusData* status_out
);

bool pressure_send_command(
    SlaveDevice slave, 
    uint8_t cmd,
    uint8_t info_bit=0
);

// Watchdog background worker task loop declaration
void slave_watchdog_task(void *pvParameters);
