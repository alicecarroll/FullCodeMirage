#pragma once
#include <stdint.h>
#include <stdbool.h>


//Crc lookup table


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

//Pointer defines how package will look if data it will only send temp data and mode for each switch
//Command is used to change stuff for specific switches
typedef enum
{
    Data_package=0x00,

    Command_s1  =0x01,
    Command_s2  =0x02,
    Command_s3  =0x03,
    Command_s4  =0x04,
    Command_s5  =0x05,
    Command_s6  =0x06,
    Command_s7  =0x07,
    Command_s8  =0x08,

    Emergency   =0x09


}Packet_pointer;

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
bool slave_send_data(  //These basically says that they will exist as functions
    SlaveDevice slave, //Send data thermal might need to switch name
    uint8_t data_id, // 0x00-0x07
    int16_t data[], //Data and mode MUST have the same number of elements use 5000= 50.00 C /Data cant be negative in current implementation which needs fixing
    uint8_t mode[], //
    int number_switches
);

bool slave_send_command(
    SlaveDevice slave,
    SlaveData data_id,  // pointer 0x01-0x08
    uint8_t mode, //Mode 0 PID , 1 bangbang 155-255 manual
    int16_t data,       //Data and target can be negative
    int16_t target
);

bool slave_update_setting(
    SlaveDevice slave,
    SlaveSetting setting,
    float value
);

bool thermal_test_send_package(  
    SlaveDevice slave, 
    uint8_t channel_id, //0x00- 0x07
    uint8_t mode, //0 bang bang 1 PID 155-255 D_cycle
    int16_t currentTemp, // 5000 = 50,0C
    int16_t target  
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

