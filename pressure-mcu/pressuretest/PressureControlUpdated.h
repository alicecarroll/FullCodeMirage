#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    PRESSURE_STANDBY,
    PRESSURE_PREPRESSURISATION,
    PRESSURE_AIR_EXCHANGE,
    PRESSURE_ERROR

} PressureState;


typedef struct
{
    PressureState state;

    float chamber_pressure;
    float compressor_inlet_pressure;  // pressure on the suction side of
                                       // the compressor, between the
                                       // vacuum pumps and the compressor

    uint8_t error;

} PressureStatus;


// Setup
void pressure_init();


// Called every loop
void pressure_update();


// Commands from Main MCU
void pressure_cmd_standby();

void pressure_cmd_measurements();


// Settings from Main MCU
void pressure_set_target_pressure(
    float pressure);

// Compressor inlet pressure that ends the prepressurisation phase
void pressure_set_compressor_inlet_upper_limit(
    float pressure);

// Compressor inlet pressure that ends the air exchange phase
void pressure_set_compressor_inlet_lower_limit(
    float pressure);


// Status reporting
PressureStatus pressure_get_status();

// Is the pressure system actively running a phase (not in standby)?
bool pressure_system_is_on();


// ------------------------------------------------------------------
// PDB relays — four on/off signals to MOSFETs on the power
// distribution board, from connector pins 1, 4, 5, 28.
// Plain independent on/off, no sequencing.
// ------------------------------------------------------------------

void pdb_relay1_on();
void pdb_relay1_off();

void pdb_relay2_on();
void pdb_relay2_off();

void pdb_relay3_on();
void pdb_relay3_off();

void pdb_relay4_on();
void pdb_relay4_off();
