#pragma once
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "driver/i2c.h"
/*
Structs
*/

//For receiving data for indvidual switches
struct individual_switch_data_rx {
    uint8_t regist;  //register/command determines packettype 0x01== default
    uint8_t switchID; //which switch
    uint8_t mode; //mode 0 Hysteris 1 PID 155-255 manual with duty cycle 0-100%
    int16_t temperature; //current temperature
    int16_t target;  //target temperature
    uint8_t crc8; //checksum
};

//For sending data for indvidual switches
struct individual_switch_data_tx{
uint8_t switchID;   //which switch 
uint8_t mode;   //Mode for switch  
uint8_t D_cycle; //WHich duty cycle is it using
float target; // which target temp is it
uint8_t status; //status/error codes
uint8_t global_mode; //Global Mode is if its in emergency mode or not ie mode not specific to a switch
//Crc8 will be added to send buffer later (it is not in this struct intentionally)
};

struct i2c_data_evt{
    uint8_t data[8];
    size_t length;
};



/*
Enums
*/
//For the register/command byte in all packages
typedef enum{
    packet_type_indvidual_switch=0x01,
    packet_stop_all=0x15,
    packet_resume_all=0x31
    
}packet_types_t;



/*
functions
*/ 
uint8_t computeCRC8(
    const uint8_t *data, 
    size_t length);

bool data_unpack_indvidual_switch(
    const uint8_t *dataBuffer, 
    individual_switch_data_rx *packet);

bool data_pack_indvidual_switch(
    const individual_switch_data_tx *packet,
    uint8_t *data // data should be 1 byte more than packet due to crc8
);


void i2c_loop_send_task(void *pvParameters);

void i2c_loop_task(void *pvParameters);

