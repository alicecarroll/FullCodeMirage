#pragma once
#include <stdio.h>


/*
Structs
*/

//For receiving data for indvidual switches
struct individual_switch_data_rx {
uint8_t regist;  //register/command determines packettype
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
int16_t target; // which target temp is it
uint8_t status; //status/error codes
uint8_t padding; // future use
//Crc8 will be added to send buffer later (it is not in this struct intentionally)
};



/*
Enums
*/
//For the register/command byte in all packages
typedef enum{
    packet_type_indvidual_switch=0x01

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
