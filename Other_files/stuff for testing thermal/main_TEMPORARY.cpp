#include <stdio.h>
#include <string.h>

#include "Settings.h"
#include "ConnectionLoss.h"
#include "EthernetCom.h"
#include "esp_timer.h"
#include "Humidity.h"
#include "Initialize.h"
#include "Multiplexer.h"
#include "Neopixel.h"
#include "read_sensors.h"
#include "SDCard.h"
#include "Uart.h"
#include "watchdog.h"

#include "slave_thermal.h"

///This implementation Is not what will be on final. 
// On final we need to be careful such that 2 parts of the program dont write to i2c at the same time
// This program is strictly for testing if we can send packages properly

uint64_t currentTime = esp_timer_get_time();
uint64_t lastTime=0; 
uint64_t interval_us = 1000000; //1 seconds


SlaveDevice slave=thermal_mcu;
uint8_t channel_id=0x00; //0x00- 0x07
uint8_t mode=155; //0 bang bang 1 PID 155-255 D_cycle
int16_t currentTemp=2000; //replace with actual sensor data??? 
int16_t target=2000;

void loop(){

    if(currentTime-lastTime>=interval_us){
        thermal_test_send_package(slave, channel_id, mode, currentTemp,target);
        
        if(channel_id==7){
            channel_id=0x00;
        }
        else{
            channel_id+=1; 
        }
        lastTime=currentTime;
    }
    currentTime=esp_timer_get_time();

}