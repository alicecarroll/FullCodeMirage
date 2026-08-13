#pragma once
#include <stdio.h>
#include "driver/gpio.h"

struct controller_data_struct{
    uint8_t mode=155; //0 bangbang 1 pid 155-255 manual
    float temperature; 
    float target;
    uint8_t duty_cycle=0;  //Used as actual dutyCycle for the switches
    uint8_t max_duty_cycle=0; //Used as max dutycycle and this equals dutyCycle in manual mode
    bool timeout=false;
    gpio_num_t pin;
    int64_t last_updated; //Used for indvidual switch timeout 


};
//Add controller class