#pragma once
#include <stdio.h>
#include "driver/gpio.h"
#include "esp_timer.h"

struct controller_data_struct{
    uint8_t mode=155; //0 bangbang 1 pid 155-255 manual
    float temperature=0; 
    float target=30;
    uint8_t duty_cycle=0;  //Used as actual dutyCycle for the switches
    uint8_t max_duty_cycle=0; //Used as max dutycycle and this equals dutyCycle in manual mode
    bool timeout=false;
    gpio_num_t pin=GPIO_NUM_9;
    int64_t last_updated=0; //Used for indvidual switch timeout 

};
//Add controller class

typedef enum{
    global_i2c_timeout=0x01,
    indvidual_i2c_timeout=0x02,
}errorType;



/*
Functions
*/
//Main loop function
void control_loop(void *pvParameters);