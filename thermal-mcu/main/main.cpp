#include <stdio.h>

#include "i2c_com.h"
#include "config.h"
#include "control.h"
#include "main.h"

extern "C" void app_main(void)
{
    ESP_LOGI("Initilization:", "Starting...");
    if(init_i2c_slave()!=ESP_OK){ //i2c initilization
        ESP_LOGE("Initilization:", "I2C initilization failed");
    }

    if(init_write_pins()!=ESP_OK){ //Write pin initilization
        ESP_LOGE("Initilization:", "Write pin initilization failed");
    }

    startup(); //Queue and paralell task inintilization

        // Heater 3 ON
    gpio_set_level(inlet1_PIN, 1);   // set HIGH = ON
    // gpio_set_level(controllerData[2].pin, 1);   // if Heater 3 is the 3rd switch/channel
    ESP_LOGI("Initialization", "Finished start up");
    
}

/*
extern "C" void app_main(void){
    init_write_pins();
    //void pinMode(8, GPIO_MODE_OUTPUT);
    while (1)
    {
        loop();
    }
// the loop function runs over and over again forever
    
}

void loop() {
        gpio_set_level(inlet1_PIN, 1);  // turn the LED on (HIGH is the voltage level)
        vTaskDelay(pdMS_TO_TICKS(1000)); 
        //gpio_set_level(inlet1_PIN, 0);  // turn the LED on (HIGH is the voltage level)
        //vTaskDelay(pdMS_TO_TICKS(1000));                      // wait for a second
    }

*/