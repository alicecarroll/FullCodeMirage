#include <stdio.h>

#include "i2c_com.h"
#include "config.h"
#include "control.h"

extern "C" void app_main(void)
{
    if(init_i2c_slave()!=ESP_OK){ //i2c initilization
        ESP_LOGE("Initilization:", "I2C initilization failed");
    }

    if(init_write_pins()!=ESP_OK){ //Write pin initilization
        ESP_LOGE("Initilization:", "Write pin initilization failed");
    }

    startup(); //Queue and paralell task inintilization


}