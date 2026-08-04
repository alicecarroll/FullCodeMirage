//Libaries
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"

//other part of the project
#include "config.h"

esp_err_t init_i2c_slave() //initilizes the i2c communication properly with correct pins
{
    esp_err_t err;
    i2c_config_t conf={ //Config values
        .mode=I2C_MODE_SLAVE,
        .sda_io_num=SDA_PIN,
        .scl_io_num= SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // i dunno what these 2 do? 
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave = {
            .addr_10bit_en = 0,
            .slave_addr = SLAVE_ADDR,
            .maximum_speed=400000,
        },
        .clk_flags = 0,
        
    };

    err=i2c_param_config(I2C_PORT, &conf); //Fixes params
    if(err!=ESP_OK){ //checks for error
        return err;
    }

    err=i2c_driver_install(I2C_PORT, conf.mode, rx_buffer_len,tx_buffer_len,0);
    if(err!=ESP_OK){ //checks for error
        return err;
    }

    return ESP_OK;
}




