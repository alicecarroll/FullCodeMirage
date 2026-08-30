//Libaries
//#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//other part of the project
#include "config.h"
#include "i2c_com.h"
#include "control.h"
//variables 
//FreeRTOS Queue

QueueHandle_t dataQueue=nullptr;
QueueHandle_t dataQueue_slave_tx=nullptr;

//Task handles
TaskHandle_t i2c_send_task_handle;
TaskHandle_t control_loop_task_handle;
void startup(){
    BaseType_t taskResult;
    //Initilizes data queues
    dataQueue=xQueueCreate(number_switches,sizeof(individual_switch_data_rx));
    dataQueue_slave_tx=xQueueCreate(1,sizeof(i2c_data_evt));


    //FreeRTOS tasks IF There are crashes check the memory allocation of the FREERTOS tasks ie change the allocated memory in xtaskcreate


    //I2C receive task
    taskResult=xTaskCreate(
        i2c_loop_task, //receivetask
        "I2C receive task",
        8192,
        NULL,
        11,
        NULL
    );

    if (taskResult != pdPASS) {
        ESP_LOGE("TaskCreation", "Failed to create task! I2C receive task");
    }
    
    //Control loop task
    taskResult=xTaskCreate(
        control_loop,
        "Control Loop task",
        16384,
        NULL,
        9,
        &control_loop_task_handle
    );
    if (taskResult != pdPASS) {
        ESP_LOGE("TaskCreation", "Failed to create task! ControlTask");
    }

    

}

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




//Initilize for the pins used to control switches
esp_err_t init_write_pins()
{
    esp_err_t err;
    
    // Reset pins first to detach them from default peripheral matrix signals (SPI/JTAG/UART)
    gpio_reset_pin(SD_Card_PIN);
    gpio_reset_pin(pressure_ch_PIN);
    gpio_reset_pin(outlet_PIN);
    gpio_reset_pin(inlet1_PIN);
    gpio_reset_pin(inlet2_PIN);
    gpio_reset_pin(plt1_PIN);
    gpio_reset_pin(plt2_PIN);
    gpio_reset_pin(backup_PIN);

    gpio_config_t io_conf = { //This should do initilization for all 8 pins
        .pin_bit_mask = (1ULL << SD_Card_PIN) | 
        (1ULL << pressure_ch_PIN)|
        (1ULL << outlet_PIN)|
        (1ULL << inlet1_PIN)|
        (1ULL << inlet2_PIN)|
        (1ULL << plt1_PIN)|
        (1ULL << plt2_PIN)|
        (1ULL << backup_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    err=gpio_config(&io_conf);
    if(err!=ESP_OK){
        return err;
    }

    gpio_set_direction(SD_Card_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(pressure_ch_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(outlet_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(inlet1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(inlet2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(plt1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(plt2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(backup_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(SD_Card_PIN,0);
    gpio_set_level(pressure_ch_PIN,0);
    gpio_set_level(outlet_PIN,0);
    gpio_set_level(inlet1_PIN,0);
    gpio_set_level(inlet2_PIN,0);
    gpio_set_level(plt1_PIN,0);
    gpio_set_level(plt2_PIN,0);
    gpio_set_level(backup_PIN,0);

    return ESP_OK;
}




