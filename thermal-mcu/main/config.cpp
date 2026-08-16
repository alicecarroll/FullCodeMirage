//Libaries
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c_slave.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//other part of the project
#include "config.h"
#include "i2c_com.h"
#include "control.h"
//variables 
i2c_slave_dev_handle_t slave_handle;
//FreeRTOS Queue

QueueHandle_t dataQueue=nullptr;
QueueHandle_t dataQueue_slave_rx=nullptr;// for slave receive callback function
QueueHandle_t dataQueue_slave_tx=nullptr;

//Task handles
TaskHandle_t i2c_send_task_handle;
TaskHandle_t control_loop_task_handle;
void startup(){
    BaseType_t taskResult;
    //Initilizes data queues
    dataQueue=xQueueCreate(1,sizeof(individual_switch_data_rx));
    dataQueue_slave_rx=xQueueCreate(1,sizeof(i2c_data_evt));
    dataQueue_slave_tx=xQueueCreate(1,sizeof(i2c_data_evt));


    //FreeRTOS tasks IF There are crashes check the memory allocation of the FREERTOS tasks ie change the allocated memory in xtaskcreate
    //I2c send task
    taskResult= xTaskCreate(
        i2c_loop_send_task, //Function name
        "I2C Send task", //Task name
        4096, //Allocated memory
        NULL, //Parameters passed to function
        11, //Task priority default 0-24
        NULL //TaskHandle used to interact with task from outside (kill msg ping etc)
    );

    if (taskResult != pdPASS) {
        ESP_LOGE("TaskCreation", "Failed to create task! I2C send task");
    }

    //I2C receive task
    taskResult=xTaskCreate(
        i2c_loop_task, //receivetask
        "I2C receive task",
        8192,
        NULL,
        18,
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

// esp_err_t init_i2c_slave() //initilizes the i2c communication properly with correct pins
// {
//     esp_err_t err;
//     i2c_config_t conf={ //Config values
//         .mode=I2C_MODE_SLAVE,
//         .sda_io_num=SDA_PIN,
//         .scl_io_num= SCL_PIN,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE, // i dunno what these 2 do? 
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .slave = {
//             .addr_10bit_en = 0,
//             .slave_addr = SLAVE_ADDR,
//             .maximum_speed=400000,
//         },
//         .clk_flags = 0,
//     };

//     err=i2c_param_config(I2C_PORT, &conf); //Fixes params
//     if(err!=ESP_OK){ //checks for error
//         return err;
//     }

//     err=i2c_driver_install(I2C_PORT, conf.mode, rx_buffer_len,tx_buffer_len,0);
//     if(err!=ESP_OK){ //checks for error
//         return err;
//     }

//     return ESP_OK;
// }

//Initilizes i2c communication using the new protocols 
esp_err_t init_i2c_slave(){
    esp_err_t err;
    i2c_slave_config_t conf{
        
        .i2c_port=I2C_PORT,
        .sda_io_num=SDA_PIN,
        .scl_io_num=SCL_PIN,
        .clk_source=I2C_CLK_SRC_DEFAULT,
        .send_buf_depth=tx_buffer_len,
        .receive_buf_depth=rx_buffer_len,
        .slave_addr=SLAVE_ADDR,
        .addr_bit_len=I2C_ADDR_BIT_LEN_7,
        .intr_priority=2,
        .flags={},
        
    };

    err=i2c_new_slave_device(&conf,&slave_handle);//Creating slave with pins and buffers

    if(err!=ESP_OK)
    { 
        return err;
    }

    i2c_slave_event_callbacks_t rec={
        .on_request =i2c_slave_on_request_cb,
        .on_receive = i2c_slave_on_receive_cb
    };

    err=i2c_slave_register_event_callbacks(slave_handle,&rec, NULL); //Creating event callbacks onrecieve and onrequest
    if(err!=ESP_OK){
        return err;
    }

    return ESP_OK;

}


//Initilize for the pins used to control switches
esp_err_t init_write_pins()
{
    esp_err_t err;
    
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
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    

    err=gpio_config(&io_conf);
    if(err!=ESP_OK){
        return err;
    }

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




