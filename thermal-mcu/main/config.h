#pragma once 
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c_slave.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//Pin definitions
#define SDA_PIN GPIO_NUM_11 //formerly 13
#define SCL_PIN GPIO_NUM_12 //formerly 14

//The names for these can be switched around. They are for consistency with the circuit board
#define SD_Card_PIN GPIO_NUM_5
#define pressure_ch_PIN GPIO_NUM_6
#define outlet_PIN GPIO_NUM_7
#define inlet1_PIN GPIO_NUM_8
#define inlet2_PIN GPIO_NUM_9
#define plt1_PIN GPIO_NUM_10
#define plt2_PIN GPIO_NUM_17
#define backup_PIN GPIO_NUM_18
//i2c
#define SLAVE_ADDR 0x10
#define I2C_PORT I2C_NUM_0

#define rx_buffer_len 128
#define tx_buffer_len 128
#define number_switches 8

#define control_timeout_us 5000000 //5 seconds

//Global variables
extern QueueHandle_t dataQueue;
extern QueueHandle_t dataQueue_slave_rx;
extern QueueHandle_t dataQueue_slave_tx;
extern TaskHandle_t i2c_send_task_handle;
extern TaskHandle_t control_loop_task_handle;
extern i2c_slave_dev_handle_t slave_handle;




/*
Functions
*/
esp_err_t init_i2c_slave();
esp_err_t init_write_pins();
void startup();