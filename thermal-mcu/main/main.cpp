#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define I2C_SLAVE_NUM           I2C_NUM_0
#define I2C_SLAVE_ADDR          0x42  // Matches Slave_MCU_addr on Master
#define I2C_SLAVE_SDA_IO        GPIO_NUM_11   // Set to your physical S3 SDA Pin
#define I2C_SLAVE_SCL_IO        GPIO_NUM_12   // Set to your physical S3 SCL Pin
#define I2C_SLAVE_RX_BUF_LEN    256
#define I2C_SLAVE_TX_BUF_LEN    256

static const char *TAG = "slave_mcu";

// Opcodes matching Master packet identifiers
#define SLAVE_PACKET_COMMAND 0x02

// System status variables
volatile bool emergency_stop = false;
volatile bool autonomous_mode = false;
volatile bool pressure_system_on = false;
volatile uint8_t heater_mask = 0x00;

volatile uint8_t system_state = 1; // 1 = Manual, 2 = Auto, 99 = Emergency
volatile uint8_t error_code = 0;
volatile uint8_t rolling_ping = 0;

/**
 * Background loop to increment the heartbeat ping value.
 * Proves to the master's watchdog that the scheduler is alive.
 */
void ping_generator_task(void *pvParameters)
{
    while (1) {
        rolling_ping = rolling_ping + 1;
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

/**
 * Continuous I2C listening processing loop
 */
void i2c_slave_rx_task(void *pvParameters)
{
    uint8_t data_buffer[I2C_SLAVE_RX_BUF_LEN];

    while (1) {
        // Blocks natively until data arrives from Master
        int bytes_received = i2c_slave_read_buffer(I2C_SLAVE_NUM, data_buffer, I2C_SLAVE_RX_BUF_LEN, portMAX_DELAY);
        
        if (bytes_received > 0) {
            // Check if it's our target multi-byte configuration state command (3 bytes total)
            if (data_buffer[0] == SLAVE_PACKET_COMMAND && bytes_received >= 3) {
                
                uint8_t flags = data_buffer[1];
                heater_mask = data_buffer[2];

                // Unpack specific bit parameters
                emergency_stop      = (flags & (1 << 0)) != 0;
                autonomous_mode     = (flags & (1 << 1)) != 0;
                pressure_system_on  = (flags & (1 << 2)) != 0;

                // Process systemic outcomes immediately
                if (emergency_stop) {
                    system_state = 99;
                    ESP_LOGE(TAG, "EMERGENCY STATE RECEIVED! All systems halted.");
                } else if (autonomous_mode) {
                    system_state = 2;
                } else {
                    system_state = 1;
                }

                // Clean visual status string summarizing runtime variables
                ESP_LOGI(TAG, "[STATUS] Mode: %s | State Code: %d | Heaters: 0x%02X | Pressure Sys: %s | Ping: %d",
                         emergency_stop ? "EMERGENCY" : (autonomous_mode ? "AUTO" : "MANUAL"),
                         system_state,
                         heater_mask,
                         pressure_system_on ? "ON" : "OFF",
                         rolling_ping);
            }
        }
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Initializing I2C Slave...");
    // Configure I2C Slave Driver Parameters
    i2c_config_t conf_slave = {};
    conf_slave.mode = I2C_MODE_SLAVE;
    conf_slave.sda_io_num = I2C_SLAVE_SDA_IO;
    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE; 
    conf_slave.scl_io_num = I2C_SLAVE_SCL_IO;
    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.slave.addr_10bit_en = 0;
    conf_slave.slave.slave_addr = I2C_SLAVE_ADDR;
    conf_slave.clk_flags = 0;

    i2c_param_config(I2C_SLAVE_NUM, &conf_slave);
    
    // Install driver with Tx/Rx internal memory frame queues
    i2c_driver_install(I2C_SLAVE_NUM, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);

    // Pre-load current status parameters into transmit buffer so they are instantly
    // available when the master runs its read query frame request
    uint8_t response_packet[3] = {system_state, error_code, rolling_ping};
    i2c_slave_write_buffer(I2C_SLAVE_NUM, response_packet, sizeof(response_packet), 0);

    ESP_LOGI(TAG, "I2C Slave driver installed. Starting tasks...");
    // Spawn execution tasks
    xTaskCreate(i2c_slave_rx_task, "i2c_slave_rx", 3072, NULL, 10, NULL);
    xTaskCreate(ping_generator_task, "ping_gen", 2048, NULL, 4, NULL);

    // Loop to keep dynamic slave metrics fresh inside TX pipeline cache
    while (1) {
        uint8_t updated_packet[3] = {system_state, error_code, rolling_ping};
        
        // Wipe old unread queue items to keep the telemetry packet fresh
        i2c_reset_tx_fifo(I2C_SLAVE_NUM);
        i2c_slave_write_buffer(I2C_SLAVE_NUM, updated_packet, sizeof(updated_packet), 0);
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}