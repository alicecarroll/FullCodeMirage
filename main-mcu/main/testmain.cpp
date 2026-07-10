
#include <stdio.h>
#include <string.h>

#include "Settings.h"
#include "ConnectionLoss.h"
#include "EthernetCom.h"
#include "Humidity.h"
#include "Initialize.h"
#include "Multiplexer.h"
#include "Neopixel.h"
#include "read_sensors.h"
#include "SDCard.h"
#include "Uart.h"
#include "watchdog.h"
#include "main.h"
#include "w5500.h"

bool loop_exp = true;



/* Modes
 * 1: Test Loop
 * 2: Standby
 * 3: Meassurement
 * 4: Humidity
 */
int mode = 2;

// Watchdog
bool system_ok = true;

// Ethernet
uint8_t ethernet_recieve_buf[ETHERNET_BUF_SIZE] = {0};
size_t ethernet_recieve_buf_size = ETHERNET_BUF_SIZE;
size_t ethernet_recieve_buf_bytes_read = 0;
uint8_t main_ip[4] = WIZ_IP;
uint16_t portw = WIZ_SOCKET;
uint8_t targetip[4] = {192, 168, 0, 3};

//bool command_received = false;

bool con_lost = false; // To track connection status
const char *message = "p";
//bool connection_lost = false; // To track connection status
//int64_t loss_timestamp_us = -1; // To track when connection was lost for termination
//int loops_since_connection = 0; // To buffer short con losses for stable running

//Pressure
bool shutters_open = false; // To track if shutters are open

static const char *TAG = "main";

/*  Put somewhere. For ConnectionLoss
 *   volatile int64_t loss_timestamp_us = -1;
 *   volatile bool    con_lost          = false;
 *   int              mode              = 0;
 *   bool             terminated        = false;
 */

// ESP-IDF expects main in C
extern "C" void app_main()
{
    //esp_log_level_set("*", ESP_LOG_WARN);

    printf("Starting application\n");

    // This should be one func in Initialize instead?
    init_gpio_pins();
    printf("GPIO initialized\n");

    esp_err_t spi_err = init_spi();
    if (spi_err != ESP_OK)
    {
        printf("SPI init failed: %s\n", esp_err_to_name(spi_err));
        return;
    }
    printf("SPI initialized\n");

    esp_err_t wiz_err = wiz_init();
    if (wiz_err != ESP_OK)
    {
        printf("W5500 init failed: %s\n", esp_err_to_name(wiz_err));
        con_lost = true;
    }
    else
    {
        printf("W5500 initialized\n");
        uint8_t read_back = WIZCHIP_READ(MR);
        printf("WIZCHIP_READ(MR) returned: 0x%02X\n", read_back);

        uint8_t sn_cr = getSn_CR(WIZ_SOCKET);
        uint8_t sn_sr = getSn_SR(WIZ_SOCKET);
        printf("Socket %u CR=0x%02X SR=0x%02X\n", WIZ_SOCKET, sn_cr, sn_sr);
    }

    esp_err_t i2c_err = init_i2c();
    if (i2c_err != ESP_OK)
    {
        printf("I2C init failed: %s\n", esp_err_to_name(i2c_err));
        return;
    }
    printf("I2C initialized\n");

    //init_uart_k96();
    init_sensors();
    printf("Sensors initialized\n");

    //spi_device_interface_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT(); 
    //slot_config.spics_io_num = CS_SD_PIN; 
    //esp_err_t sd_err = sd_mount(); //sd_mount(&slot_config)
    //if (sd_err != ESP_OK)
    //{
    //    printf("SD card mount failed: %s\n", esp_err_to_name(sd_err));
    //}
    //else
    //{
    //    printf("SD card mounted\n");
    //}
    printf("before socket\n");
    fflush(stdout);

    int8_t s = wizsocket(WIZ_SOCKET, Sn_MR_TCP, 5000, SF_BROAD_BLOCK);

    printf("after socket s=%d\n", s);
    fflush(stdout);
    esp_err_t err = wiz_connect(targetip, 5000);
    printf("connect result: %d\n", err);
    wiz_connect(targetip, 5000);
    const char *message = "p";
    wiz_ping(targetip, message);
    printf("connection?: %d\n", wiz_ensure_connected(targetip, 5000));
    if(wiz_ensure_connected(targetip, 5000) == ESP_OK)
    {
        con_lost=false;
        printf("connected\n");
        wiz_ping(targetip, message);

    }
    else
    {
        con_lost=true;
        printf("Not connected\n");
    }
//
    while (loop_exp == true)
    {
        loop();
    }
    
}

void loop()
{
    // Common actions
    //feed_watchdog(system_ok);
    printf("Hello!\n");
    printf("Connection lost: %d\n", con_lost ? 1 : 0);
    

    wiz_ping(targetip, message);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // Check for commands
    //loops_since_connection++; //Will be reset in handle_ethernet_data if connection is ok
    //esp_err_status = wiz_receive(ethernet_recieve_buf, ethernet_recieve_buf_size, &ethernet_recieve_buf_bytes_read);
    //handle_ethernet_data(esp_err_status);


    // Collect I2C data
    if (mode != 1)
    {
        //printf("Reading sensors...\n");
        //read_sensors();
        //print_sensor_data(&sensor_data);
        // buffer_SD_data_csv(&sensor_data);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }



    // Mode dependent actions
    switch (mode)
    {
    // Test loop
    case 1:
        //read_sensors();
        
        printf("Hello!\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // Bark at subsystems
        //!!!!!!!!!!

        //!!!!!
        // Enter IP when given by SSC Space
        // Ping ground that status is OK.
        //esp_err_status = wiz_ping(uint8_t *target_ip, "No command recieved. Status: OK.");

        break;
    
    }
}