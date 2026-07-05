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

bool loop_exp = true;

static void print_sensor_data(const SensorData *data)
{
    if (data == nullptr)
    {
        return;
    }

    printf("Time: %02u:%02u:%02u\n", data->hours, data->minutes, data->seconds);
    printf("Temps: Tp1=%.2f C Tp2=%.2f C Tp3=%.2f C Tp4=%.2f C Tp5=%.2f C Tp6=%.2f C Tt1=%.2f C Tt2=%.2f C Tt3=%.2f C Ta1=%.2f C Ta2=%.2f C\n",
           data->Tp1, data->Tp2, data->Tp3, data->Tp4, data->Tp5, data->Tp6,
           data->Tt1, data->Tt2, data->Tt3, data->Ta1, data->Ta2);
    printf("Pressures: Pa1=%.2f Pp1=%.2f Pp2=%.2f Pp3=%.2f\n",
           data->Pa1, data->Pp1, data->Pp2, data->Pp3);
    printf("Ambient: Ha1=%.2f %% RH\n", data->Ha1);
    printf("K96: CO2=%.2f ppm pressure=%.2f hPa temp=%.2f C humidity=%.2f RH error=%u\n",
           data->K96_CO2, data->K96_pressure, data->K96_temperature,
           data->K96_humidity, data->K96_error);
}

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

//bool command_received = false;

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
    esp_log_level_set("*", ESP_LOG_WARN);

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
    esp_err_t sd_err = sd_mount(); //sd_mount(&slot_config)
    if (sd_err != ESP_OK)
    {
        printf("SD card mount failed: %s\n", esp_err_to_name(sd_err));
    }
    else
    {
        printf("SD card mounted\n");
    }

    while (loop_exp == true)
    {
        loop();
    }
    
}

void loop()
{
    // Common actions
    //feed_watchdog(system_ok);
    //printf("Hello!\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // Check for commands
    //loops_since_connection++; //Will be reset in handle_ethernet_data if connection is ok
    //esp_err_status = wiz_receive(ethernet_recieve_buf, ethernet_recieve_buf_size, &ethernet_recieve_buf_bytes_read);
    //handle_ethernet_data(esp_err_status);


    // Collect I2C data
    if (mode != 1)
    {
        printf("Reading sensors...\n");
        read_sensors();
        print_sensor_data(&sensor_data);
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
// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

