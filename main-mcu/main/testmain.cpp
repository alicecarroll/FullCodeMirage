
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
#include <stdio.h>
#include <string.h>
#include <format>
#include <string>
#include <cstdio>
#include <cctype>

#include "Settings.h"
#include "ConnectionLoss.h"
#include "EthernetCom.h"
#include "SDCard.h"
//#include "Humidity.h"
#include "Initialize.h"
#include "Multiplexer.h"
#include "Neopixel.h"
#include "SystemStatus.h"
#include "read_sensors.h"

#include "Uart.h"
#include "watchdog.h"
#include "main.h"
#include "w5500.h"
#include "Slaves.h" 

int mode = 1;//DEFAULT_MODE; // 1

// Watchdog
bool system_ok;

// Ethernet
uint8_t ethernet_recieve_buf[ETHERNET_BUF_SIZE] = {0};
size_t ethernet_recieve_buf_size = ETHERNET_BUF_SIZE;
size_t ethernet_recieve_buf_bytes_read = 0;
uint8_t main_ip[4] = WIZ_IP;
uint16_t portw = WIZ_SOCKET;
uint8_t targetip[4] = {192, 168, 0, 3};

bool command_received = false;

bool con_lost = false; // To track connection status
bool status_ok = true;
int64_t loss_timestamp_us = -1; // To track when connection was lost for termination
int loops_since_connection = 0; // To buffer short con losses for stable running
static SlaveStatus thermal_status = {};
static uint8_t active_heater_mask = 0x00;
static bool pressure_system_active = false;

static std::string ethernet_command_text;
static const char *TAG = "main";

extern "C" void app_main(){
//gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);
init_gpio_pins();
//gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);
init_spi();
init_i2c();
init_uart();
init_sensors();
wiz_init();

while(!wizphy_getphylink())
{
    vTaskDelay(pdMS_TO_TICKS(500));
}

ESP_LOGI(TAG,"Ethernet link up");

vTaskDelay(pdMS_TO_TICKS(1000));

wiz_ensure_connected(targetip, REMOTE_PORT);
sd_mount();
}