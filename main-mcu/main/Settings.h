#pragma once
#include "driver/gpio.h"
#include <stdint.h>
//#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/i2c.h"
//#include "uart.h"
//#include "gpio.h" 
//#include "spi_common.h"
//#include "ledc.h"
#include "driver/spi_master.h"
//#include "driver/sdspi_host.h" // in main.cpp I am configuring the SD card using sdspi_device_config_t, so I need to include this header for that struct

// PINS definition
#define Watchdog_PIN         GPIO_NUM_2
#define Thermal_reset_PIN    GPIO_NUM_3
#define Preassure_reset_PIN  GPIO_NUM_27
#define K96_TX_PIN           GPIO_NUM_0
#define K96_RX_PIN           GPIO_NUM_1
#define K96_EN_PIN           GPIO_NUM_6
#define Neo_PIN              GPIO_NUM_7
#define Reset_WIZ_PIN        GPIO_NUM_8
#define CS_SD_PIN            GPIO_NUM_9
#define CS_WIZ_PIN           GPIO_NUM_10
#define SPI_MOSI_PIN         GPIO_NUM_5
#define SPI_MISO_PIN         GPIO_NUM_15
#define SPI_clk_PIN          GPIO_NUM_4
#define I2C_SDA              GPIO_NUM_25
#define I2C_SCL              GPIO_NUM_26

// Uart K96 channel
#define UART_K96 UART_NUM_1
#define UART_buff_size 256

// I2c adresses and multiplexer channels
#define multiplex_addr 0x70      // TCA9548
#define sensor_Tp1_addr 0x48     // TMP1075 for temperature in pump 1
#define sensor_Tp2_addr 0x48     // TMP1075 for temperature in pump 2
#define sensor_Tp3_addr 0x48     // TMP1075 for temperature in compressor
#define sensor_Tp4_Pp1_addr 0x28 // ABP2 for preassure and temperature pipe going between pumps
#define sensor_Tp6_Pp3_addr 0x28 // ABP2 for preassure and temperature in pipe going into compressor
#define sensor_Tp5_addr 0x44     // SHT45 for temperature in meassurment chamber
#define sensor_Pp2_addr 0x77     // MS5803 for preassure in meassurment chamber
#define sensor_Ta1_addr 0x48     // TMP117 temperature in ambient
#define sensor_Pa1_addr 0x77     // MS5803 for ambient preassure
#define sensor_Ha1_Ta2_addr 0x44 // SHT45 for humidity and temperature in ambient
#define RTC_addr 0x68            // DS3231 for real-time clk
#define sensor_Tt1_addr 0x44     // SHT45 for temperature in outlet air
#define sensor_Tt2_addr 0x48     // TMP1075 for temperature in SD-card
#define sensor_Tt3_addr 0x44     // SHT45 for temperature in inlet air

#define TMP1075_addr 0x48 // TMP1075 for temperature in pump 1, pump 2 and compressor, can be same since they are on different channels
#define ABP2_addr 0x28    // ABP2 for preassure and temperature in pipe going between pumps and into compressor, can be same since they are on different channels
#define TMP117_addr 0x48  // TMP117 for temperature in ambient, can be same since they are on different channels
#define SHT45_addr 0x44   // SHT45 for temperature and humidity in ambient, temperature in meassurment chamber, temperature in outlet air and inlet air, can be same since they are on different channels
#define MS5803_addr 0x77  // MS5803 for preassure in ambient and meassurment chamber, can be same since they are on different channels

#define I2C_master I2C_NUM_0
#define I2C_FREQ_HZ 400000 // 400kHz I2C clock speed

#define multiplex_Tp1_devT 0//0x01        // Channel 0 on multiplexer for temperature in pump 1 and Temperature MCU
#define multiplex_RTC_Tp2 1//0x02         // Channel 1 on multiplexer for RTC and temperature in pump 2
#define multiplex_Ambient 2//0x04         // Channel 2 on multiplexer for ambient sensors (temperature and humidity, preassure)
#define multiplex_Tp4_Pp1_Tp5_Pp2 3//0x08 // Channel 3 on multiplexer for temperature and preassure in pipe going into compressor and meassurment chamber
#define multiplex_Tp3 4//0x10             // Channel 4 on multiplexer for temperature in compressor
#define multiplex_Outlet 5//0x20       // Channel 5 on multiplexer for temperature in outlet air
#define multiplex_Tp6_Pp3 6//0x40         // Channel 6 on multiplexer for SHT45 that is unsure what it will be used for
#define multiplex_Tt3_devP 7//0x80        // Channel 7 on multiplexer for temperature in inlet air and preassure MCU

// SPI settings
extern spi_device_handle_t SD_handle;
extern spi_device_handle_t WIZ_handle;

#define ethernet_queue_size 7
#define WIZ_clk_spd_hz 1000000 // 1MHz

// Ethernet (WIZ)
#define WIZ_SOCKET 0      // TCP
#define WIZ_PING_SOCKET 1 // ICMP uses socket 1, independent of TCP
// Socket 0 14KB TX + 14KB RX internal buffer
// Socket 1  2KB TX +  2KB RX internal buffer

#define ETHERNET_BUF_SIZE 128 // 64 probly enough //datatype should be: size_t
#define LOOP_WO_CONNECTION 50 // Number of loops to wait before entering safe mode when connection is lost

// Ethernet Network configuration
#define WIZ_MAC {0x00, 0x08, 0xDC, 0x01, 0x02, 0x03}
#define WIZ_IP {192, 168, 0, 2}
#define WIZ_GATEWAY {192, 168, 1, 1}
#define WIZ_SUBNET {255, 255, 255, 0}
#define WIZ_DNS {8, 8, 8, 8}

#define SD_queue_size 7
#define SD_clk_spd_hz 1000000 // 1MHz

// SD Card settings
#define SD_MOUNT_POINT "/sdcard" // Used to build path name
#define SD_MAX_PATH_LEN 64

#define SD_BUFFER_SIZE 8 * 512 // In bytes, min 512 bytes

#define SENSOR_READING_SIZE sizeof(SensorData)
#define READINGS_PER_BUFFER (SD_BUFFER_SIZE / SENSOR_READING_SIZE)


// Uart
#define UART_PORT UART_NUM_1

// Humidity settings !!!PLACEHOLDER NUMBERS!!!
#define HUMIDITY_TARGET 80
#define TEMPERATURE_TARGET 20

// Conneciton lost timer
//  30 min in microseconds (esp_timer runs on us)
#define TERMINATION_TIMEOUT (30LL * 60LL * 1000000LL)
// Neopixel
//extern led_strip_handle_t s_strip;
#define NEOPIXEL_COUNT 3 // 3 Neopixels available
#define DEFAULT_MODE 1

// Sensor value thresholds
#define P_STRATOSPHERE 40 // Stratosphere limit, TBD
#define CHAMBER_P_SHUTTER_THRESHOLD // Pressure threshold for opening shutter to meassurment chamber, TBD
#define CHAMBER_P_CHAMBER_THRESHOLD // Pressure threshold for taking measurements,TBD
#define INLET_TEMPERATURE_THRESHOLD // Temperature threshold for taking measurements, TBD