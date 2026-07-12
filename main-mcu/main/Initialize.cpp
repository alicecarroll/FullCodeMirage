#include "Settings.h"
#include "led_strip.h"
#include "led_strip_rmt.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "Multiplexer.h"
#include "initialize.h"
#include "read_sensors.h"
#include "esp_log.h"
#include <stdio.h>



// Initialize pins
void init_gpio_pins()
{
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask =
        (1ULL << Watchdog_PIN) |
        (1ULL << Thermal_reset_PIN) |
        (1ULL << Preassure_reset_PIN) |
        (1ULL << K96_EN_PIN) |
        (1ULL << Reset_WIZ_PIN) |
        (1ULL << CS_WIZ_PIN)|
        (1ULL << CS_SD_PIN);

    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // Safe startup states
    gpio_set_level(Watchdog_PIN, 0); 
    gpio_set_level(Thermal_reset_PIN, 1);   
    gpio_set_level(Preassure_reset_PIN, 1); 
    gpio_set_level(Reset_WIZ_PIN, 1);       
    gpio_set_level(CS_WIZ_PIN, 1);
    gpio_set_level(CS_SD_PIN, 1);   // deselect SD card by default
    gpio_set_level(K96_EN_PIN, 0);

    gpio_reset_pin(Thermal_reset_PIN);
    gpio_set_direction(Thermal_reset_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(Thermal_reset_PIN, 1); 

    gpio_reset_pin(Preassure_reset_PIN);
    gpio_set_direction(Preassure_reset_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(Preassure_reset_PIN, 1);
}

// Initialize SPI
spi_device_handle_t SD_handle;
spi_device_handle_t WIZ_handle;

esp_err_t init_spi()
{
    // configure bus
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = SPI_MOSI_PIN;
    buscfg.miso_io_num = SPI_MISO_PIN;
    buscfg.sclk_io_num = SPI_clk_PIN;
    buscfg.quadwp_io_num = -1; // Disabling quadpins
    buscfg.quadhd_io_num = -1; // Disabling quadpins

    esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK)
    {
        ESP_LOGE("Init", "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    // Add SD-card
    //spi_device_interface_config_t SD_cfg = {};
    //SD_cfg.clock_speed_hz = SD_clk_spd_hz;
    //SD_cfg.mode = 0;                 // SPI mode 0
    //SD_cfg.spics_io_num = CS_SD_PIN; // CS pin for SD card
    //SD_cfg.queue_size = SD_queue_size;

    //err = spi_bus_add_device(SPI2_HOST, &SD_cfg, &SD_handle);
    //if (err != ESP_OK)
    //{
    //    ESP_LOGE("Init", "spi_bus_add_device(SD) failed: %s", esp_err_to_name(err));
    //    return err;
    //}

    // Add Ethernet
    spi_device_interface_config_t WIZ_cfg = {};
    WIZ_cfg.clock_speed_hz = WIZ_clk_spd_hz;
    WIZ_cfg.mode = 0;                  // SPI mode 0
    WIZ_cfg.spics_io_num = -1;         // ioLibrary keeps CS asserted across transactions
    WIZ_cfg.queue_size = ethernet_queue_size;

    err = spi_bus_add_device(SPI2_HOST, &WIZ_cfg, &WIZ_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("Init", "spi_bus_add_device(WIZ) failed: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

// initialize I2c
esp_err_t init_i2c()
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA;
    conf.scl_io_num = I2C_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;

    esp_err_t err = i2c_param_config(I2C_master, &conf);
    if (err != ESP_OK)
    {
        ESP_LOGE("Init", "i2c_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(I2C_master, conf.mode, 0, 0, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE("Init", "i2c_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

// Uart initialize
void init_uart()
{
    uart_config_t uart_config = {};

    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_2;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    // Apply UART configuration
    uart_param_config(UART_PORT, &uart_config);

    // Set UART pins
    uart_set_pin(
        UART_PORT,
        K96_TX_PIN,
        K96_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE);

    // Install UART driver
    uart_driver_install(
        UART_PORT,
        1024, // RX buffer
        0,    // TX buffer
        0,    // Event queue size
        NULL,
        0);
}

// Starting sensors that needs to be initialized
static void init_sht45_sensor(uint8_t channel)
{
    sel_mux_channel(channel);

    uint8_t cmd = 0xFD; // High precision measurement command

    // 1. Send the command to wake the sensor up and trigger a measurement
    esp_err_t err = i2c_master_write_to_device(
        I2C_master,
        SHT45_addr,
        &cmd,
        1,
        pdMS_TO_TICKS(20));

    if (err != ESP_OK)
    {
        ESP_LOGW("Init", "SHT45 init failed on MUX Ch %d (Mask: 0x%02x): %s", 
                 channel, (1 << channel), esp_err_to_name(err));
    }
}

// initioalize MS5803 sensors

static void ms5803_reset()
{
    uint8_t cmd = 0x1E;

    esp_err_t err = i2c_master_write_to_device(
        I2C_master,
        MS5803_addr,
        &cmd,
        1,
        pdMS_TO_TICKS(20));

    if (err != ESP_OK)
    {
        printf("MS5803 reset failed\n");
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}

static void init_ms5803(uint8_t channel, MS5803_Calibration *cal, MS5803Model model)
{
    sel_mux_channel(channel);

    ms5803_reset();

    uint8_t data[2];

    cal -> model = model;

    for (uint8_t i = 0; i < 6; i++)
    {
        uint8_t cmd = 0xA2 + (i * 2);

        esp_err_t err = i2c_master_write_read_device(
            I2C_master,
            MS5803_addr,
            &cmd,
            1,
            data,
            2,
            pdMS_TO_TICKS(20));

        if (err != ESP_OK)
        {
            ESP_LOGW("Init", "MS5803 init failed at coeff %u: %s", i, esp_err_to_name(err));
            return;
        }

        cal->C[i + 1] =
            (data[0] << 8) | data[1];
    }
}

void init_sensors()
{
    init_sht45_sensor(multiplex_Ambient);
    //init_sht45_sensor(multiplex_Tp4_Pp1_Tp5_Pp2); There is no SHT45 connected on the K96 board
    init_sht45_sensor(multiplex_Outlet);
    init_ms5803(multiplex_Ambient, &pa1_cal, MS5803Model::MS5803_01BA);
    init_ms5803(multiplex_Tp4_Pp1_Tp5_Pp2, &pp2_cal, MS5803Model::MS5803_14BA);
}

// Initialize neopixel
void init_neopixel()
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = Neo_PIN,
        .max_leds = NEOPIXEL_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format =
            LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false}};

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags = {
            .with_dma = false}};

    esp_err_t err =
        led_strip_new_rmt_device(
            &strip_config,
            &rmt_config,
            &s_strip);

    if (err != ESP_OK)
    {
        ESP_LOGE(
            "NeoPixel",
            "Initialization failed: %s",
            esp_err_to_name(err));

        return;
    }

    led_strip_clear(s_strip);

    ESP_LOGI(
        "NeoPixel",
        "NeoPixel initialized");
}
