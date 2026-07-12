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

bool loop_exp = true;
uint16_t time_loop;

/* Modes
 * 1: Test Loop
 * 2: Standby
 * 3: Meassurement
 * 4: Humidity
 */
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

static void set_heater_bit(uint8_t heater_index, bool enabled)
{
    if (heater_index >= 8)
    {
        return;
    }

    uint8_t mask = static_cast<uint8_t>(1U << heater_index);
    if (enabled)
    {
        active_heater_mask |= mask;
    }
    else
    {
        active_heater_mask &= static_cast<uint8_t>(~mask);
    }
}

static std::string to_upper_copy(const uint8_t *data, size_t length)
{
    std::string text;
    text.reserve(length);

    for (size_t index = 0; index < length; ++index)
    {
        text.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(data[index]))));
    }

    return text;
}

static void trim_in_place(std::string &text)
{
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front())))
    {
        text.erase(text.begin());
    }

    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back())))
    {
        text.pop_back();
    }
}

//Pressure
bool shutters_open = false; // To track if shutters are open

// Watchdog variables for tracking slave pings locally inside the loop
#define SLAVE_WATCHDOG_TIMEOUT_MS 5000
uint32_t last_thermal_ping_time = 0;
uint32_t last_pressure_ping_time = 0;

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
    init_gpio_pins();
    init_spi();
    init_i2c();
    init_uart();
    init_sensors();
    wiz_init();
    sd_mount();
    printf("Initialization done\n");

    int8_t s = wizsocket(WIZ_SOCKET, Sn_MR_TCP, LOCAL_PORT, 0);
    printf("after socket s=%d\n", s);

    wiz_connect(targetip, REMOTE_PORT);
    setSn_IR(WIZ_SOCKET, Sn_IR_CON);

    wiz_ensure_connected(targetip, REMOTE_PORT);
    wiz_ping(targetip, "h\n");

    // Set baseline slave watchdog timestamps here, AFTER Ethernet blocks!
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    last_thermal_ping_time = current_time;
    last_pressure_ping_time = current_time;
    active_heater_mask = 0x01;

    while (loop_exp == true)
    {
        loop();
    }
}

void loop()
{
    // Common actions
    TickType_t current_time_start = xTaskGetTickCount();
    uint32_t current_time_ms = current_time_start * portTICK_PERIOD_MS;
    //feed_watchdog(system_ok);
    //wiz_connect(targetip, REMOTE_PORT);

    // Check for commands
    loops_since_connection++; //Will be reset in handle_ethernet_data if connection is ok
    esp_err_t esp_err_status = wiz_receive(ethernet_recieve_buf, ethernet_recieve_buf_size, &ethernet_recieve_buf_bytes_read);
    handle_ethernet_data(esp_err_status);

    printf("check for commands done\n");
    // Collect I2C data
    if (mode != 1)
    {
        read_sensors();
        //buffer_SD_data_binary_single(); //est time: 1.5 ms
        //buffer_SD_data_csv_single();      //est time: 3 ms
        //buffer_SD_data_binary(sensor_data); //4k - est time: 1.5 ms every 8th loop
        buffer_SD_data_csv(&sensor_data);      //4k - est time: 3 ms every 8th loop
        print_sensor_data(&sensor_data);
    }

    pressure_system_active = (mode == 1 || mode == 3);

    bool thermal_autonomous_mode = (mode == 3);
    bool thermal_tx_ok = slave_send_complex_state(
        thermal_mcu,
        false,
        thermal_autonomous_mode,
        pressure_system_active,
        active_heater_mask);

    if (thermal_tx_ok)
    {
        if (slave_read_status(thermal_mcu, &thermal_status))
        {
            last_thermal_ping_time = current_time_ms;
        }
        else
        {
            ESP_LOGW(TAG, "Thermal MCU failed to respond to state read status query");
        }
    }
    else
    {
        ESP_LOGE(TAG, "I2C Write transmission failed to Thermal MCU");
    }

    if ((current_time_ms - last_thermal_ping_time) > SLAVE_WATCHDOG_TIMEOUT_MS)
    {
        ESP_LOGE(TAG, "!!! Watchdog Triggered: Thermal MCU timed out. Resetting device via Pin %d !!!", Thermal_reset_PIN);
        slave_reset(thermal_mcu);
        last_thermal_ping_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }

    // Mode dependent actions
    printf("mode %d\n", mode);
    switch (mode)
    {
    // Test loop
    case 1:
        {
            // Repeated workflow for Pressure MCU (Keeping lines cleanly separated)
            //slave_send_complex_state(pressure_mcu, false, false, true, 0x00);
            //SlaveStatus pressure_status;
            //if (slave_read_status(pressure_mcu, &pressure_status)) {
            //    last_pressure_ping_time = current_time_ms;
            //}
            //if ((current_time_ms - last_pressure_ping_time) > SLAVE_WATCHDOG_TIMEOUT_MS) {
            //    ESP_LOGE(TAG, "!!! Watchdog Triggered: Pressure MCU timed out. Resetting device via Pin %d !!!", Preassure_reset_PIN);
            //    slave_reset(pressure_mcu);
            //    last_pressure_ping_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            //}

            // Bark at subsystems
            // Enter IP when given by ESA
            // Ping ground that status is OK.
            esp_err_status = wiz_ping(targetip, "No command received. Status: OK."); 
        }
        break;

    // Standby
    case 2:
        //Reset overrides

        //Pressure communication
        
        //Thermal communication

        break;

    // Measurement
    case 3:
        // Humidity check here if any 

        // Elevation check in terms of pressure
        if (sensor_data.Pa1 < P_STRATOSPHERE)
        {
            if (loops_since_connection > LOOP_WO_CONNECTION) // If connection lost for more than LOOP_WO_CONNECTION loops, enter safe mode
            {
                mode = 2; // Standby
                con_lost = true;
                connection_lost(&con_lost, &loss_timestamp_us);
                
                wiz_ping(targetip, "Connection lost. Entering safe mode.");
                break;
            }
            //else: high altidude but have connection.
        }
        //Check if pressure in chamber is below threshold, if so, increase pressure first.
        if (sensor_data.Pp2 < CHAMBER_P_SHUTTER_THRESHOLD)
        {
            //Pressure communication: increase p in chamber.
            break;
        } 
        //Open shutters if close
        //Skip data collection to ensure proper pressure in chamber.
        if (shutters_open == false)
        {
            //Pressure communication: open shutters
            shutters_open = true;
            break;
        }
        //Check if pressure in chamber is above threshold, if so, take meassurements.
        if (sensor_data.Pp2 < CHAMBER_P_CHAMBER_THRESHOLD)
        {
            //Pressure communication: increase p in chamber.
            break;
        }
        // Check if inlet temperature is above threshold, if so, take meassurements. If not, decrease inlet temperature.
        //if (sensor_data.Tt3 < INLET_TEMPERATURE_THRESHOLD)
        //{
        //    //Thermal communication: increase inlet temperature
        //    break;
        //}
        // Take meassurements!!!
        read_k96();
        //buffer_SD_data_csv(sensor_data); 
        break;

    // Leave for now as stated by Anna
    // Humidity
    case 4:
        
        ESP_LOGI(TAG, "Humidity loop not implemented");
        break;

    default:
        mode = 1;
        //std::string msg = "Unknown mode. Returning to test loop.";
        //wiz_send(msg, sizeof(msg));
        break;
    }
    printf("mode %d\n", mode);

    //Transmit data over E-Link
    //uint8_t ethernet_send_buf[sizeof(sensor_data)];
    //printf("ethernet1\n");
    //char msg[] = "Hello from ESP32!";
    //wizsend(WIZ_SOCKET, (uint8_t*)msg, strlen(msg));
    //wizsend(WIZ_SOCKET, (uint8_t*)sensorout, strlen(msg));
    read_sensors();
    buffer_SD_data_csv(&sensor_data);

    //char status_message[100]; 
    //int len = snprintf(status_message, sizeof(status_message),
    //                "Status: %d. Command received: %d. Mode: %d.",
    //                status_ok, command_received, mode);
//
    //if (len > 0 && len < (int)sizeof(status_message)) {
    //    memcpy(ethernet_send_buf, status_message, len); // copy only the real message length
    //    wiz_send(ethernet_send_buf, len);                // send only that many bytes
    //} else {
    //    // formatting error or truncation — handle appropriately
    //}
    print_sensor_data(&sensor_data);
    
    //char buf[300];
    //int len2 = snprintf(buf, sizeof(buf), "Time: %02u:%02u:%02u, Pa1=%.2f \n", sensor_data.hours, sensor_data.minutes, sensor_data.seconds, sensor_data.Pa1);
    //wiz_send((uint8_t*)buf, len2);

    //printf("ethernet3\n");

    MainSystemStatusPacket system_status_packet = {};
    system_status_packet.sensor_data = sensor_data;
    system_status_packet.operating_mode = static_cast<uint8_t>(mode);
    system_status_packet.command_received = command_received ? 1 : 0;
    system_status_packet.connection_lost = con_lost ? 1 : 0;
    system_status_packet.status_ok = status_ok ? 1 : 0;
    system_status_packet.pressure_system_on = pressure_system_active ? 1 : 0;
    system_status_packet.heater_mask = active_heater_mask;
    system_status_packet.thermal_online = thermal_status.online ? 1 : 0;
    system_status_packet.thermal_state = thermal_status.state;
    system_status_packet.thermal_error = thermal_status.error;

    wiz_send((uint8_t *)&system_status_packet, sizeof(system_status_packet));

    // Wait until loop has taken 100 ms.
    TickType_t current_time_stop = xTaskGetTickCount();
    //printf("ethernet4\n");
    time_loop = pdMS_TO_TICKS(1000); //- (current_time_stop - current_time_start);
    //printf("time_loop %d\n", time_loop);
    if (time_loop > 0)
    {
        //printf("wait\n");
        vTaskDelay(time_loop);
    }
    //printf("loop end\n");
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void handle_ethernet_data(esp_err_t esp_err_status)
{
    switch (esp_err_status)
    {
    case ESP_ERR_NOT_FOUND:
        // No data in buffer = no command from ground
        command_received = false;
        break;
    
    case ESP_OK:
        // Command recieved
        if (mode == 1)
        {
            mode = 2;
        }
        handle_command(); 
        command_received = true;
        con_lost = false;
        status_ok = true;
        loops_since_connection = 0; // Reset connection loss buffer
        connection_reestablished(&con_lost, &loss_timestamp_us); 
        break;

    case ESP_FAIL:
        // Error when receiving data
        // What to do here?
         break;
    
    default:
        //Unexpected return
        ESP_LOGI(TAG, "Unexpected return from wiz_receive: %d", esp_err_status);
        break;
    }
}

void handle_command()
{
    ethernet_command_text = to_upper_copy(ethernet_recieve_buf, ethernet_recieve_buf_bytes_read);

    if (ethernet_command_text.empty())
    {
        ESP_LOGW(TAG, "Received empty ethernet command");
        return;
    }

    trim_in_place(ethernet_command_text);

    int heater_index = 0;

    if (ethernet_command_text == "HEATER ON")
    {
        set_heater_bit(0, true);
        ESP_LOGI(TAG, "Heater 1 turned ON. Mask now 0x%02X", active_heater_mask);
        return;
    }

    if (ethernet_command_text == "HEATER OFF")
    {
        set_heater_bit(0, false);
        ESP_LOGI(TAG, "Heater 1 turned OFF. Mask now 0x%02X", active_heater_mask);
        return;
    }

    if (sscanf(ethernet_command_text.c_str(), "HEATER ON %d", &heater_index) == 1)
    {
        if (heater_index >= 1 && heater_index <= 8)
        {
            set_heater_bit(static_cast<uint8_t>(heater_index - 1), true);
            ESP_LOGI(TAG, "Heater %d turned ON. Mask now 0x%02X", heater_index, active_heater_mask);
        }
        else
        {
            ESP_LOGW(TAG, "Invalid heater index in command: %s", ethernet_command_text.c_str());
        }
        return;
    }

    if (sscanf(ethernet_command_text.c_str(), "HEATER OFF %d", &heater_index) == 1)
    {
        if (heater_index >= 1 && heater_index <= 8)
        {
            set_heater_bit(static_cast<uint8_t>(heater_index - 1), false);
            ESP_LOGI(TAG, "Heater %d turned OFF. Mask now 0x%02X", heater_index, active_heater_mask);
        }
        else
        {
            ESP_LOGW(TAG, "Invalid heater index in command: %s", ethernet_command_text.c_str());
        }
        return;
    }

    if (ethernet_command_text == "HEATER ALL ON")
    {
        active_heater_mask = 0xFF;
        ESP_LOGI(TAG, "All heaters turned ON");
        return;
    }

    if (ethernet_command_text == "HEATER ALL OFF")
    {
        active_heater_mask = 0x00;
        ESP_LOGI(TAG, "All heaters turned OFF");
        return;
    }

    ESP_LOGW(TAG, "Unrecognized ethernet command: %.*s", (int)ethernet_recieve_buf_bytes_read, ethernet_recieve_buf);
}