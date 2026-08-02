#include <stdio.h>
#include <string.h>
#include <format>
#include <string>
#include <cstdio>
#include <cctype>
#include <forward_list>

#include "Settings.h"
#include "ConnectionLoss.h"
#include "EthernetCom.h"
#include "SDCard.h"
//#include "Humidity.h"
#include "Initialize.h"
#include "Multiplexer.h"
#include "Neopixel.h"
#include "SystemStatus.h"
#include "ErrorStatus.h" // For error logging and status tracking. Before flight it would be good to check that only used errors are defined as bits in ErrorStatus.h
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
 * 3: Measurement
 * 4: Humidity
 */
int mode = 1;//DEFAULT_MODE; // 1

// Watchdog
bool system_ok;

// Watchdog variables for tracking slave pings locally inside the loop
#define SLAVE_WATCHDOG_TIMEOUT_MS 5000
uint32_t last_thermal_ping_time = 0;
uint32_t last_pressure_ping_time = 0;
int16_t thermal_watchdog_count = 0; // Count of subsequent times the thermal slave is reset
int16_t pressure_watchdog_count = 0; // Count of subsequent times the pressure slave is reset

int32_t flightphase = 0; // To track flight phase: 0 = ascend, 1 = float, 2 = descend

static const char *TAG = "main";

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
uint64_t captured_errors = 0;
int32_t LOOP_RETRY_CONNECTION = 10; // Number of loops to wait before retrying connection
int64_t loss_timestamp_us = -1; // To track when connection was lost for termination
int loops_since_connection = 0; // To buffer short con losses for stable running
static SlaveStatus thermal_status = {};
static PressureStatusData pressure_status = {};
static uint8_t active_heater_mask = 0x00;
static bool pressure_system_active = false;
static bool status_update_requested = false;
static bool status_packet_sent_this_loop = false;
static std::string ethernet_command_text;
bool manual_mode_overwrite = false; // To track if manual mode overwrite is active

std::forward_list <int> pressure_slave_commands; // List of commands to send to the pressure slave in the current loop iteration


struct CommandMapping
{
    const char *text;
    SlaveCommands cmd;
};

static const CommandMapping pressure_commands[] =
{
    {"RELAY 1 ON",  CMD_OPEN_RELAY1},
    {"RELAY 1 OFF", CMD_CLOSE_RELAY1},
    {"RELAY 2 ON",  CMD_OPEN_RELAY2},
    {"RELAY 2 OFF", CMD_CLOSE_RELAY2},
    {"RELAY 3 ON",  CMD_OPEN_RELAY3},
    {"RELAY 3 OFF", CMD_CLOSE_RELAY3},
    {"RELAY 4 ON",  CMD_OPEN_RELAY4},
    {"RELAY 4 OFF", CMD_CLOSE_RELAY4},
    {"SHUTTERS OPEN",  CMD_OPEN_SHUTTERS},
    {"SHUTTERS CLOSE", CMD_CLOSE_SHUTTERS},
    {"VALVE OPEN",     CMD_OPEN_SHUTTERS},
    {"VALVE CLOSE",    CMD_CLOSE_SHUTTERS},
    {"PUMP 1 ON", CMD_VPUMP1_ON},
    {"PUMP 1 OFF", CMD_VPUMP1_OFF},
    {"PUMP 2 ON", CMD_VPUMP2_ON},
    {"PUMP 2 OFF", CMD_VPUMP2_OFF},
    {"COMPRESSOR ON", CMD_COMPRESSOR_ON},
    {"COMPRESSOR OFF", CMD_COMPRESSOR_OFF}
};

static void set_heater_bit(uint8_t heater_index, bool enabled)
// Currently, this function is not used to pass information to the thermal mcu. This should be added (Jonathan, 26.7.)
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

    //if (ethernet_command_text == "STATUS" || ethernet_command_text == "REQUEST STATUS" || ethernet_command_text == "REQUEST STATUS UPDATE")
    //{
    //    status_update_requested = true;
    //    ESP_LOGI(TAG, "Status update requested");
    //    return;
    //}

    if (ethernet_command_text == "MODE MEASUREMENTS")
    {
        mode = 3;
        manual_mode_overwrite = true;
        ESP_LOGI(TAG, "Mode changed to MEASUREMENTS");
        return;
    }

    if (ethernet_command_text == "MODE STANDBY")
    {
        mode = 2;
        manual_mode_overwrite = true;
        ESP_LOGI(TAG, "Mode changed to STANDBY");
        return;
    }

    if (ethernet_command_text == "MODE OVERRIDE RESET")
    {
        manual_mode_overwrite = false;
        ESP_LOGI(TAG, "Manual mode overwrite reset");
        return;
    }

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

    for (const auto &entry : pressure_commands)
    {
        if (ethernet_command_text == entry.text)
        {
            ESP_LOGI(TAG, "%s command received", entry.text);
            pressure_slave_commands.push_front(entry.cmd);
            return;
        }
    }
    

    ESP_LOGW(TAG, "Unrecognized ethernet command: %.*s", (int)ethernet_recieve_buf_bytes_read, ethernet_recieve_buf);
}

static esp_err_t send_system_status_packet()
{
    MainSystemStatusPacket system_status_packet  = {};
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
    system_status_packet.pressure_state = pressure_status.state;
    system_status_packet.pressure_error = pressure_status.error_code;
    system_status_packet.pressure_relay_mask = pressure_status.relay_mask & 0x0F;
    system_status_packet.pressure_pump1_pwm = pressure_status.pump1_pwm;
    system_status_packet.pressure_pump2_pwm = pressure_status.pump2_pwm;
    system_status_packet.pressure_compressor_pwm = pressure_status.compressor_pwm;
    system_status_packet.pressure_actuator_mask = pressure_status.relay_mask & 0x80 ? 0x01 : 0x00;
    system_status_packet.pressure_manual_override = pressure_status.manual_override ? 1 : 0;
    system_status_packet.pressure_valve_open = pressure_status.valve_open ? 1 : 0;
    system_status_packet.captured_errors = captured_errors;

    return wiz_send((uint8_t *)&system_status_packet, sizeof(system_status_packet));
}

static void handle_ethernet_send_status(esp_err_t esp_err_status)
{
    switch (esp_err_status)
    {
    case ESP_OK:
        break;

    case ESP_FAIL:
        status_ok = false;
        con_lost = true;
        connection_lost(&con_lost, &loss_timestamp_us);
        break;

    default:
        ESP_LOGI(TAG, "Unexpected return from wiz_send: %d", esp_err_status);
        break;
    }
}

static void handle_ethernet_receive_status(esp_err_t esp_err_status)
{
    switch (esp_err_status)
    {
    case ESP_ERR_NOT_FOUND:
        // No data in buffer = no command from ground
        command_received = false;
        break;

    case ESP_OK:
        // Command received from the gateway / ground GUI.
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

        //if (status_update_requested)
        //{
        //    read_sensors();
        //    buffer_SD_data_csv(&sensor_data);
        //    print_sensor_data(&sensor_data);

        //    esp_err_t esp_err_status_send = send_system_status_packet();
        //    handle_ethernet_send_status(esp_err_status_send);
        //    status_packet_sent_this_loop = (esp_err_status_send == ESP_OK);
        //    status_update_requested = false;
        //}
        break;

    case ESP_FAIL:
        // Error when receiving data
        command_received = false;
        con_lost = true;
        status_ok = false;
        connection_lost(&con_lost, &loss_timestamp_us);
        /*  Put somewhere. For ConnectionLoss. I (Jonathan) do not know where this comes from and what mode 0 should do. I will leave it here for now.
        *   volatile int64_t loss_timestamp_us = -1;
        *   volatile bool    con_lost          = false;
        *   int              mode              = 0;
        *   bool             terminated        = false;
        */
        break;

    default:
        //Unexpected return
        ESP_LOGI(TAG, "Unexpected return from wiz_receive: %d", esp_err_status);
        break;
    }
}

//Values for thermal slave
uint8_t number_channels_thermal=8;  //0-8 depending on the number of switches used
//Variables for thermal under this comment will need to have value assigned in loop. Currently using placeholders (Remove comment when this has changed)
uint8_t thermal_mode=1; //0 bang bang 1 PID 155-255 D_cycle
int16_t thermal_currentTemp=2000; // 5000 = 50,0C  
int16_t thermal_target=5000;
int16_t thermal_watchdog_tolerance=3000; // 1 according to SEDv3. Number of subsequent times where the thermal slave is reset. If reset more than this number of times, the thermal MCU will be considered lost.
bool thermal_mcu_lost=false; // To track if the thermal slave is lost.
//Data recieved from thermal
uint8_t received_channel_id_thermal; //Reason i seperate recieved and sent is to be able to compare later if data packet made it
uint8_t received_mode_thermal;
uint8_t received_power_thermal;
uint16_t received_target_thermal;
uint8_t status_thermal;
uint8_t error_thermal;
uint16_t thermal_current_temperatures[8];
int32_t OUTLET_TEMPERATURE_THRESHOLD = 50; // Threshold for outlet temperature in Celsius


static void comms_thermal_sensor(SensorData &sensor_data, uint32_t current_time_ms){
    uint8_t chosen_channel_id_thermal=0x00; //0x00- 0x07
    //temperature array used for temperature data for thermal
    thermal_current_temperatures[0]=static_cast<uint16_t> (sensor_data.Tt2*100); //thermal expect temp values where 5000=50.00 C
    thermal_current_temperatures[1]=static_cast<uint16_t>(sensor_data.Tp2*100);
    thermal_current_temperatures[2]=static_cast<uint16_t>(sensor_data.Tp3*100);
    thermal_current_temperatures[3]=static_cast<uint16_t>(sensor_data.Tp4*100);
    thermal_current_temperatures[4]=static_cast<uint16_t>(sensor_data.Tp5*100);
    thermal_current_temperatures[5]=static_cast<uint16_t>(sensor_data.Tp6*100);
    thermal_current_temperatures[6]=static_cast<uint16_t>(sensor_data.Tt1*100);
    thermal_current_temperatures[7]=static_cast<uint16_t>(sensor_data.Tt2*100);

    bool thermal_tx_ok = false;

    while (chosen_channel_id_thermal!=(number_channels_thermal-1)){
        thermal_tx_ok = thermal_test_send_package(
            thermal_mcu, 
            chosen_channel_id_thermal, //0x00- 0x07
            thermal_mode, //0 bang bang 1 PID 155-255 D_cycle
            thermal_current_temperatures[chosen_channel_id_thermal], // 5000 = 50,0C 
            thermal_target);
        chosen_channel_id_thermal++;
    }
        
    chosen_channel_id_thermal=0;

    if (thermal_tx_ok)
    {
        if (thermal_test_receive_package(  //when passing variable to this one remember to pass as &channel_id for all pointer
    thermal_mcu,
    &received_channel_id_thermal,
    &received_mode_thermal,
    &received_power_thermal,
    &received_target_thermal,
    &status_thermal,
    &error_thermal))
        {
            //Info recieved from thermal Used for trouble-shooting
            last_thermal_ping_time = current_time_ms;
            ESP_LOGI(TAG, "Feedback from thermal slave - Channel: %u, Mode: %u, Power: %u, Target: %u, Status: %u, Error: %u",
            received_channel_id_thermal, 
            received_mode_thermal,
            received_power_thermal,
            received_target_thermal,
            status_thermal,
            error_thermal);
        }
        else
        {
            ESP_LOGW(TAG, "Thermal MCU failed to respond to state read status query");
        }
    }
    else
    {
        ESP_LOGE_CAPTURED(ERROR_BIT_50, TAG, "I2C Write transmission failed to Thermal MCU");
    }

    //Watchdog reset for thermal
    if ((current_time_ms - last_thermal_ping_time) > SLAVE_WATCHDOG_TIMEOUT_MS)
    {
        ESP_LOGW(TAG, "!!! Watchdog Triggered: Thermal MCU timed out. Resetting device via Pin %d !!!", Thermal_reset_PIN);
        slave_reset(thermal_mcu);
        last_thermal_ping_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        thermal_watchdog_count++;
        if (thermal_watchdog_count >= thermal_watchdog_tolerance)
        {
            thermal_mcu_lost = true;
            ESP_LOGE_CAPTURED(ERROR_BIT_51, TAG, "!!! Thermal MCU considered lost after %d resets. Manual intervention required. !!!", thermal_watchdog_count);
        }
    }
}


//Pressure
bool shutters_open = false; // To track if shutters are open
int16_t pressure_watchdog_tolerance=3000; // 1 according to SEDv3. Number of subsequent times where the pressure slave is reset. If reset more than this number of times, the pressure MCU will be considered lost.
bool pressure_mcu_lost=false; // To track if the pressure slave is lost.

static void comms_pressure_sensor(SensorData &sensor_data, uint32_t current_time_ms)
{
    pressure_send_sensors(pressure_mcu, sensor_data);

    // Send the actual Main-MCU mode, including its numeric value. The
    // pressure MCU uses mode 3/2 to control relay 2/3 automatically.
    pressure_slave_commands.push_front(CMD_SET_MODE);

    // Check if there are any commands to send to the pressure slave
    auto commands = pressure_slave_commands;
    pressure_slave_commands.clear();
    for (int command : commands)
    {
        ESP_LOGI(TAG, "Sending command 0x%02X to Pressure MCU", command);
        // Send the command to the pressure slave
        const bool is_mode_command = command == CMD_SET_MODE;
        const bool sent = is_mode_command
            ? pressure_send_command(pressure_mcu, static_cast<uint8_t>(command), static_cast<uint8_t>(mode))
            : pressure_send_command(pressure_mcu, static_cast<uint8_t>(command));
        if (sent)
        {
            ESP_LOGI(TAG, "Command 0x%02X sent successfully to Pressure MCU", command);
        }
        else
        {
            ESP_LOGE_CAPTURED(ERROR_BIT_52, TAG, "Failed to send command 0x%02X to Pressure MCU", command);
        }
    }

    if (pressure_receive_package(pressure_mcu, &pressure_status))
    {
        last_pressure_ping_time = current_time_ms;
        pressure_system_active = pressure_status.state != 0;
        ESP_LOGI(TAG, "Pressure MCU responded successfully. Channel ID: %d, State: %d, Error Code: %d, "
                 "Relay mask: 0x%02X, manual override: %s",
                 pressure_status.channel_id, pressure_status.state, pressure_status.error_code,
                 pressure_status.relay_mask & 0x0F,
                 pressure_status.manual_override ? "yes" : "no");
    }
    else
    {
        ESP_LOGE_CAPTURED(ERROR_BIT_53, TAG, "Pressure MCU failed to respond to sensor query");
    }

    // Watchdog reset for pressure
    if ((current_time_ms - last_pressure_ping_time) > SLAVE_WATCHDOG_TIMEOUT_MS)
    {
        ESP_LOGW(TAG, "!!! Watchdog Triggered: Pressure MCU timed out. Resetting device via Pin %d !!!", Pressure_reset_PIN);
        slave_reset(pressure_mcu);
        last_pressure_ping_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        pressure_watchdog_count++;
        if (pressure_watchdog_count >= pressure_watchdog_tolerance)
        {
            pressure_mcu_lost = true;
            ESP_LOGE_CAPTURED(ERROR_BIT_54, TAG, "!!! Pressure MCU considered lost after %d resets. Manual intervention required. !!!", pressure_watchdog_count);
        }
    }
}

// ESP-IDF expects main in C
extern "C" void app_main()
{
    init_gpio_pins();
    init_spi();
    wiz_init();
    // Try to establish Ethernet for 5 seconds before proceeding. This is to ensure that the system can still run even if Ethernet is not available.
    TickType_t start_time = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(5000) && !wizphy_getphylink()){
        ESP_LOGW(TAG, "Ethernet link not established yet. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
        

    ESP_LOGI(TAG,"Ethernet link up");
    init_i2c();
    init_uart();
    init_sensors();
    sd_mount();
    printf("Initialization done\n");

    //int8_t s = wizsocket(WIZ_SOCKET, Sn_MR_TCP, LOCAL_PORT, 0);
    //printf("after socket s=%d\n", s);

    //wiz_connect(targetip, REMOTE_PORT);
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
    status_packet_sent_this_loop = false;
    //feed_watchdog(system_ok);
    //wiz_connect(targetip, REMOTE_PORT);


    // Ethernet Receive Block
    if (con_lost && loops_since_connection % LOOP_RETRY_CONNECTION == 0)
    // Try to reestablish connection if lost every LOOP_RETRY_CONNECTION loops
    {
        if (wiz_connect(targetip, REMOTE_PORT)== ESP_OK)
        {
            status_ok = true;
            connection_reestablished(&con_lost, &loss_timestamp_us);
        }
        else
        {
            con_lost = true;
            status_ok = false;
            connection_lost(&con_lost, &loss_timestamp_us);
        }
    }
    // Check for commands
    loops_since_connection++; //Will be reset in handle_ethernet_receive_status if connection is ok.
    esp_err_t esp_err_status_receive = wiz_receive(ethernet_recieve_buf, ethernet_recieve_buf_size, &ethernet_recieve_buf_bytes_read);
    handle_ethernet_receive_status(esp_err_status_receive);
    printf("check for commands done\n");


    // Read I2C Data Block
    read_sensors();
    //buffer_SD_data_binary_single(); //est time: 1.5 ms
    //buffer_SD_data_csv_single();      //est time: 3 ms
    //buffer_SD_data_binary(sensor_data); //4k - est time: 1.5 ms every 8th loop
    buffer_SD_data_csv(&sensor_data);      //4k - est time: 3 ms every 8th loop
    print_sensor_data(&sensor_data);

    // Status Check Block
    if (sensor_data.Pa1 < P_STRATOSPHERE)
    {
        if (flightphase == 0) // If flightphase was in ascent, switch it to float
        {
            flightphase = 1; // Update flightphase to float
            ESP_LOGI(TAG, "Flight phase updated to Float");
        }
        if (!manual_mode_overwrite) // If manual mode overwrite is not active, enter standby mode if pressure is below threshold
        {
            mode = 2; // Enter standby mode if pressure is below threshold
        }
    }
    else {
        if (flightphase == 1) // If flightphase was in float, switch it to descend
        {
            flightphase = 2; // Update flightphase to descend
            ESP_LOGI(TAG, "Flight phase updated to Descend");
        }
        else if (flightphase == 0) // If flightphase is in ascend, the mode should be measurement mode
        {
            if (!manual_mode_overwrite) // If manual mode overwrite is not active, enter measurement mode if pressure is above threshold
            {
                mode = 3; // Enter measurement mode if pressure is above threshold
            }
        }
    }

    // I (Jonathan) skipped the thermal checks, because I do not understand why they are necessary. The thermal slave should be able to handle its own temperature regulation.

    // I (Jonathan) skipped the current checks, because the current PDB has no functioning current sensor. 

    // I (Jonathan) skipped the sensor checks, because I do not know what should happen if a sensor is not working. 
    // For testing it is also good to have the system continue running even if a sensor is not working.

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

            // Bark at pressure subsystem
            if (not pressure_mcu_lost)
            {
                comms_pressure_sensor(sensor_data, current_time_ms);
            }

            //Bark at thermal subsystem
            if (not thermal_mcu_lost)
            {  
                comms_thermal_sensor(sensor_data, current_time_ms);
            }
            // Enter IP when given by ESA
            // Ping ground that status is OK.
            esp_err_t esp_err_status_ping = wiz_ping(targetip, "No command received. Status: OK.");
        }
        break;

    // Standby
    case 2:
        // Deactivate K96
        K96_off();

        //Reset overrides

        //Pressure communication
        if (not pressure_mcu_lost)
        {
            comms_pressure_sensor(sensor_data, current_time_ms);
        }
        
        //Thermal communication
        if (not thermal_mcu_lost)
        {  
        comms_thermal_sensor(sensor_data, current_time_ms);
        }
        break;

    // measurement
    case 3:

        // Activate K96
        K96_on();


        // Pressure check block to see if pressure in chamber is too high 
        //Check if pressure in chamber is below threshold, if so, increase pressure first.
        if (sensor_data.Pp2 < CHAMBER_P_SHUTTER_THRESHOLD)
        {
            //Pressure communication: increase p in chamber.
        } 
        //Open shutters if close
        //Skip data collection to ensure proper pressure in chamber.
        if (shutters_open == false)
        {
            //Pressure communication: open shutters
            shutters_open = true;
        }
        //Check if pressure in chamber is above threshold, if so, take meassurements.
        if (sensor_data.Pp2 < CHAMBER_P_CHAMBER_THRESHOLD)
        {
            //Pressure communication: increase p in chamber.
        }

        // Thermal check block to see if temperatures are out of limits 
        // Check if inlet temperature is above threshold, if so, take meassurements. If not, decrease inlet temperature.
        //if (sensor_data.Tt3 < INLET_TEMPERATURE_THRESHOLD)
        //{
        //    //Thermal communication: increase inlet temperature
        //    break;
        //}

        // Pressure communication block 
        if (not pressure_mcu_lost)
        {
            comms_pressure_sensor(sensor_data, current_time_ms);
        }

        // Thermal communication block
        if (not thermal_mcu_lost)
        {
        comms_thermal_sensor(sensor_data, current_time_ms);
        }

        // Elevation check in terms of pressure
        if (sensor_data.Pa1 < P_STRATOSPHERE)
        {
            if (loops_since_connection > LOOP_WO_CONNECTION) // If connection lost for more than LOOP_WO_CONNECTION loops, enter safe mode
            {
                mode = 2; // Standby
                ESP_LOGE_CAPTURED(ERROR_BIT_55, TAG, "Connection lost for more than %d loops. Entering standby mode.", LOOP_WO_CONNECTION);
                //con_lost = true;
                //connection_lost(&con_lost, &loss_timestamp_us);
                //wiz_ping(targetip, "Connection lost. Entering safe mode."); // Why are we pinging when the connection is lost? This seems counterintuitive. If the connection is lost, how can we ping? This might be a logic error or a misunderstanding of the system's state.
                break;
            }
            //else: high altidude but have connection.
        }
        

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
    //print_sensor_data(&sensor_data);
    
    //char buf[300];
    //int len2 = snprintf(buf, sizeof(buf), "Time: %02u:%02u:%02u, Pa1=%.2f \n", sensor_data.hours, sensor_data.minutes, sensor_data.seconds, sensor_data.Pa1);
    //wiz_send((uint8_t*)buf, len2);

    //printf("ethernet3\n");

    if (!status_packet_sent_this_loop)
    {
        esp_err_t esp_err_status_send = send_system_status_packet();
        handle_ethernet_send_status(esp_err_status_send);
    }
    captured_errors = 0;

    // Delay only the remaining time so the full loop period stays near 1 second.
    TickType_t current_time_stop = xTaskGetTickCount();
    TickType_t elapsed_ticks = current_time_stop - current_time_start;
    TickType_t target_period_ticks = pdMS_TO_TICKS(1000);
    if (elapsed_ticks < target_period_ticks)
    {
        time_loop = static_cast<uint16_t>(target_period_ticks - elapsed_ticks);
    }
    else
    {
        time_loop = 0;
    }
    if (time_loop > 0)
    {
        vTaskDelay(time_loop);
    }
}
