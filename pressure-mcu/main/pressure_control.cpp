#include "main.h"
#include "pressure_hardware.h"
#include "../../protocol/pressure_protocol.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include <string.h>

namespace {
PressureStatus status = {};
float external_sensors[7] = {};
bool external_sensors_valid = false;
bool manual_pump1 = false, manual_pump2 = false, manual_compressor = false, manual_valve = false;
bool relay_manual = false;
uint8_t applied_mode = 0;
float target_pressure = 3.75f; //in pressure chamber
float inlet_upper = 1.5f, inlet_lower = 1.0f; //inlet upper and lower boundaries for compressor inlet
//constexpr float FLUSH_COMPLETE_PRESSURE_BAR = 0.05f; //Why so low?
constexpr uint8_t ERR_NONE = 0, ERR_CHAMBER_SENSOR = 1, ERR_INLET_SENSOR = 2;

TickType_t measurement_time_start;
TickType_t flushstep_start;
TickType_t flushstep_stop;
TickType_t flushticks;

float measure_time = 20.0f; //in s
float flushsum = 0.0f; // sums up exchanged air
float V = 0.075; //75 ml estimated chamber volume
float Qout = 1.5; // l/min based on compressor out flow rate measured in test for lower end of pressure range
float flushtarget = 0.225; // flushtarget = -Setup.V*np.log(0.05) which means that 95% of the air should be exchanged

bool compressed = false;

uint8_t clamp_pwm(uint8_t pwm) { return pwm > 100 ? 100 : pwm; }
void set_pump1(uint8_t pwm) { pwm = clamp_pwm(pwm); pressure_pump1_set(pwm); status.pump1_pwm = pwm; }
void set_pump2(uint8_t pwm) { pwm = clamp_pwm(pwm); pressure_pump2_set(pwm); status.pump2_pwm = pwm; }
void set_compressor(uint8_t pwm) { pwm = clamp_pwm(pwm); pressure_compressor_set(pwm); status.compressor_pwm = pwm; }
void set_valve(bool open) { pressure_valve_set(open); status.valve_open = open; }
void set_relay(uint8_t relay, bool on) {
    pressure_relay_set(relay, on);
    const uint8_t bit = static_cast<uint8_t>(1U << (relay - 1));
    status.relay_mask = on ? status.relay_mask | bit : status.relay_mask & ~bit;
}
void clear_overrides() { manual_pump1 = manual_pump2 = manual_compressor = manual_valve = false; relay_manual = false; }
void set_measurement_outputs() {
    set_pump1(100); set_pump2(100); set_compressor(0);
}
void stop_pressure_train(bool open_valve) {
    clear_overrides();
    set_pump1(0);
    set_pump2(0);
    set_compressor(0);
    set_valve(open_valve);
}
void safe_off() {
    if (!manual_pump1) set_pump1(0);
    if (!manual_pump2) set_pump2(0);
    if (!manual_compressor) set_compressor(0);
    if (!manual_valve) set_valve(false);
}
void mode_changed(uint8_t mode) {
    if (mode == applied_mode) return;
    applied_mode = mode;
    clear_overrides();
    status.error = ERR_NONE;
    if (mode == PRESSURE_MODE_MEASUREMENTS) {
        status.state = PRESSURE_PREPRESSURISATION;
        set_relay(2, true);
        set_relay(3, true);
        set_measurement_outputs();
    } else if (mode == PRESSURE_MODE_STANDBY) {
        status.state = PRESSURE_STANDBY;
        set_relay(2, false);
        set_relay(3, false);
        safe_off();
    }
}
}

void pressure_init() {
    pressure_hardware_init();
    status.state = PRESSURE_STANDBY;
    status.error = ERR_NONE;
    set_pump1(0); set_pump2(0); set_compressor(0); set_valve(false);
    for (uint8_t relay = 1; relay <= 4; ++relay) set_relay(relay, false);
}

void pressure_update_external_sensors(const float sensors[7]) {
    memcpy(external_sensors, sensors, sizeof(external_sensors));
    external_sensors_valid = true;
    status.chamber_pressure = sensors[2];
    status.ambient_pressure = sensors[3];
    status.compressor_inlet_pressure = (sensors[1]+status.ambient_pressure);
    
}

void adjust_pressure_target(){
    //upper limit of compressor inlet pressure should depend on ambient pressure
    if ((status.ambient_pressure < 0.9) and (status.ambient_pressure>=0.7)){
        inlet_upper = 1.5;
        inlet_lower = 1.1;
    }
    else if ((status.ambient_pressure < 0.7) and (status.ambient_pressure>=0.5)){
        inlet_upper = 1.3;
        inlet_lower = 1.0;
    }
    else if ((status.ambient_pressure < 0.5) and (status.ambient_pressure>=0.2)){
        inlet_upper = 1.1;
        inlet_lower = 0.9;
    }
    else if (status.ambient_pressure < 0.2){
        inlet_upper = 1.1;
        inlet_lower = 0.9;
    }
}

void pressure_execute_command(uint8_t command, uint8_t info) {
    switch (command) {
        // Legacy ON commands do not carry a PWM value. They must mean full
        // speed; explicit partial duty cycles use the *_PWM commands below.
        case PRESSURE_CMD_PUMP1_ON: manual_pump1 = true; set_pump1(100); break;
        case PRESSURE_CMD_PUMP1_OFF: manual_pump1 = true; set_pump1(0); break;
        case PRESSURE_CMD_PUMP2_ON: manual_pump2 = true; set_pump2(100); break;
        case PRESSURE_CMD_PUMP2_OFF: manual_pump2 = true; set_pump2(0); break;
        case PRESSURE_CMD_COMPRESSOR_ON: manual_compressor = true; set_compressor(100); break;
        case PRESSURE_CMD_COMPRESSOR_OFF: manual_compressor = true; set_compressor(0); break;
        case PRESSURE_CMD_PUMP1_PWM: manual_pump1 = true; set_pump1(info > 100 ? 100 : info); break;
        case PRESSURE_CMD_PUMP2_PWM: manual_pump2 = true; set_pump2(info > 100 ? 100 : info); break;
        case PRESSURE_CMD_COMPRESSOR_PWM: manual_compressor = true; set_compressor(info > 100 ? 100 : info); break;
        case PRESSURE_CMD_VALVE_OPEN: manual_valve = true; set_valve(true); break;
        case PRESSURE_CMD_VALVE_CLOSE: manual_valve = true; set_valve(false); break;
        case PRESSURE_CMD_SET_MODE:
            mode_changed(info);
            break;
        case PRESSURE_CMD_START_PRESSURISATION:
            clear_overrides();
            status.state = PRESSURE_COMPRESSION;
            status.error = ERR_NONE;
            set_valve(false);
            set_measurement_outputs();
            break;
        case PRESSURE_CMD_STOP_PRESSURISATION:
            stop_pressure_train(false);
            status.state = PRESSURE_STANDBY;
            status.error = ERR_NONE;
            break;
        case PRESSURE_CMD_START_PREPRESSURISATION:
            clear_overrides();
            status.state = PRESSURE_PREPRESSURISATION;
            status.error = ERR_NONE;
            set_pump1(100);
            set_pump2(100);
            set_compressor(0);
            set_valve(false);
            break;
        case PRESSURE_CMD_SAFE_SHUTDOWN:
            stop_pressure_train(true);
            set_relay(2, false);
            set_relay(3, false);
            status.state = PRESSURE_STANDBY;
            status.error = ERR_NONE;
            break;
        case PRESSURE_CMD_RELAY1_ON: relay_manual = true; set_relay(1, true); break;
        case PRESSURE_CMD_RELAY1_OFF: relay_manual = true; set_relay(1, false); break;
        case PRESSURE_CMD_RELAY2_ON: relay_manual = true; set_relay(2, true); break;
        case PRESSURE_CMD_RELAY2_OFF: relay_manual = true; set_relay(2, false); break;
        case PRESSURE_CMD_RELAY3_ON: relay_manual = true; set_relay(3, true); break;
        case PRESSURE_CMD_RELAY3_OFF: relay_manual = true; set_relay(3, false); break;
        case PRESSURE_CMD_RELAY4_ON: relay_manual = true; set_relay(4, true); break;
        case PRESSURE_CMD_RELAY4_OFF: relay_manual = true; set_relay(4, false); break;
        default: ESP_LOGW("pressure", "unknown command 0x%02X", command); break;
    }
    status.relay_manual_override = relay_manual;
}

void pressure_update() {
    adjust_pressure_target();
    //ESP_LOGI("Sensors2: ", "Ambient pressure: %.3f, Inlet of Compressor %.3f, Chamber pressure: %.3f", status.ambient_pressure, status.compressor_inlet_pressure, status.chamber_pressure);

    if (!external_sensors_valid) {
        // Keep the existing bench/simulation behavior until real pressure
        // drivers are installed. The Main MCU sensor frame supersedes these
        // values as soon as one is received.
        static float fake_chamber = 0.0f;
        static float fake_inlet = 0.0f;
        fake_chamber += 2.0f;
        fake_inlet += 0.05f;
        status.chamber_pressure = fake_chamber;
        status.compressor_inlet_pressure = fake_inlet;
    }
    if (status.state == PRESSURE_PREPRESSURISATION) {
        //Prepressurise the volume infront of the compressor
        //if (applied_mode == PRESSURE_MODE_MEASUREMENTS) {
        //    if (!manual_pump1) set_pump1(100);
        //    if (!manual_pump2) set_pump2(100);
        //    if (!manual_compressor) set_compressor(0); 
        //    return;
        //}
        if (!manual_valve) set_valve(false); 
        if (!manual_compressor) set_compressor(0); //should be 0 because compressor and pumps can't be on at the same time
        if (!manual_pump1) set_pump1(50);
        if (!manual_pump2) set_pump2(50);
        if (status.compressor_inlet_pressure >= inlet_upper) {
            if (!manual_pump1) set_pump1(0);
            if (!manual_pump2) set_pump2(0);
            status.state = PRESSURE_COMPRESSION;
            flushstep_start = xTaskGetTickCount();
        }
    } else if (status.state == PRESSURE_COMPRESSION) {
        if (!manual_pump1) set_pump1(0);
        if (!manual_pump2) set_pump2(0);
        if (!manual_valve) set_valve(true);
        if (!manual_compressor) set_compressor(50);

        flushstep_stop = xTaskGetTickCount();
        flushticks = flushstep_stop-flushstep_start;
        flushsum = flushsum + 1/V*Qout*flushticks*portTICK_PERIOD_MS/1000/60;
        flushstep_start = xTaskGetTickCount();

        if ((status.compressor_inlet_pressure <= inlet_lower) 
            and (abs(status.chamber_pressure - target_pressure) < 0.1)) {
                if (flushsum > flushtarget) {
                    if (!manual_compressor) set_compressor(0);
                    if (!manual_valve) set_valve(false);
                    flushsum = 0;
                    status.state = PRESSURE_MEASUREMENT;
                    measurement_time_start = xTaskGetTickCount();
                    ESP_LOGI("pressure", "Measurement started at %.3f bar", status.chamber_pressure);
                }
                else {
                    if (!manual_compressor) set_compressor(0);
                    if (!manual_valve) set_valve(false);
                    status.state = PRESSURE_AIR_EXCHANGE;
                }
        } 
        else if ((status.compressor_inlet_pressure <= inlet_lower) and ((status.chamber_pressure - target_pressure) > 0)){
            if (!manual_compressor) set_compressor(0);
            if (!manual_valve) set_valve(false); //to make sure its closed until Air exchange is started in next loop
            status.state = PRESSURE_AIR_EXCHANGE;
            ESP_LOGI("pressure", "Too large chamber pressure: %.3f bar", status.chamber_pressure);
        } 
        else if ((status.compressor_inlet_pressure <= inlet_lower) and ((status.chamber_pressure - target_pressure) < 0)){
            if (!manual_compressor) set_compressor(0);
            if (!manual_valve) set_valve(false);
            status.state = PRESSURE_PREPRESSURISATION;
            compressed = true;
        } 
        else if ((compressed == true) & (abs(status.chamber_pressure - target_pressure) < 0.1)){
            if (flushsum > flushtarget) {
                if (!manual_compressor) set_compressor(0);
                if (!manual_valve) set_valve(false);
                flushsum = 0;
                status.state = PRESSURE_MEASUREMENT;
                measurement_time_start = xTaskGetTickCount();
                ESP_LOGI("pressure", "Measurement started at %.3f bar", status.chamber_pressure);
            }
            else {
                if (!manual_compressor) set_compressor(0);
                if (!manual_valve) set_valve(false);
                status.state = PRESSURE_AIR_EXCHANGE;
            }
        } 
    } else if (status.state == PRESSURE_MEASUREMENT) {
        if (!manual_pump1) set_pump1(0);
        if (!manual_pump2) set_pump2(0);
        if (!manual_valve) set_valve(false);
        if (!manual_compressor) set_compressor(0);
        TickType_t current_time = xTaskGetTickCount();
        TickType_t elapsed_ticks = current_time - measurement_time_start;
        if (elapsed_ticks*portTICK_PERIOD_MS/1000 >= measure_time) {
            status.state = PRESSURE_PREPRESSURISATION;
            ESP_LOGI("pressure", "Measurement done at %.3f bar", status.chamber_pressure);
            bool compressed = false;
        }
    } else if (status.state == PRESSURE_AIR_EXCHANGE) {
        set_pump1(0);
        set_pump2(0);
        set_compressor(0);
        set_valve(true);
        if ((status.chamber_pressure <= target_pressure+0.1) and (flushsum > flushtarget)) {
            status.state = PRESSURE_MEASUREMENT;
            flushsum = 0.0;
            set_valve(false);
            TickType_t measurement_time_start = xTaskGetTickCount();
            ESP_LOGI("pressure", "Measurement started at %.3f bar", status.chamber_pressure);
        }
        else if (status.chamber_pressure <= 2.0){
            set_valve(false);
            status.state = PRESSURE_PREPRESSURISATION;
        }
        
    } else if (status.state == PRESSURE_CORRECTION) {
        set_pump1(0);
        set_pump2(0);
        set_compressor(50);
        set_valve(false);
        if ((status.compressor_inlet_pressure <= inlet_lower) and (abs(status.chamber_pressure - target_pressure) > 0.1)){
            if (!manual_compressor) set_compressor(0);
            if (!manual_valve) set_valve(false);
            status.state = PRESSURE_PREPRESSURISATION;
            bool compressed = true;
        }
        else if (status.chamber_pressure >= target_pressure) {
            status.state = PRESSURE_MEASUREMENT;
            TickType_t measurement_time_start = xTaskGetTickCount();
            ESP_LOGI("pressure", "Measurement started at %.3f bar", status.chamber_pressure);
        }
    } else if (status.state == PRESSURE_STANDBY) {
        if (status.chamber_pressure < 3.0) {
            safe_off();
        }
        else {
            set_valve(true);
        }
    } else if (status.state == PRESSURE_ERROR) {
        safe_off();
    } 
}

void pressure_cmd_standby() { pressure_execute_command(PRESSURE_CMD_SET_MODE, PRESSURE_MODE_STANDBY); }
void pressure_cmd_measurements() { pressure_execute_command(PRESSURE_CMD_SET_MODE, PRESSURE_MODE_MEASUREMENTS); }
void pressure_set_target_pressure(float pressure) { target_pressure = pressure; }
void pressure_set_compressor_inlet_upper_limit(float pressure) { inlet_upper = pressure; }
void pressure_set_compressor_inlet_lower_limit(float pressure) { inlet_lower = pressure; }
PressureStatus pressure_get_status() { status.relay_manual_override = relay_manual; return status; }
bool pressure_system_is_on() { return status.state != PRESSURE_STANDBY; }
void pdb_relay1_on() { set_relay(1, true); } void pdb_relay1_off() { set_relay(1, false); }
void pdb_relay2_on() { set_relay(2, true); } void pdb_relay2_off() { set_relay(2, false); }
void pdb_relay3_on() { set_relay(3, true); } void pdb_relay3_off() { set_relay(3, false); }
void pdb_relay4_on() { set_relay(4, true); } void pdb_relay4_off() { set_relay(4, false); }
