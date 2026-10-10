#include "main.h"
#include "pressure_hardware.h"
#include "../../protocol/pressure_protocol.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include <algorithm>
#include <cmath>
#include <string.h>

namespace {
PressureStatus status = {};
float external_sensors[7] = {};
bool external_sensors_valid = false;
bool manual_pump1 = false, manual_pump2 = false, manual_compressor = false, manual_valve = false;
bool relay_manual = false;
uint8_t applied_mode = 0;
float target_pressure = 3.2f; //in pressure chamber
float inlet_upper = 1.5f, inlet_lower = 1.0f; //inlet upper and lower boundaries for compressor inlet
float compressor_safe_start_inlet_upper = 1.7f; //inlet upper boundary for compressor safe start. Otherwise it might scream at low duty cycles.
// Leave 0.20 bar of headroom below the 1.70 bar safe-start ceiling for
// sensor/update latency and pressure rise after the prefill pumps stop.
constexpr float INLET_PREFILL_MAX_BAR = 1.5f;
constexpr uint32_t INLET_START_WAIT_MAX_MS = 7000;
constexpr float CHAMBER_TARGET_TOLERANCE_BAR = 0.1f;
// In reality, the difference between interstage and pressure chamber is also relevant. I don't know how to account for that. 
float pwm1 = 0.0f, pwm2 = 0.0f, pwm3 = 0.0f;
//constexpr float FLUSH_COMPLETE_PRESSURE_BAR = 0.05f; //Why so low?
constexpr uint8_t ERR_NONE = 0, ERR_CHAMBER_SENSOR = 1, ERR_INLET_SENSOR = 2;
constexpr uint8_t ERR_INLET_START_TIMEOUT = 3;
bool pwm_targets_initialized = false; // to keep track of one-time-initialisation of pwm targets
bool inlet_start_wait_active = false;
TickType_t inlet_start_wait_started = 0;


TickType_t measurement_time_start;
TickType_t flushstep_start;
TickType_t flushstep_stop;
TickType_t flushticks;
TickType_t last_sensor_update;
TickType_t sensor_update_interval;
float previous_chamber_pressure;
float previous_compressor_inlet_pressure;
bool sensor_history_valid = false;

float measure_time = 20.0f; //in s
float flushsum = 0.0f; // sums up exchanged air
float V = 0.075; //75 ml estimated chamber volume
//float Qout = 1.0; // l/min based on compressor out flow rate measured in test for lower end of pressure range. Now replaced with flow_rate variable
float flow_rate;
float flushtarget = 0.225; // flushtarget = -Setup.V*np.log(0.05) which means that 95% of the air should be exchanged

bool compressed = false;

bool chamber_at_target() {
    return std::fabs(status.chamber_pressure - target_pressure) <= CHAMBER_TARGET_TOLERANCE_BAR;
}

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

void set_pwm_target_pump12(){
    if (!manual_pump1) set_pump1(pwm1);
    if (!manual_pump2) set_pump2(pwm2);
    ESP_LOGW("pressure", "Adjusted PWM target. Pump1: %.2f%%, Pump2: %.2f%%", pwm1, pwm2);
}

void set_pwm_target_pump3(){
    // When the compressor is at low duty cycles, it can struggle to start. Thus, we power it on briefly at 100% and then set it to what is required.
    // However, this is causing high current spikes and thus we will not use this.
    //if (!manual_compressor) set_compressor(20);
    //vTaskDelay(pdMS_TO_TICKS(100));
    if (!manual_compressor) set_compressor(pwm3);
    ESP_LOGW("pressure", "Adjusted PWM target. Compressor: %.2f%%", pwm3);
}

/*
I am not sure why, but sometimes the compressor squeaks and struggles to start. I believed this might be due to the required pressure drop
over the compressor increasing with inlet pressure. I now believe that it is solely a function of the inlet pressure, not of the 
pressure drop. I therefore have commented out the function below.


bool compressor_can_start() {
// This is calculating the required pressure drop over the compressor necessary to start. 
// The dp_crit is calculated from measurement data at 10% duty cycle. 
    float p_in = status.compressor_inlet_pressure;
    float p_out = status.chamber_pressure;
    float k = -0.65f; // This is determined by experiments.
    float dp_crit = k * (p_out - 1.0f); // If 1 bar inlet pressure: no pressure drop required. If 2 bar inlet pressure: k bar pressure drop required.
    bool can_start = (p_out - p_in) < dp_crit;
    if (!can_start) {
        ESP_LOGW("pressure", "Compressor cannot start. Inlet pressure: %.2f bar, Chamber pressure: %.2f bar, Pressure drop: %.2f bar, Required pressure drop: %.2f bar", p_in, p_out, (p_in - p_out), -dp_crit);
    }
    return can_start;
}
*/

bool compressor_can_start(){
    if (status.compressor_inlet_pressure > compressor_safe_start_inlet_upper){
        ESP_LOGW("pressure", "Compressor cannot start. Inlet pressure: %.2f bar, Upper limit: %.2f bar", status.compressor_inlet_pressure, compressor_safe_start_inlet_upper);
        return false;
    }
    else{
        return true;
    }
}


void set_measurement_outputs() {
    // If the measurement phase is entered while the interstage pressure is already above the lower limit, we can let the compressor run to deplete it to avoid the vacuum pumps hurting.
    if (status.compressor_inlet_pressure >= inlet_lower) {
        status.state = PRESSURE_COMPRESSION;
        return;
    }
    set_pwm_target_pump12();
    set_compressor(0);
}
void stop_pressure_train(bool open_valve) {
    clear_overrides();
    inlet_start_wait_active = false;
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

float calc_projected_chamber_pressure() {
    // This code is not working well. I will omit it for now.
    if (true) return status.chamber_pressure;
    const float pressure_rate = sensor_history_valid && sensor_update_interval > 0
        ? (status.chamber_pressure - previous_chamber_pressure) /
          (sensor_update_interval * portTICK_PERIOD_MS / 1000.0f)
        : 0.0f;
    return status.chamber_pressure +
        pressure_rate * sensor_update_interval * portTICK_PERIOD_MS / 1000.0f;
}

float calc_projected_inlet_pressure() {
    // This code is not working well. I will omit it for now.
    if (true) return status.compressor_inlet_pressure;
    const float pressure_rate = sensor_history_valid && sensor_update_interval > 0
        ? (status.compressor_inlet_pressure - previous_compressor_inlet_pressure) /
          (sensor_update_interval * portTICK_PERIOD_MS / 1000.0f)
        : 0.0f;
    return status.compressor_inlet_pressure +
        pressure_rate * sensor_update_interval * portTICK_PERIOD_MS / 1000.0f;
}

bool stop_pumps_if_sensor_invalid() {
    const TickType_t now = xTaskGetTickCount();
    const bool sensor_timeout = now - last_sensor_update > pdMS_TO_TICKS(1000);
    const float projected_chamber_pressure = calc_projected_chamber_pressure();
    const float projected_inlet_pressure = calc_projected_inlet_pressure();
    //if (!sensor_timeout && projected_chamber_pressure <= target_pressure && projected_inlet_pressure <= inlet_upper) return false;
    if (sensor_timeout) {
        ESP_LOGW("pressure", "Sensor timeout. No valid sensor data for more than 1 second. Stopping pumps.");
        return true;
    }
    return false;

    // I will not use the projected pressures for now, as they are not working well. I will just use the current pressures.
    /*
    else if (!sensor_timeout && projected_chamber_pressure > target_pressure) {
        ESP_LOGW("pressure", "Chamber pressure too high. Projected: %.2f bar, Target: %.2f bar. Stopping compressor.", projected_chamber_pressure, target_pressure);
        set_compressor(0);
    } else if (!sensor_timeout && projected_inlet_pressure > inlet_upper) {
        ESP_LOGW("pressure", "Compressor inlet pressure too high. Projected: %.2f bar, Target: %.2f bar. Stopping pumps.", projected_inlet_pressure, inlet_upper);
        set_pump1(0);
        set_pump2(0);
    } else if (sensor_timeout) {
        ESP_LOGW("pressure", "Sensor timeout. No valid sensor data for more than 1 second. Stopping pumps.");
        set_pump1(0);
        set_pump2(0);
        set_compressor(0);
    }
    return true;
    */
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

float calc_Q_out() {
    // Calculate the flow rate based on the pressure change in the chamber and the volume of the chamber.
    const float pressure_change = status.chamber_pressure - previous_chamber_pressure;
    flow_rate = (pressure_change * V) / (sensor_update_interval * portTICK_PERIOD_MS / 1000.0f); // in bar*L/s
    flow_rate = flow_rate* 1/status.chamber_pressure; // Convert to L/s using the ideal gas law (assuming constant temperature)
    return flow_rate * 60.0f; // Convert to L/min
}

}

void pressure_init_pwm(){
    // the pwm settings of the pumps need to depend on the ambient pressure as the vacuum pumps would overpressurise the interstage otherwise.
    // the pwm settings should be linearly dependent such that 0.04 bar -> 100% pwm and 1.0 bar -> 20% pwm
    pwm1 = 100.0f - ((status.ambient_pressure - 0.04f) / (1.0f - 0.04f)) * 80.0f;
    pwm1 = (pwm1 > 100) ? 100 : ((pwm1 < 20) ? 20 : pwm1);
    pwm2 = 100.0f - ((status.ambient_pressure - 0.04f) / (1.0f - 0.04f)) * 80.0f;
    pwm2 = (pwm2 > 100) ? 100 : ((pwm2 < 20) ? 20 : pwm2);
    //The pwm setting of the compressor might be decided to depend on ambient conditions
    //For now (02.10.) the compressor will just be set to 10% duty cycle
    pwm3 = 10.0f;
    pwm3 = (pwm3 > 100) ? 100 : ((pwm3 < 10) ? 10 : pwm3);
}

void pressure_init() {
    pressure_hardware_init();
    status.state = PRESSURE_STANDBY;
    status.error = ERR_NONE;
    set_pump1(0); set_pump2(0); set_compressor(0); set_valve(false);
    for (uint8_t relay = 1; relay <= 4; ++relay) set_relay(relay, false);
}

void pressure_update_external_sensors(const float sensors[7]) {
    const TickType_t now = xTaskGetTickCount();
    if (external_sensors_valid) {
        sensor_update_interval = now - last_sensor_update;
        previous_chamber_pressure = status.chamber_pressure;
        previous_compressor_inlet_pressure = status.compressor_inlet_pressure;
        sensor_history_valid = sensor_update_interval > 0;
    }
    last_sensor_update = now;
    memcpy(external_sensors, sensors, sizeof(external_sensors));
    external_sensors_valid = true;
    status.chamber_pressure = sensors[2];
    status.ambient_pressure = sensors[3];
    status.compressor_inlet_pressure = (sensors[1]+status.ambient_pressure);
    if (!pwm_targets_initialized) {
        pressure_init_pwm();
        pwm_targets_initialized = true;
    }
}



void adjust_pressure_target(){
    //upper limit of compressor inlet pressure should depend on ambient pressure
    if ((status.ambient_pressure < 1.1) and (status.ambient_pressure>=0.7)){
        inlet_upper = 1.7;
        inlet_lower = 0.95;
    }
    else if ((status.ambient_pressure < 0.7) and (status.ambient_pressure>=0.5)){
        inlet_upper = 1.5;
        inlet_lower = 0.95;
    }
    else if ((status.ambient_pressure < 0.5) and (status.ambient_pressure>=0.2)){
        inlet_upper = 1.5;
        inlet_lower = 0.95;
    }
    else if (status.ambient_pressure < 0.2){
        inlet_upper = 1.5;
        inlet_lower = 0.95;
    }
}

void pressure_execute_command(uint8_t command, uint8_t info) {
    switch (command) {
        // Legacy ON commands do not carry a PWM value. They must mean full
        // speed; explicit partial duty cycles use the *_PWM commands below.
        case PRESSURE_CMD_PUMP1_ON: manual_pump1 = true; pwm1 = (pwm1 > 100 ? 100 : ((pwm1 < 15) ? 15 : pwm1)); set_pump1(pwm1); break;
        case PRESSURE_CMD_PUMP1_OFF: manual_pump1 = true; set_pump1(0); break;
        case PRESSURE_CMD_PUMP2_ON: manual_pump2 = true; pwm2 = (pwm2 > 100 ? 100 : ((pwm2 < 15) ? 15 : pwm2)); set_pump2(pwm2); break;
        case PRESSURE_CMD_PUMP2_OFF: manual_pump2 = true; set_pump2(0); break;
        case PRESSURE_CMD_COMPRESSOR_ON: manual_compressor = true; pwm3 = (pwm3 > 100 ? 100 : ((pwm3 < 15) ? 15 : pwm3)); set_compressor(pwm3); break;
        case PRESSURE_CMD_COMPRESSOR_OFF: manual_compressor = true; set_compressor(0); break;
        case PRESSURE_CMD_PUMP1_PWM: manual_pump1 = true; pwm1 = (info > 100 ? 100 : (info < 15 ? 15 : info)); set_pump1(pwm1); break;
        case PRESSURE_CMD_PUMP2_PWM: manual_pump2 = true; pwm2 = (info > 100 ? 100 : (info < 15 ? 15 : info)); set_pump2(pwm2); break;
        case PRESSURE_CMD_COMPRESSOR_PWM: manual_compressor = true; pwm3 = (info > 100 ? 100 : (info < 15 ? 15 : info)); set_compressor(pwm3); break;
        case PRESSURE_CMD_PUMP1_PWM_NoInterrupt: manual_pump1 = false; pwm1 = (info > 100 ? 100 : info); break;
        case PRESSURE_CMD_PUMP2_PWM_NoInterrupt: manual_pump2 = false; pwm2 = (info > 100 ? 100 : info); break;
        case PRESSURE_CMD_COMPRESSOR_PWM_NoInterrupt: manual_compressor = false; pwm3 = (info > 100 ? 100 : info); break;
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
            set_measurement_outputs();
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
    if (status.state != PRESSURE_COMPRESSION) inlet_start_wait_active = false;
    ESP_LOGI("pressure", "Starting pressure update. State: %d, Chamber: %.3f bar, Inlet: %.3f bar, Ambient: %.3f bar",
             status.state, status.chamber_pressure, status.compressor_inlet_pressure, status.ambient_pressure);
    //ESP_LOGI("Sensors2: ", "Ambient pressure: %.3f, Inlet of Compressor %.3f, Chamber pressure: %.3f", status.ambient_pressure, status.compressor_inlet_pressure, status.chamber_pressure);

    if (!external_sensors_valid) {
       ESP_LOGE("pressure", "External sensors not valid. Cannot update pressure control.");
        return;
    }
    if (stop_pumps_if_sensor_invalid()) {
        safe_off(); 
        ESP_LOGE("pressure", "Sensor invalid. Stopping all pumps and valves.");
        return;
    }
    if (status.state == PRESSURE_PREPRESSURISATION) {
        ESP_LOGI("pressure", "Prepressurisation state. Chamber: %.3f bar, Inlet: %.3f bar, Ambient: %.3f bar",
                 status.chamber_pressure, status.compressor_inlet_pressure, status.ambient_pressure);
        //Prepressurise the volume infront of the compressor
        //if (applied_mode == PRESSURE_MODE_MEASUREMENTS) {
        //    if (!manual_pump1) set_pump1(100);
        //    if (!manual_pump2) set_pump2(100);
        //    if (!manual_compressor) set_compressor(0); 
        //    return;
        //}

        const float inlet_fill_target = std::min(inlet_upper, INLET_PREFILL_MAX_BAR);
        if (status.compressor_inlet_pressure >= inlet_fill_target) {
            // Stop filling in this update, not one update after changing state.
            if (!manual_pump1) set_pump1(0);
            if (!manual_pump2) set_pump2(0);
            if (!manual_compressor) set_compressor(0);
            if (!manual_valve) set_valve(false);
            inlet_start_wait_active = false;
            status.state = PRESSURE_COMPRESSION;
            flushstep_start = xTaskGetTickCount();
            return;
        }
        else {
            if (!manual_valve) set_valve(false); 
            if (!manual_compressor) set_compressor(0); //should be 0 because compressor and pumps can't be on at the same time
            set_pwm_target_pump12();
        }
    } else if (status.state == PRESSURE_COMPRESSION) {
        ESP_LOGI("pressure", "Compression state. Chamber: %.3f bar, Inlet: %.3f bar, Ambient: %.3f bar",
                 status.chamber_pressure, status.compressor_inlet_pressure, status.ambient_pressure);
        if (!manual_pump1) set_pump1(0);
        if (!manual_pump2) set_pump2(0);

        // Before turning the compressor on we should check that there is not too much pressure in the chamber. If there is, we should first flush the chamber to avoid overpressurisation.
        if ((status.chamber_pressure - target_pressure) > 0.1) {
            if (!manual_compressor) set_compressor(0);
            status.state = PRESSURE_AIR_EXCHANGE;
            ESP_LOGI("pressure", "Chamber pressure too high: %.3f bar. Starting air exchange.", status.chamber_pressure);
            return;
        }
        // Before turning on the compressor we should check that the inlet/interstage pressure is not too high. 
        else if (status.compressor_inlet_pressure > compressor_safe_start_inlet_upper) {
            if (!manual_compressor) set_compressor(0);
            if (!manual_valve) set_valve(false);
            const TickType_t now = xTaskGetTickCount();
            if (!inlet_start_wait_active) {
                inlet_start_wait_active = true;
                inlet_start_wait_started = now;
            }
            if (now - inlet_start_wait_started >= pdMS_TO_TICKS(INLET_START_WAIT_MAX_MS)) {
                stop_pressure_train(false);
                status.error = ERR_INLET_START_TIMEOUT;
                status.state = PRESSURE_ERROR;
                ESP_LOGE("pressure", "Compressor inlet remained above %.3f bar for 7 seconds. Stopping pressure train.", compressor_safe_start_inlet_upper);
                return;
            }
            ESP_LOGI("pressure", "Compressor inlet pressure too high: %.3f bar. Waiting for it to drop.", status.compressor_inlet_pressure);
            return;
        }
        else {
            // Exclude an idle high-inlet wait from exchanged-air integration.
            if (inlet_start_wait_active) flushstep_start = xTaskGetTickCount();
            inlet_start_wait_active = false;
            if (compressor_can_start()) {
                set_pwm_target_pump3();
                if (!manual_valve) set_valve(false); //The valve must be closed to allow the compressor to pressurise the chamber.
            }
            else {
                if (!manual_compressor) set_compressor(0);
                if (!manual_valve) set_valve(false);
                status.state = PRESSURE_COMPRESSION; // Making it explicit that we want to remain in compression state
                ESP_LOGI("pressure", "Compressor cannot start due to high inlet pressure: %.3f bar. Waiting for it to drop.", status.compressor_inlet_pressure);
                return;
            }
        }

        flushstep_stop = xTaskGetTickCount();
        flushticks = flushstep_stop-flushstep_start;
        float Qout = calc_Q_out();
        flushsum = flushsum + 1/V*Qout*flushticks*portTICK_PERIOD_MS/1000/60;
        flushstep_start = xTaskGetTickCount();

        if (chamber_at_target()) {
                ESP_LOGI("pressure", "Chamber pressure at target (%.3f bar): %.3f bar, Inlet: %.3f bar", target_pressure, status.chamber_pressure, status.compressor_inlet_pressure);
                // If the chamber pressure is at target and the inlet is emptied, we can check if we can go to measurement. 
                if (flushsum > flushtarget) {
                    // We can only go to measurement if the air has been properly flushed.
                    if (!manual_compressor) set_compressor(0);
                    if (!manual_valve) set_valve(false);
                    flushsum = 0;
                    status.state = PRESSURE_MEASUREMENT;
                    measurement_time_start = xTaskGetTickCount();
                    ESP_LOGI("pressure", "Measurement started at %.3f bar", status.chamber_pressure);
                }
                else {
                    // If the air hasn't been flushed enough, we need to go to air exchange to flush the chamber.
                    if (!manual_compressor) set_compressor(0);
                    if (!manual_valve) set_valve(false);
                    status.state = PRESSURE_AIR_EXCHANGE;
                }
        } 
        else if ((status.compressor_inlet_pressure <= inlet_lower) and ((status.chamber_pressure - target_pressure) < 0)){
            // If the chamber pressure is below target and the inlet is emptied, we can go to prepressurisation to fill the inlet again.
            if (!manual_compressor) set_compressor(0);
            if (!manual_valve) set_valve(false);
            status.state = PRESSURE_PREPRESSURISATION;
            ESP_LOGI("pressure", "Chamber pressure below target: %.3f bar. Starting prepressurisation.", status.chamber_pressure);
        }
            
    } else if (status.state == PRESSURE_MEASUREMENT) {
        ESP_LOGI("pressure", "Measurement state. Chamber: %.3f bar, Inlet: %.3f bar, Ambient: %.3f bar",
                 status.chamber_pressure, status.compressor_inlet_pressure, status.ambient_pressure);
        if (!manual_pump1) set_pump1(0);
        if (!manual_pump2) set_pump2(0);
        if (!manual_valve) set_valve(false);
        if (!manual_compressor) set_compressor(0);
        TickType_t current_time = xTaskGetTickCount();
        TickType_t elapsed_ticks = current_time - measurement_time_start;
        if (elapsed_ticks*portTICK_PERIOD_MS/1000 >= measure_time) {
            status.state = PRESSURE_AIR_EXCHANGE;
            if (!manual_valve) set_valve(true);
            ESP_LOGI("pressure", "Measurement done at %.3f bar", status.chamber_pressure);
            bool compressed = false;
        }
    } else if (status.state == PRESSURE_AIR_EXCHANGE) {
        set_pump1(0);
        set_pump2(0);
        set_compressor(0);
        set_valve(true);
        ESP_LOGI("pressure", "Air exchange state. Chamber: %.3f bar, Inlet: %.3f bar, Ambient: %.3f bar",
                 status.chamber_pressure, status.compressor_inlet_pressure, status.ambient_pressure);
        if (chamber_at_target() && (flushsum > flushtarget)) {
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
        
    } else if (status.state == PRESSURE_CORRECTION) { // What is this for - Jonathan 02.10.
        ESP_LOGW("pressure", "Correction state. Chamber: %.3f bar, Inlet: %.3f bar, Ambient: %.3f bar",
                 status.chamber_pressure, status.compressor_inlet_pressure, status.ambient_pressure);
        set_pump1(0);
        set_pump2(0);
        set_compressor(50);
        set_valve(false);
        if ((status.compressor_inlet_pressure <= inlet_lower) and !chamber_at_target()){
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
        ESP_LOGI("pressure", "Standby state. Chamber: %.3f bar, Inlet: %.3f bar, Ambient: %.3f bar",
                 status.chamber_pressure, status.compressor_inlet_pressure, status.ambient_pressure);
        if (status.chamber_pressure < 3.0) {
            safe_off();
        }
        else {
            set_valve(true);
        }
    } else if (status.state == PRESSURE_ERROR) {
        ESP_LOGE("pressure", "Error state. Chamber: %.3f bar, Inlet: %.3f bar, Ambient: %.3f bar",
                 status.chamber_pressure, status.compressor_inlet_pressure, status.ambient_pressure);
        safe_off();
    } 
    else {
        ESP_LOGE("pressure", "Unknown state %d. Stopping all pumps and valves.", status.state);
        safe_off();
        status.state = PRESSURE_ERROR;
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
