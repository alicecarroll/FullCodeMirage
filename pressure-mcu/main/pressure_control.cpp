#include "main.h"
#include "pressure_hardware.h"
#include "../../protocol/pressure_protocol.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

namespace {
PressureStatus status = {};
float external_sensors[7] = {};
bool external_sensors_valid = false;
bool manual_pump1 = false, manual_pump2 = false, manual_compressor = false, manual_valve = false;
bool relay_manual = false;
uint8_t applied_mode = 0;
float target_pressure = 3.0f; // absolute pressure in the measurement chamber
float inlet_upper = 1.6f, inlet_lower = 1.2f; //inlet upper and lower boundaries for compressor inlet
constexpr float TARGET_TOLERANCE_BAR = 0.1f;
constexpr float MINIMUM_CHAMBER_PRESSURE_BAR = 2.0f;
constexpr float MEASUREMENT_TIME_SECONDS = 20.0f;
constexpr float CHAMBER_VOLUME_LITRES = 0.075f;
constexpr float COMPRESSOR_FLOW_LITRES_PER_MINUTE = 1.5f;
// Three chamber volumes replaces about 95% of a well-mixed gas sample.
constexpr float EXCHANGE_TARGET_LITRES = 3.0f * CHAMBER_VOLUME_LITRES;
constexpr uint8_t AUTOMATIC_PWM = 50;
constexpr uint8_t ERR_NONE = 0;

TickType_t measurement_time_start;
TickType_t compression_update_start;
float exchanged_air_litres = 0.0f;

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

bool chamber_at_target() {
    return status.chamber_pressure >= target_pressure - TARGET_TOLERANCE_BAR;
}

void enter_prepressurisation() {
    status.state = PRESSURE_PREPRESSURISATION;
    if (!manual_pump1) set_pump1(AUTOMATIC_PWM);
    if (!manual_pump2) set_pump2(AUTOMATIC_PWM);
    if (!manual_compressor) set_compressor(0);
    if (!manual_valve) set_valve(false);
}

void enter_compression() {
    status.state = PRESSURE_COMPRESSION;
    if (!manual_pump1) set_pump1(0);
    if (!manual_pump2) set_pump2(0);
    if (!manual_compressor) set_compressor(AUTOMATIC_PWM);
    // This is the chamber outlet valve. It must remain closed while filling.
    if (!manual_valve) set_valve(false);
    compression_update_start = xTaskGetTickCount();
}

void enter_air_exchange() {
    status.state = PRESSURE_AIR_EXCHANGE;
    if (!manual_pump1) set_pump1(0);
    if (!manual_pump2) set_pump2(0);
    if (!manual_compressor) set_compressor(0);
    if (!manual_valve) set_valve(true);
}

void enter_measurement() {
    status.state = PRESSURE_MEASUREMENT;
    if (!manual_pump1) set_pump1(0);
    if (!manual_pump2) set_pump2(0);
    if (!manual_compressor) set_compressor(0);
    if (!manual_valve) set_valve(false);
    exchanged_air_litres = 0.0f;
    measurement_time_start = xTaskGetTickCount();
    ESP_LOGI("pressure", "Measurement started at %.3f bar", status.chamber_pressure);
}

void start_automatic_cycle() {
    exchanged_air_litres = 0.0f;
    if (chamber_at_target()) enter_air_exchange();
    else enter_prepressurisation();
}

void mode_changed(uint8_t mode) {
    if (mode == applied_mode) return;
    applied_mode = mode;
    clear_overrides();
    status.error = ERR_NONE;
    if (mode == PRESSURE_MODE_MEASUREMENTS) {
        set_relay(2, true);
        set_relay(3, true);
        start_automatic_cycle();
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
    external_sensors_valid = false;
    applied_mode = 0;
    exchanged_air_litres = 0.0f;
    clear_overrides();
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
            status.error = ERR_NONE;
            // Run the whole automatic cycle. Starting in compression used to
            // bypass the compressor-inlet pressure prerequisite.
            start_automatic_cycle();
            break;
        case PRESSURE_CMD_STOP_PRESSURISATION:
            stop_pressure_train(false);
            exchanged_air_litres = 0.0f;
            status.state = PRESSURE_STANDBY;
            status.error = ERR_NONE;
            break;
        case PRESSURE_CMD_START_PREPRESSURISATION:
            clear_overrides();
            status.error = ERR_NONE;
            exchanged_air_litres = 0.0f;
            enter_prepressurisation();
            break;
        case PRESSURE_CMD_FLUSH_CHAMBER:
            clear_overrides();
            status.error = ERR_NONE;
            exchanged_air_litres = 0.0f;
            enter_air_exchange();
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
        if (!manual_valve) set_valve(false);
        if (!manual_compressor) set_compressor(0);
        if (!manual_pump1) set_pump1(AUTOMATIC_PWM);
        if (!manual_pump2) set_pump2(AUTOMATIC_PWM);

        // A full chamber needs fresh air, not another compression pulse.
        if (chamber_at_target()) enter_air_exchange();
        else if (status.compressor_inlet_pressure >= inlet_upper) enter_compression();
    } else if (status.state == PRESSURE_COMPRESSION) {
        if (!manual_pump1) set_pump1(0);
        if (!manual_pump2) set_pump2(0);
        if (!manual_valve) set_valve(false);
        if (!manual_compressor) set_compressor(AUTOMATIC_PWM);

        const TickType_t now = xTaskGetTickCount();
        const TickType_t elapsed_ticks = now - compression_update_start;
        if (status.compressor_pwm > 0) {
            const float elapsed_minutes =
                elapsed_ticks * portTICK_PERIOD_MS / 1000.0f / 60.0f;
            exchanged_air_litres +=
                COMPRESSOR_FLOW_LITRES_PER_MINUTE * elapsed_minutes;
        }
        compression_update_start = now;

        // Reaching chamber pressure ends compression immediately. If the
        // inlet charge runs out first, refill it and repeat.
        if (chamber_at_target()) {
            if (exchanged_air_litres >= EXCHANGE_TARGET_LITRES) enter_measurement();
            else enter_air_exchange();
        } else if (status.compressor_inlet_pressure <= inlet_lower) {
            enter_prepressurisation();
        }
    } else if (status.state == PRESSURE_MEASUREMENT) {
        if (!manual_pump1) set_pump1(0);
        if (!manual_pump2) set_pump2(0);
        if (!manual_valve) set_valve(false);
        if (!manual_compressor) set_compressor(0);
        const TickType_t current_time = xTaskGetTickCount();
        const TickType_t elapsed_ticks = current_time - measurement_time_start;
        const float elapsed_seconds = elapsed_ticks * portTICK_PERIOD_MS / 1000.0f;
        if (elapsed_seconds >= MEASUREMENT_TIME_SECONDS) {
            ESP_LOGI("pressure", "Measurement done at %.3f bar", status.chamber_pressure);
            exchanged_air_litres = 0.0f;
            enter_air_exchange();
        }
    } else if (status.state == PRESSURE_AIR_EXCHANGE) {
        if (!manual_pump1) set_pump1(0);
        if (!manual_pump2) set_pump2(0);
        if (!manual_compressor) set_compressor(0);
        if (!manual_valve) set_valve(true);
        if (status.chamber_pressure <= MINIMUM_CHAMBER_PRESSURE_BAR) {
            enter_prepressurisation();
        }
    } else if (status.state == PRESSURE_CORRECTION) {
        // Retained for wire compatibility with older firmware. New cycles
        // recover under-pressure through prepressurisation and compression.
        enter_prepressurisation();
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
