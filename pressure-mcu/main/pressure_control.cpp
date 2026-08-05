#include "main.h"
#include "pressure_hardware.h"
#include "pressure_protocol.h"
#include "esp_log.h"
#include <string.h>

namespace {
PressureStatus status = {};
float external_sensors[7] = {};
bool external_sensors_valid = false;
bool manual_pump1 = false, manual_pump2 = false, manual_compressor = false, manual_valve = false;
bool relay_manual = false;
uint8_t applied_mode = 0;
float target_pressure = 3.0f;
float inlet_upper = 1.0f, inlet_lower = 0.2f;
constexpr uint8_t ERR_NONE = 0, ERR_CHAMBER_SENSOR = 1, ERR_INLET_SENSOR = 2;

void set_pump1(uint8_t pwm) { pressure_pump1_set(pwm); status.pump1_pwm = pwm; }
void set_pump2(uint8_t pwm) { pressure_pump2_set(pwm); status.pump2_pwm = pwm; }
void set_compressor(uint8_t pwm) { pressure_compressor_set(pwm); status.compressor_pwm = pwm; }
void set_valve(bool open) { pressure_valve_set(open); status.valve_open = open; }
void set_relay(uint8_t relay, bool on) {
    pressure_relay_set(relay, on);
    const uint8_t bit = static_cast<uint8_t>(1U << (relay - 1));
    status.relay_mask = on ? status.relay_mask | bit : status.relay_mask & ~bit;
}
void clear_overrides() { manual_pump1 = manual_pump2 = manual_compressor = manual_valve = false; relay_manual = false; }
void set_measurement_outputs() {
    set_pump1(100); set_pump2(100); set_compressor(100);
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
    if (mode == PRESSURE_MODE_MEASUREMENTS) { set_relay(2, true); set_relay(3, true); set_measurement_outputs(); }
    else if (mode == PRESSURE_MODE_STANDBY) { set_relay(2, false); set_relay(3, false); safe_off(); }
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
    status.compressor_inlet_pressure = sensors[1];
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
            status.state = info == PRESSURE_MODE_STANDBY ? PRESSURE_STANDBY : PRESSURE_PREPRESSURISATION;
            status.error = ERR_NONE;
            if (status.state == PRESSURE_STANDBY) safe_off();
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
    if (!external_sensors_valid) {
        // Keep the existing bench/simulation behavior until real pressure
        // drivers are installed. The Main MCU sensor frame supersedes these
        // values as soon as one is received.
        static float fake_chamber = 0.0f;
        static float fake_inlet = 0.0f;
        fake_chamber += 0.05f;
        fake_inlet += 0.05f;
        status.chamber_pressure = fake_chamber;
        status.compressor_inlet_pressure = fake_inlet;
    }
    if (status.state == PRESSURE_PREPRESSURISATION) {
        if (applied_mode == PRESSURE_MODE_MEASUREMENTS) {
            if (!manual_pump1) set_pump1(100);
            if (!manual_pump2) set_pump2(100);
            if (!manual_compressor) set_compressor(100);
            return;
        }
        if (!manual_valve) set_valve(false);
        if (!manual_compressor) set_compressor(255);
        if (!manual_pump1) set_pump1(255);
        if (!manual_pump2) set_pump2(255);
        if (status.compressor_inlet_pressure >= inlet_upper) {
            if (!manual_pump1) set_pump1(0);
            if (!manual_pump2) set_pump2(0);
            status.state = PRESSURE_AIR_EXCHANGE;
        }
    } else if (status.state == PRESSURE_AIR_EXCHANGE) {
        if (!manual_pump1) set_pump1(0);
        if (!manual_pump2) set_pump2(0);
        if (!manual_valve) set_valve(true);
        if (!manual_compressor) set_compressor(255);
        if (status.compressor_inlet_pressure <= inlet_lower) {
            if (!manual_compressor) set_compressor(0);
            if (!manual_valve) set_valve(false);
            status.state = PRESSURE_PREPRESSURISATION;
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
