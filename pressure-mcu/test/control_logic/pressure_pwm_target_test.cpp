#include <cassert>
#include <cmath>
#include <cstdio>

// Host-only regression tests for the real controller, with no hardware access.
// From the repo root, build with:
// g++ -std=c++17 -Ipressure-mcu/test/control_logic pressure-mcu/test/control_logic/pressure_pwm_target_test.cpp -o /tmp/mirage-pressure-pwm-test
#include "../../main/pressure_control.cpp"

namespace {
TickType_t simulated_ticks = 100;
uint8_t hardware_pwm[3] = {};
bool hardware_valve = false;
bool hardware_relays[4] = {};
uint16_t simulated_current_ma = 0;
bool simulated_adc_valid = true, simulated_adc_saturated = false;

void reset_controller() {
    simulated_ticks = 100;
    status = {};
    status.state = PRESSURE_STANDBY;
    status.current_valid = true;
    external_sensors_valid = false;
    applied_mode = PRESSURE_MODE_STANDBY;
    current_shutdown_standby = false;
    clear_overrides();
    pump1_pwm_user_set = pump2_pwm_user_set = compressor_pwm_user_set = false;
    pwm1 = pwm2 = pwm3 = 0;
    target_pressure = 3.2f;
    inlet_lower = 0.95f;
    inlet_upper = compressor_safe_start_inlet_upper = 1.7f;
    air_exchange_threshold = 2.0f;
    inlet_start_wait_active = false;
    inlet_start_wait_started = 0;
    last_sensor_update = 0;
    sensor_update_interval = pdMS_TO_TICKS(100);
    sensor_history_valid = false;
    previous_chamber_pressure = previous_compressor_inlet_pressure = 0;
    measurement_time_start = flushstep_start = simulated_ticks;
    flushsum = 0;
    simulated_current_ma = 0;
    simulated_adc_valid = true;
    simulated_adc_saturated = false;
    set_pump1(0); set_pump2(0); set_compressor(0); set_valve(false);
    for (uint8_t relay = 1; relay <= 4; ++relay) set_relay(relay, false);
}

void sensor_frame(float ambient = 0.99f, float chamber = 2.2f, float inlet = 1.2f) {
    const float sensors[12] = {
        0.1f, inlet - ambient, chamber, ambient, 20, 20, 20,
        3.2f, 0.95f, 1.7f, 2.0f, 0
    };
    simulated_ticks += pdMS_TO_TICKS(100);
    pressure_update_external_sensors(sensors);
}

void select_targets() {
    pressure_execute_command(PRESSURE_CMD_PUMP1_PWM_NoInterrupt, 47);
    pressure_execute_command(PRESSURE_CMD_PUMP2_PWM_NoInterrupt, 62);
    pressure_execute_command(PRESSURE_CMD_COMPRESSOR_PWM_NoInterrupt, 55);
}

void assert_selected_targets() {
    assert(pwm1 == 47 && pwm2 == 62 && pwm3 == 55);
    assert(pump1_pwm_user_set && pump2_pwm_user_set && compressor_pwm_user_set);
    assert(!manual_pump1 && !manual_pump2 && !manual_compressor);
}

void assert_pwm_off() {
    for (uint8_t pwm : hardware_pwm) assert(pwm == 0);
}

void assert_all_loads_off() {
    assert_pwm_off();
    for (bool relay : hardware_relays) assert(!relay);
    assert(status.relay_mask == 0 && !hardware_valve);
}

float default_pump_pwm(float ambient) {
    return std::max(20.0f, std::min(100.0f, 100.0f - ((ambient - 0.04f) / 0.96f) * 80.0f));
}

void test_defaults_remain_ambient_dependent() {
    reset_controller();
    for (float ambient : {0.99f, 0.5f, 0.04f, 1.2f}) {
        sensor_frame(ambient);
        const float expected = default_pump_pwm(ambient);
        assert(std::fabs(pwm1 - (ambient < 0.7f ? expected : 0)) < 0.001f);
        assert(std::fabs(pwm2 - expected) < 0.001f);
        assert(pwm3 == 10);
        assert(!pump1_pwm_user_set && !pump2_pwm_user_set && !compressor_pwm_user_set);
        assert_all_loads_off();
    }
}

void test_targets_survive_first_and_repeated_sensor_frames() {
    reset_controller();
    // A command may arrive before the first sensor frame.
    select_targets();
    assert_selected_targets();
    assert_all_loads_off();
    for (unsigned frame = 0; frame < 30; ++frame) {
        sensor_frame(frame % 2 ? 0.5f : 0.99f);
        pressure_update();
        assert_selected_targets();
        assert_all_loads_off();
    }
}

void test_unselected_targets_still_adapt_independently() {
    reset_controller();
    sensor_frame();
    pressure_execute_command(PRESSURE_CMD_COMPRESSOR_PWM_NoInterrupt, 55);
    for (float ambient : {0.5f, 0.99f}) {
        sensor_frame(ambient);
        assert(pwm3 == 55 && compressor_pwm_user_set && !manual_compressor);
        assert(std::fabs(pwm2 - default_pump_pwm(ambient)) < 0.001f);
        assert(!pump1_pwm_user_set && !pump2_pwm_user_set);
    }
    pressure_execute_command(PRESSURE_CMD_PUMP1_PWM_NoInterrupt, 47);
    sensor_frame(0.04f);
    assert(pwm1 == 47 && pwm2 == 100 && pwm3 == 55);
    assert(!pump2_pwm_user_set);
}

void test_later_commands_replace_targets_without_changing_limits() {
    reset_controller();
    for (uint8_t command : {PRESSURE_CMD_PUMP1_PWM_NoInterrupt,
                            PRESSURE_CMD_PUMP2_PWM_NoInterrupt,
                            PRESSURE_CMD_COMPRESSOR_PWM_NoInterrupt}) {
        for (uint8_t value : {55, 70, 0, 100, 255}) {
            pressure_execute_command(command, value);
            sensor_frame();
            const float actual = command == PRESSURE_CMD_PUMP1_PWM_NoInterrupt ? pwm1 :
                                 command == PRESSURE_CMD_PUMP2_PWM_NoInterrupt ? pwm2 : pwm3;
            assert(actual == (value > 100 ? 100 : value));
            assert(!manual_pump1 && !manual_pump2 && !manual_compressor);
            assert_all_loads_off();
        }
    }
}

void test_automatic_phases_still_control_outputs() {
    reset_controller();
    sensor_frame(0.99f, 2.2f, 0.9f);
    select_targets();
    pressure_execute_command(PRESSURE_CMD_SET_MODE, PRESSURE_MODE_MEASUREMENTS);
    pressure_update();
    assert(status.state == PRESSURE_PREPRESSURISATION);
    assert(hardware_pwm[0] == 47 && hardware_pwm[1] == 62 && hardware_pwm[2] == 0);

    sensor_frame(0.99f, 2.2f, 1.7f);
    pressure_update();
    assert(status.state == PRESSURE_COMPRESSION);
    assert_pwm_off();

    sensor_frame();
    pressure_update();
    assert(hardware_pwm[0] == 0 && hardware_pwm[1] == 0 && hardware_pwm[2] == 55);

    sensor_frame(0.99f, 3.2f);
    flushsum = flushtarget + 1;
    pressure_update();
    assert(status.state == PRESSURE_MEASUREMENT);
    assert_pwm_off();
    sensor_frame(0.99f, 3.2f);
    pressure_update();
    assert(status.state == PRESSURE_MEASUREMENT && !hardware_valve);
    assert_pwm_off();

    simulated_ticks += pdMS_TO_TICKS(20000);
    sensor_frame(0.99f, 3.2f);
    pressure_update();
    assert(status.state == PRESSURE_AIR_EXCHANGE && hardware_valve);
    assert_pwm_off();
    sensor_frame(0.99f, 1.9f, 0.9f);
    pressure_update();
    assert(status.state == PRESSURE_PREPRESSURISATION && !hardware_valve);
    sensor_frame(0.99f, 1.9f, 0.9f);
    pressure_update();
    assert(hardware_pwm[0] == 47 && hardware_pwm[1] == 62 && hardware_pwm[2] == 0);
    assert_selected_targets();
}

void test_stop_and_mode_changes_keep_selected_targets() {
    reset_controller();
    sensor_frame(0.99f, 2.2f, 0.9f);
    select_targets();
    pressure_cmd_measurements();
    pressure_update();
    pressure_execute_command(PRESSURE_CMD_STOP_PRESSURISATION, 0);
    assert_pwm_off();
    sensor_frame(0.5f, 2.2f, 0.9f);
    pressure_update();
    assert_selected_targets();
    assert_pwm_off();
    pressure_cmd_standby();
    assert_all_loads_off();
    pressure_cmd_measurements();
    pressure_update();
    assert(hardware_pwm[0] == 47 && hardware_pwm[1] == 62 && hardware_pwm[2] == 0);
    assert_selected_targets();
}

void test_current_cutoff_still_wins_and_recovery_keeps_targets() {
    reset_controller();
    sensor_frame();
    select_targets();
    pressure_cmd_measurements();
    pressure_update();
    for (uint8_t relay = 1; relay <= 4; ++relay) set_relay(relay, true);
    simulated_current_ma = 2899;
    pressure_update_current();
    assert(!status.overcurrent_tripped && hardware_pwm[2] == 55 && status.relay_mask == 0x0f);
    simulated_current_ma = 2900;
    pressure_update_current();
    assert(status.overcurrent_tripped && status.error == ERR_OVERCURRENT);
    assert_all_loads_off();
    pressure_execute_command(PRESSURE_CMD_COMPRESSOR_PWM_NoInterrupt, 100);
    pressure_execute_command(PRESSURE_CMD_RELAY1_ON, 0);
    pressure_update();
    assert_selected_targets();
    assert_all_loads_off();

    simulated_current_ma = 0;
    pressure_update_current();
    for (unsigned frame = 0; frame < 3; ++frame) {
        sensor_frame();
        pressure_cmd_measurements();
        pressure_update();
        assert(current_shutdown_standby);
        assert_selected_targets();
        assert_all_loads_off();
    }
    pressure_execute_command(PRESSURE_CMD_START_PRESSURISATION, 0);
    pressure_update();
    assert(!current_shutdown_standby && hardware_pwm[2] == 55);
    assert_selected_targets();

    simulated_adc_valid = false;
    pressure_update_current();
    assert(status.error == ERR_CURRENT_SENSOR);
    assert_selected_targets();
    assert_all_loads_off();
    simulated_adc_valid = true;
    simulated_adc_saturated = true;
    pressure_update_current();
    assert(status.current_adc_saturated && status.overcurrent_tripped);
    assert_selected_targets();
    assert_all_loads_off();
}

void test_direct_manual_pwm_is_not_changed() {
    reset_controller();
    pressure_execute_command(PRESSURE_CMD_COMPRESSOR_PWM, 45);
    sensor_frame(0.5f);
    assert(manual_compressor && pwm3 == 45 && hardware_pwm[2] == 45);
    pressure_execute_command(PRESSURE_CMD_COMPRESSOR_PWM_NoInterrupt, 70);
    sensor_frame();
    status.state = PRESSURE_COMPRESSION;
    pressure_update();
    assert(!manual_compressor && compressor_pwm_user_set && pwm3 == 70 && hardware_pwm[2] == 70);
}
}

TickType_t xTaskGetTickCount() { return simulated_ticks; }
void pressure_hardware_init() {}
void pressure_pump1_set(uint8_t pwm) { hardware_pwm[0] = pwm; }
void pressure_pump2_set(uint8_t pwm) { hardware_pwm[1] = pwm; }
void pressure_compressor_set(uint8_t pwm) { hardware_pwm[2] = pwm; }
void pressure_valve_set(bool open) { hardware_valve = open; }
void pressure_relay_set(uint8_t relay, bool on) { hardware_relays[relay - 1] = on; }
bool pressure_current_read(uint16_t *current_ma, bool *saturated) {
    *current_ma = simulated_current_ma;
    *saturated = simulated_adc_saturated;
    return simulated_adc_valid;
}

int main() {
    test_defaults_remain_ambient_dependent();
    test_targets_survive_first_and_repeated_sensor_frames();
    test_unselected_targets_still_adapt_independently();
    test_later_commands_replace_targets_without_changing_limits();
    test_automatic_phases_still_control_outputs();
    test_stop_and_mode_changes_keep_selected_targets();
    test_current_cutoff_still_wins_and_recovery_keeps_targets();
    test_direct_manual_pwm_is_not_changed();
    std::puts("All 8 automatic PWM target regression tests passed.");
}
