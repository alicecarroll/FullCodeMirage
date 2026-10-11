#include <cassert>
#include <cstdio>

// Exercise the real controller with a simulated clock and no hardware access.
#include "../../main/pressure_control.cpp"

namespace {
TickType_t simulated_ticks = 100;
uint8_t hardware_pwm[3] = {};
bool hardware_valve = false;
bool hardware_relays[4] = {};
uint16_t simulated_current_ma = 0;
bool simulated_adc_valid = true, simulated_adc_saturated = false;

void reset_controller(PressureState state, float chamber, float inlet) {
    simulated_ticks = 100;
    status = {};
    status.current_valid = true;
    simulated_current_ma = 0;
    simulated_adc_valid = true;
    simulated_adc_saturated = false;
    for (bool &relay : hardware_relays) relay = false;
    applied_mode = PRESSURE_MODE_STANDBY;
    current_shutdown_standby = false;
    status.state = state;
    status.chamber_pressure = chamber;
    status.compressor_inlet_pressure = inlet;
    status.ambient_pressure = 0.99f;
    target_pressure = 3.2f;
    compressor_safe_start_inlet_upper = 1.7f;
    clear_overrides();
    inlet_start_wait_active = false;
    inlet_start_wait_started = 0;
    external_sensors_valid = true;
    pwm_targets_initialized = true;
    sensor_history_valid = true;
    sensor_update_interval = pdMS_TO_TICKS(100);
    last_sensor_update = simulated_ticks;
    previous_chamber_pressure = chamber;
    previous_compressor_inlet_pressure = inlet;
    flushstep_start = simulated_ticks;
    flushsum = 0.0f;
    pwm1 = 20;
    pwm2 = 27;
    pwm3 = 34;
    set_pump1(0);
    set_pump2(0);
    set_compressor(0);
    set_valve(false);
}

void advance_ms(uint32_t ms) {
    simulated_ticks += pdMS_TO_TICKS(ms);
    // Simulate fresh, unchanged pressure samples throughout the wait.
    last_sensor_update = simulated_ticks;
}

void assert_pwm_targets_unchanged() {
    assert(pwm1 == 20 && pwm2 == 27 && pwm3 == 34);
}

void test_floating_point_target_band() {
    for (const float chamber : {2.21f, 3.0f, 3.4f, 4.19f}) {
        reset_controller(PRESSURE_COMPRESSION, chamber, 1.2f);
        assert(!chamber_at_target());
    }
    for (const float chamber : {3.15f, 3.2f, 3.25f}) {
        reset_controller(PRESSURE_COMPRESSION, chamber, 1.2f);
        assert(chamber_at_target());
    }
    reset_controller(PRESSURE_COMPRESSION, 2.21f, 1.2f);
    flushsum = 1.0f;
    pressure_update();
    assert(status.state == PRESSURE_COMPRESSION);
    assert(hardware_pwm[2] == 34);
    assert_pwm_targets_unchanged();

    reset_controller(PRESSURE_AIR_EXCHANGE, 2.21f, 1.2f);
    flushsum = 1.0f;
    pressure_update();
    assert(status.state == PRESSURE_AIR_EXCHANGE);
    assert(hardware_valve);

    reset_controller(PRESSURE_CORRECTION, 2.21f, 0.94f);
    pressure_update();
    assert(status.state == PRESSURE_PREPRESSURISATION);
}

void test_prefill_margin_and_immediate_stop() {
    reset_controller(PRESSURE_PREPRESSURISATION, 2.21f, 1.49f);
    pressure_update();
    assert(status.state == PRESSURE_PREPRESSURISATION);
    assert(hardware_pwm[0] == 20 && hardware_pwm[1] == 27);

    status.compressor_inlet_pressure = 1.5f;
    pressure_update();
    assert(status.state == PRESSURE_COMPRESSION);
    assert(hardware_pwm[0] == 0 && hardware_pwm[1] == 0);
    assert(hardware_pwm[2] == 0 && !hardware_valve);

    // Previously observed overshoot fits below the unchanged safety ceiling.
    status.compressor_inlet_pressure = 1.64f;
    advance_ms(20);
    pressure_update();
    assert(status.state == PRESSURE_COMPRESSION);
    assert(hardware_pwm[2] == 34);
    assert(!inlet_start_wait_active);
    assert(compressor_safe_start_inlet_upper == 1.7f);
    assert_pwm_targets_unchanged();
}

void test_high_inlet_timeout_and_recovery() {
    reset_controller(PRESSURE_COMPRESSION, 2.21f, 1.8f);
    pressure_update();
    assert(inlet_start_wait_active && hardware_pwm[2] == 0);
    advance_ms(6990);
    pressure_update();
    assert(status.state == PRESSURE_COMPRESSION);
    advance_ms(10);
    pressure_update();
    assert(status.state == PRESSURE_ERROR);
    assert(status.error == ERR_INLET_START_TIMEOUT);
    assert(hardware_pwm[0] == 0 && hardware_pwm[1] == 0 && hardware_pwm[2] == 0);
    assert(!hardware_valve);
    assert_pwm_targets_unchanged();

    reset_controller(PRESSURE_COMPRESSION, 2.21f, 1.8f);
    pressure_update();
    advance_ms(3000);
    status.compressor_inlet_pressure = 1.6f;
    pressure_update();
    assert(status.state == PRESSURE_COMPRESSION);
    assert(!inlet_start_wait_active && hardware_pwm[2] == 34);
    assert(flushstep_start == simulated_ticks);
    assert(flushsum == 0);

    // A subsequent wait starts a fresh seven-second deadline.
    advance_ms(10);
    status.compressor_inlet_pressure = 1.8f;
    pressure_update();
    assert(inlet_start_wait_started == simulated_ticks);
    advance_ms(6000);
    pressure_update();
    assert(status.state == PRESSURE_COMPRESSION);
}

void test_measurement_then_air_exchange() {
    reset_controller(PRESSURE_COMPRESSION, 3.2f, 1.2f);
    flushsum = 1.0f;
    pressure_update();
    assert(status.state == PRESSURE_MEASUREMENT);
    assert(!hardware_valve && hardware_pwm[2] == 0);
    advance_ms(19990);
    pressure_update();
    assert(status.state == PRESSURE_MEASUREMENT && !hardware_valve);
    advance_ms(10);
    pressure_update();
    assert(status.state == PRESSURE_AIR_EXCHANGE && hardware_valve);
    assert(hardware_pwm[0] == 0 && hardware_pwm[1] == 0 && hardware_pwm[2] == 0);
    assert_pwm_targets_unchanged();

    status.chamber_pressure = 2.0f;
    pressure_update();
    assert(status.state == PRESSURE_PREPRESSURISATION && !hardware_valve);
}

void assert_all_loads_off() {
    for (bool relay : hardware_relays) assert(!relay);
    for (uint8_t pwm : hardware_pwm) assert(pwm == 0);
    assert(status.relay_mask == 0 && !hardware_valve);
    assert_pwm_targets_unchanged();
}

void test_exact_current_cutoff_and_manual_recovery() {
    reset_controller(PRESSURE_COMPRESSION, 3.2f, 1.2f);
    applied_mode = PRESSURE_MODE_MEASUREMENTS;
    for (uint8_t relay = 1; relay <= 4; ++relay) set_relay(relay, true);
    manual_pump1 = manual_pump2 = manual_compressor = manual_valve = relay_manual = true;
    set_pump1(20); set_pump2(27); set_compressor(34); set_valve(true);
    simulated_current_ma = 2899;
    pressure_update_current();
    assert(status.current_valid && status.current_ma == 2899 && !status.overcurrent_tripped);
    assert(status.relay_mask == 0x0f && hardware_pwm[2] == 34);

    simulated_current_ma = 2900;
    pressure_update_current();
    assert(status.overcurrent_tripped && status.state == PRESSURE_STANDBY && status.error == ERR_OVERCURRENT);
    assert_all_loads_off();
    // An unsafe reading still blocks every command, without requiring reset.
    for (unsigned command = 1; command <= PRESSURE_CMD_COMPRESSOR_PWM_NoInterrupt; ++command) {
        pressure_execute_command(static_cast<uint8_t>(command), 100);
        assert_all_loads_off();
        assert(status.overcurrent_tripped && status.state == PRESSURE_STANDBY && status.error == ERR_OVERCURRENT);
    }
    // Public legacy relay helpers cannot bypass a presently unsafe reading.
    pdb_relay1_on(); pdb_relay2_on(); pdb_relay3_on(); pdb_relay4_on();
    pressure_update();
    assert_all_loads_off();

    simulated_current_ma = 0; // opening the relays naturally removes current
    pressure_update_current();
    assert(!status.overcurrent_tripped && status.error == ERR_NONE && status.state == PRESSURE_STANDBY);
    for (unsigned i = 0; i < 10; ++i) {
        pressure_execute_command(PRESSURE_CMD_SET_MODE, PRESSURE_MODE_MEASUREMENTS);
        pressure_update();
        assert_all_loads_off();
    }

    // Manual continuation works directly, with no MCU reset or mode cycle.
    for (uint8_t relay = 1; relay <= 4; ++relay) set_relay(relay, true);
    pressure_execute_command(PRESSURE_CMD_PUMP1_ON, 0);
    pressure_execute_command(PRESSURE_CMD_PUMP2_ON, 0);
    pressure_execute_command(PRESSURE_CMD_COMPRESSOR_ON, 0);
    pressure_execute_command(PRESSURE_CMD_VALVE_OPEN, 0);
    pressure_update();
    assert(status.state == PRESSURE_STANDBY && status.relay_mask == 0x0f);
    assert(hardware_pwm[0] == 20 && hardware_pwm[1] == 27 && hardware_pwm[2] == 34 && hardware_valve);
    assert_pwm_targets_unchanged();

    // Another over-limit sample stops even these manually restarted loads.
    simulated_current_ma = 3000;
    pressure_update_current();
    assert(status.state == PRESSURE_STANDBY && status.overcurrent_tripped);
    assert_all_loads_off();
    simulated_current_ma = 0;
    pressure_update_current();
    pressure_execute_command(PRESSURE_CMD_START_PRESSURISATION, 0);
    pressure_update();
    assert(!current_shutdown_standby && status.state != PRESSURE_STANDBY);
    assert((status.relay_mask & 0x06) == 0x06);
    assert_pwm_targets_unchanged();
}

void test_invalid_adc_and_saturation() {
    reset_controller(PRESSURE_COMPRESSION, 2.2f, 1.2f);
    applied_mode = PRESSURE_MODE_MEASUREMENTS;
    set_relay(4, true); set_compressor(34);
    simulated_adc_valid = false;
    pressure_update_current();
    assert(!status.current_valid && !status.overcurrent_tripped && status.error == ERR_CURRENT_SENSOR && status.state == PRESSURE_STANDBY);
    assert_all_loads_off();
    pressure_execute_command(PRESSURE_CMD_SET_MODE, PRESSURE_MODE_MEASUREMENTS);
    pressure_execute_command(PRESSURE_CMD_RELAY4_ON, 0);
    assert_all_loads_off();
    simulated_adc_valid = true;
    pressure_update_current();
    assert(status.state == PRESSURE_STANDBY && status.error == ERR_NONE);
    assert_all_loads_off();
    // Main repeats its current mode; recovery must not treat that as a restart.
    pressure_execute_command(PRESSURE_CMD_SET_MODE, PRESSURE_MODE_MEASUREMENTS);
    assert(status.state == PRESSURE_STANDBY);
    assert_all_loads_off();
    simulated_adc_saturated = true;
    pressure_update_current();
    assert(status.current_adc_saturated && status.overcurrent_tripped);
    assert_all_loads_off();
    simulated_adc_saturated = false;
    pressure_update_current();
    assert(!status.overcurrent_tripped && status.state == PRESSURE_STANDBY);
    pressure_execute_command(PRESSURE_CMD_RELAY4_ON, 0);
    assert(hardware_relays[3]);
}

void test_mode_cycle_recovery_and_boot_trip() {
    for (const uint8_t initial_mode : {static_cast<uint8_t>(PRESSURE_MODE_MEASUREMENTS), static_cast<uint8_t>(0)}) {
        reset_controller(PRESSURE_COMPRESSION, 3.2f, 1.2f);
        applied_mode = initial_mode;
        simulated_current_ma = 2900;
        pressure_update_current();
        simulated_current_ma = 0;
        pressure_update_current();
        pressure_execute_command(PRESSURE_CMD_SET_MODE, PRESSURE_MODE_MEASUREMENTS);
        pressure_update();
        assert(status.state == PRESSURE_STANDBY && current_shutdown_standby);
        assert_all_loads_off();

        pressure_execute_command(PRESSURE_CMD_SET_MODE, PRESSURE_MODE_STANDBY);
        pressure_execute_command(PRESSURE_CMD_SET_MODE, PRESSURE_MODE_MEASUREMENTS);
        pressure_update();
        assert(!current_shutdown_standby && status.state != PRESSURE_STANDBY);
        assert((status.relay_mask & 0x06) == 0x06);
        assert_pwm_targets_unchanged();
    }
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
    test_floating_point_target_band();
    test_prefill_margin_and_immediate_stop();
    test_high_inlet_timeout_and_recovery();
    test_measurement_then_air_exchange();
    test_exact_current_cutoff_and_manual_recovery();
    test_invalid_adc_and_saturation();
    test_mode_cycle_recovery_and_boot_trip();
    std::puts("All pressure controller regression tests passed.");
}
