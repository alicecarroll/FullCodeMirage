#include "main.h"
#include "pressure_hardware.h"
#include "pressure_protocol.h"
#include "freertos/FreeRTOS.h"

#include <assert.h>
#include <limits>

namespace {
TickType_t fake_ticks = 0;

void update_sensors(float chamber_bar, float inlet_absolute_bar,
                    float ambient_bar = 1.0f) {
    const float sensors[7] = {
        0.0f,
        inlet_absolute_bar - ambient_bar,
        chamber_bar,
        ambient_bar,
        0.0f,
        0.0f,
        0.0f,
    };
    pressure_update_external_sensors(sensors);
}

void update_invalid_inlet_sensor(float chamber_bar, float ambient_bar) {
    const float sensors[7] = {
        0.0f,
        std::numeric_limits<float>::quiet_NaN(),
        chamber_bar,
        ambient_bar,
        0.0f,
        0.0f,
        0.0f,
    };
    pressure_update_external_sensors(sensors);
}

void expect_outputs(uint8_t pump_pwm, uint8_t compressor_pwm, bool valve_open) {
    const PressureStatus status = pressure_get_status();
    assert(status.pump1_pwm == pump_pwm);
    assert(status.pump2_pwm == pump_pwm);
    assert(status.compressor_pwm == compressor_pwm);
    assert(status.valve_open == valve_open);
}
}

TickType_t xTaskGetTickCount() { return fake_ticks; }

void pressure_hardware_init() {}
void pressure_pump1_set(uint8_t) {}
void pressure_pump2_set(uint8_t) {}
void pressure_compressor_set(uint8_t) {}
void pressure_valve_set(bool) {}
void pressure_relay_set(uint8_t, bool) {}

int main() {
    // Automatic operation without even one real sensor frame fails safe.
    pressure_init();
    pressure_execute_command(PRESSURE_CMD_START_PRESSURISATION, 0);
    assert(pressure_get_status().state == PRESSURE_ERROR);
    assert(pressure_get_status().error == 3);
    expect_outputs(0, 0, false);
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_ERROR);
    assert(pressure_get_status().error == 3);
    expect_outputs(0, 0, false);

    pressure_init();
    update_sensors(1.0f, 1.0f);
    pressure_execute_command(PRESSURE_CMD_START_PRESSURISATION, 0);

    // Starting pressurisation must first charge the compressor inlet.
    assert(pressure_get_status().state == PRESSURE_PREPRESSURISATION);
    expect_outputs(100, 0, false);

    update_sensors(1.0f, 1.61f);
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_COMPRESSION);
    expect_outputs(0, 100, false);

    // An exhausted inlet charge below the chamber target starts another
    // prepressurisation/compression round.
    fake_ticks = 1000;
    update_sensors(1.5f, 1.04f);
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_PREPRESSURISATION);
    expect_outputs(100, 0, false);

    update_sensors(1.5f, 1.61f);
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_COMPRESSION);

    // A compression pulse is capped at seven seconds even if the inlet has
    // not yet reached its lower threshold.
    fake_ticks = 8000;
    update_sensors(1.8f, 1.4f);
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_PREPRESSURISATION);
    expect_outputs(100, 0, false);

    update_sensors(1.8f, 1.61f);
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_COMPRESSION);

    // Ten seconds of total compressor flow exceeds three chamber volumes.
    // At 3 bar the controller starts the quiet measurement phase.
    fake_ticks = 10000;
    update_sensors(3.0f, 1.4f);
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_MEASUREMENT);
    expect_outputs(0, 0, false);

    fake_ticks = 29999;
    update_sensors(3.0f, 1.4f);
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_MEASUREMENT);

    // After exactly 20 seconds the outlet opens for the next exchange.
    fake_ticks = 30000;
    update_sensors(3.0f, 1.4f);
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_AIR_EXCHANGE);
    expect_outputs(0, 0, true);

    update_sensors(2.1f, 1.0f);
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_AIR_EXCHANGE);

    // The outlet closes at the configured 2 bar floor before refilling.
    update_sensors(2.0f, 1.0f);
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_PREPRESSURISATION);
    expect_outputs(100, 0, false);

    // An already-full chamber is vented first instead of being compressed.
    pressure_execute_command(PRESSURE_CMD_STOP_PRESSURISATION, 0);
    update_sensors(3.0f, 1.0f);
    pressure_execute_command(PRESSURE_CMD_START_PRESSURISATION, 0);
    assert(pressure_get_status().state == PRESSURE_AIR_EXCHANGE);
    expect_outputs(0, 0, true);

    // Reaching 3 bar before enough fresh-air volume has been delivered
    // starts another vent/refill round instead of measuring stale gas.
    update_sensors(2.0f, 1.0f);
    pressure_update();
    update_sensors(2.0f, 1.61f);
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_COMPRESSION);
    fake_ticks = 31000;
    update_sensors(3.0f, 1.4f);
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_AIR_EXCHANGE);
    expect_outputs(0, 0, true);

    // Missing sensor frames stop every actuator and latch an error.
    fake_ticks = 34001;
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_ERROR);
    assert(pressure_get_status().error == 3);
    expect_outputs(0, 0, false);

    // Explicit inlet settings remain authoritative at low ambient pressure.
    pressure_set_compressor_inlet_upper_limit(1.4f);
    pressure_set_compressor_inlet_lower_limit(1.3f);
    update_sensors(1.0f, 1.2f, 0.1f);
    pressure_execute_command(PRESSURE_CMD_START_PRESSURISATION, 0);
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_PREPRESSURISATION);

    update_sensors(1.0f, 1.41f, 0.1f);
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_COMPRESSION);
    update_sensors(1.5f, 1.29f, 0.1f);
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_PREPRESSURISATION);

    // An explicitly invalid ABP2 value also stops the automatic cycle.
    update_invalid_inlet_sensor(1.5f, 0.1f);
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_ERROR);
    assert(pressure_get_status().error == 2);
    expect_outputs(0, 0, false);
}
