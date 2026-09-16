#include "main.h"
#include "pressure_hardware.h"
#include "pressure_protocol.h"
#include "freertos/FreeRTOS.h"

#include <assert.h>

namespace {
TickType_t fake_ticks = 0;

void update_sensors(float chamber_bar, float inlet_absolute_bar) {
    constexpr float ambient_bar = 1.0f;
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
    pressure_init();
    update_sensors(1.0f, 1.0f);
    pressure_execute_command(PRESSURE_CMD_START_PRESSURISATION, 0);

    // Starting pressurisation must first charge the compressor inlet.
    assert(pressure_get_status().state == PRESSURE_PREPRESSURISATION);
    expect_outputs(50, 0, false);

    update_sensors(1.0f, 1.61f);
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_COMPRESSION);
    expect_outputs(0, 50, false);

    // An exhausted inlet charge below the chamber target starts another
    // prepressurisation/compression round.
    fake_ticks = 1000;
    update_sensors(1.5f, 1.1f);
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_PREPRESSURISATION);
    expect_outputs(50, 0, false);

    update_sensors(1.5f, 1.61f);
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
    pressure_update();
    assert(pressure_get_status().state == PRESSURE_MEASUREMENT);

    // After exactly 20 seconds the outlet opens for the next exchange.
    fake_ticks = 30000;
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
    expect_outputs(50, 0, false);

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
}
