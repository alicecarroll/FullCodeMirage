#pragma once

#include <stdint.h>
#include <stdbool.h>

void pressure_hardware_init();
void pressure_pump1_set(uint8_t pwm);
void pressure_pump2_set(uint8_t pwm);
void pressure_compressor_set(uint8_t pwm);
void pressure_valve_set(bool open);
void pressure_relay_set(uint8_t relay, bool on);

