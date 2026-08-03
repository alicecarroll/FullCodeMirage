#include "pressure_hardware.h"
#include "main.h"
#include "driver/gpio.h"
#include "esp_log.h"

namespace {
constexpr gpio_num_t VALVE_PIN = GPIO_NUM_9;
constexpr gpio_num_t COMPRESSOR_PIN = GPIO_NUM_10;
constexpr gpio_num_t PUMP1_PIN = GPIO_NUM_47;
constexpr gpio_num_t PUMP2_PIN = GPIO_NUM_38;
constexpr gpio_num_t RELAY_PINS[] = {GPIO_NUM_21, GPIO_NUM_48, GPIO_NUM_1, GPIO_NUM_2};
PressureStatus *g_status = nullptr;
constexpr const char *TAG = "pressure_hw";

void set_output(gpio_num_t pin, int level) {
    const esp_err_t err = gpio_set_level(pin, level);
    if (err != ESP_OK) ESP_LOGE(TAG, "GPIO %d failed: %s", pin, esp_err_to_name(err));
}
}

void pressure_hardware_init() {
    uint64_t mask = (1ULL << VALVE_PIN) | (1ULL << COMPRESSOR_PIN) |
                    (1ULL << PUMP1_PIN) | (1ULL << PUMP2_PIN);
    for (gpio_num_t pin : RELAY_PINS) mask |= (1ULL << pin);
    gpio_config_t config = {};
    config.mode = GPIO_MODE_OUTPUT;
    config.pin_bit_mask = mask;
    if (gpio_config(&config) != ESP_OK) ESP_LOGE(TAG, "actuator GPIO setup failed");
}

void pressure_pump1_set(uint8_t pwm) { set_output(PUMP1_PIN, pwm != 0); }
void pressure_pump2_set(uint8_t pwm) { set_output(PUMP2_PIN, pwm != 0); }
void pressure_compressor_set(uint8_t pwm) { set_output(COMPRESSOR_PIN, pwm != 0); }
void pressure_valve_set(bool open) { set_output(VALVE_PIN, open); }
void pressure_relay_set(uint8_t relay, bool on) {
    if (relay < 1 || relay > 4) return;
    set_output(RELAY_PINS[relay - 1], on);
}

