#include "pressure_hardware.h"
#include "main.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

namespace {
constexpr gpio_num_t VALVE_PIN = GPIO_NUM_9;
constexpr gpio_num_t COMPRESSOR_PIN = GPIO_NUM_10;
constexpr gpio_num_t PUMP1_PIN = GPIO_NUM_47;
constexpr gpio_num_t PUMP2_PIN = GPIO_NUM_38;
constexpr gpio_num_t RELAY_PINS[] = {GPIO_NUM_21, GPIO_NUM_48, GPIO_NUM_1, GPIO_NUM_2};
constexpr const char *TAG = "pressure_hw";
constexpr ledc_mode_t PWM_MODE = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t PWM_TIMER = LEDC_TIMER_0;
constexpr ledc_channel_t PUMP1_CHANNEL = LEDC_CHANNEL_0;
constexpr ledc_channel_t PUMP2_CHANNEL = LEDC_CHANNEL_1;
constexpr ledc_channel_t COMPRESSOR_CHANNEL = LEDC_CHANNEL_2;
constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 10;
constexpr uint32_t PWM_MAX_DUTY = (1U << PWM_RESOLUTION_BITS) - 1U;

void set_output(gpio_num_t pin, int level) {
    const esp_err_t err = gpio_set_level(pin, level);
    if (err != ESP_OK) ESP_LOGE(TAG, "GPIO %d failed: %s", pin, esp_err_to_name(err));
}

void configure_pwm_channel(gpio_num_t pin, ledc_channel_t channel) {
    ledc_channel_config_t config = {};
    config.gpio_num = pin;
    config.speed_mode = PWM_MODE;
    config.channel = channel;
    config.timer_sel = PWM_TIMER;
    config.duty = 0;
    config.hpoint = 0;
    if (ledc_channel_config(&config) != ESP_OK) ESP_LOGE(TAG, "PWM channel setup failed");
}
}

void pressure_hardware_init() {
    // LEDC owns the three pump pins; configuring them here as ordinary GPIO
    // outputs makes the LEDC driver report a pin conflict at startup.
    uint64_t mask = (1ULL << VALVE_PIN);
    for (gpio_num_t pin : RELAY_PINS) mask |= (1ULL << pin);
    gpio_config_t config = {};
    config.mode = GPIO_MODE_OUTPUT;
    config.pin_bit_mask = mask;
    if (gpio_config(&config) != ESP_OK) ESP_LOGE(TAG, "actuator GPIO setup failed");

    ledc_timer_config_t timer = {};
    timer.speed_mode = PWM_MODE;
    timer.timer_num = PWM_TIMER;
    timer.duty_resolution = LEDC_TIMER_10_BIT;
    timer.freq_hz = PWM_FREQUENCY_HZ;
    timer.clk_cfg = LEDC_AUTO_CLK;
    if (ledc_timer_config(&timer) != ESP_OK) ESP_LOGE(TAG, "PWM timer setup failed");

    configure_pwm_channel(PUMP1_PIN, PUMP1_CHANNEL);
    configure_pwm_channel(PUMP2_PIN, PUMP2_CHANNEL);
    configure_pwm_channel(COMPRESSOR_PIN, COMPRESSOR_CHANNEL);
}

void set_pwm(ledc_channel_t channel, uint8_t pwm) {
    const uint32_t duty = (static_cast<uint32_t>(pwm) * PWM_MAX_DUTY) / 100U;
    ledc_set_duty(PWM_MODE, channel, duty);
    ledc_update_duty(PWM_MODE, channel);
}
void pressure_pump1_set(uint8_t pwm) { set_pwm(PUMP1_CHANNEL, pwm); }
void pressure_pump2_set(uint8_t pwm) { set_pwm(PUMP2_CHANNEL, pwm); }
void pressure_compressor_set(uint8_t pwm) { set_pwm(COMPRESSOR_CHANNEL, pwm); }
void pressure_valve_set(bool open) { set_output(VALVE_PIN, open); }
void pressure_relay_set(uint8_t relay, bool on) {
    if (relay < 1 || relay > 4) return;
    set_output(RELAY_PINS[relay - 1], on);
}
