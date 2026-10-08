#include "pressure_hardware.h"
#include "main.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

namespace {
constexpr gpio_num_t VALVE_PIN = GPIO_NUM_9;
constexpr gpio_num_t COMPRESSOR_PIN = GPIO_NUM_10;
constexpr gpio_num_t PUMP1_PIN = GPIO_NUM_47;
constexpr gpio_num_t PUMP2_PIN = GPIO_NUM_38;
constexpr gpio_num_t RELAY_PINS[] = {GPIO_NUM_48, GPIO_NUM_2, GPIO_NUM_1, GPIO_NUM_21};
constexpr const char *TAG = "pressure_hw";
constexpr ledc_mode_t PWM_MODE = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t PWM_TIMER = LEDC_TIMER_0;
constexpr ledc_channel_t PUMP1_CHANNEL = LEDC_CHANNEL_0;
constexpr ledc_channel_t PUMP2_CHANNEL = LEDC_CHANNEL_1;
constexpr ledc_channel_t COMPRESSOR_CHANNEL = LEDC_CHANNEL_2;
constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 10;
constexpr uint32_t PWM_MAX_DUTY = (1U << PWM_RESOLUTION_BITS) - 1U;
constexpr int CURRENT_GPIO = 12; // native ESP32-S3 GPIO12, not Arduino D12
adc_oneshot_unit_handle_t current_adc = nullptr;
adc_cali_handle_t current_calibration = nullptr;
adc_channel_t current_channel;

void configure_current_sensor() {
    adc_unit_t unit;
    esp_err_t err = adc_oneshot_io_to_channel(CURRENT_GPIO, &unit, &current_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO12 ADC mapping failed: %s", esp_err_to_name(err));
        return;
    }
    adc_oneshot_unit_init_cfg_t init = {};
    init.unit_id = unit;
    err = adc_oneshot_new_unit(&init, &current_adc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Current ADC setup failed: %s", esp_err_to_name(err));
        return;
    }
    adc_oneshot_chan_cfg_t channel = {};
    channel.atten = ADC_ATTEN_DB_12;
    channel.bitwidth = ADC_BITWIDTH_12;
    err = adc_oneshot_config_channel(current_adc, current_channel, &channel);
    if (err == ESP_OK) {
        adc_cali_curve_fitting_config_t calibration = {};
        calibration.unit_id = unit;
        calibration.chan = current_channel;
        calibration.atten = channel.atten;
        calibration.bitwidth = channel.bitwidth;
        err = adc_cali_create_scheme_curve_fitting(&calibration, &current_calibration);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Current ADC calibration/setup failed: %s; relay activation will be blocked", esp_err_to_name(err));
        adc_oneshot_del_unit(current_adc);
        current_adc = nullptr;
        return;
    }
    ESP_LOGI(TAG, "Current sensor on GPIO12: 1 V/A, calibrated ADC; relay cutoff at 2.900 A");
}

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
    set_output(VALVE_PIN, 0);
    for (gpio_num_t pin : RELAY_PINS) set_output(pin, 0);

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
    configure_current_sensor();
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

bool pressure_current_read(uint16_t *current_ma, bool *adc_saturated) {
    if (!current_ma || !adc_saturated || !current_adc || !current_calibration) return false;
    int raw = 0;
    int millivolts = 0;
    if (adc_oneshot_read(current_adc, current_channel, &raw) != ESP_OK ||
        adc_cali_raw_to_voltage(current_calibration, raw, &millivolts) != ESP_OK) return false;
    // The specified sensor transfer is 1 V/A with no offset/divider.
    // Do not average: protection uses the very first over-limit sample.
    *current_ma = static_cast<uint16_t>(millivolts < 0 ? 0 : millivolts > UINT16_MAX ? UINT16_MAX : millivolts);
    *adc_saturated = raw >= 4095;
    return true;
}
