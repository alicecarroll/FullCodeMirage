#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Settings.h"
#include "watchdog.h"

/* This is the old implementation of feed_watchdog
void feed_watchdog(bool system_ok)
{
    static uint8_t counter = 0; // creates the variable once and keeps it in memory

    counter++; // increases every cycle

    if (counter >= 5) // threshold so it doesnt update too often, can be adjusted based on how long each cycle takes and how long the watchdog timeout is
    {
        counter = 0;

        if (system_ok)
        {
            static bool state = false;
            state = !state;
            gpio_set_level(Watchdog_PIN, state);
        }
    }
}
    */

void feed_watchdog(bool system_ok){
    if (system_ok){
        gpio_set_level(Watchdog_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(Watchdog_PIN, 0);
    }

}