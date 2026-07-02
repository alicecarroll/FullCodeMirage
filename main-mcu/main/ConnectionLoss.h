#pragma once

// us = microseconds

/* Use this instead of external RTC since big time span
 and it will still work in case something would happen to
 the external RTC.
*/
#include "esp_timer.h" // for esp_timer_get_time()

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include "Settings.h"

/*
 * ConnectionLoss.h
 *
 * Tracks Ethernet connection loss and triggers experiment termination
 * if the connection is not reestablished within TERMINATION_TIMEOUT.
 *
 * The caller owns and passes in:
 *   loss_timestamp_us  — set to esp_timer_get_time() on loss, -1 when inactive
 *   con_lost           — true while connection is down
 *   mode               — set to 1 (power-save loop) on termination
 *   terminated         — set to true on termination
 */

#ifdef __cplusplus
extern "C"
{
#endif

    /*
     * Call when the Ethernet connection is lost.
     * Records the loss timestamp and sets con_lost = true.
     */
    esp_err_t connection_lost(bool *con_lost, int64_t *loss_timestamp_us);

    /*
     * Call when the Ethernet connection is restored.
     * Resets the timer and sets con_lost = false.
     */
    esp_err_t connection_reestablished(bool *con_lost, int64_t *loss_timestamp_us);

    /*
     * Poll this periodically (e.g. from the sensor loop or a FreeRTOS task).
     * Returns ESP_OK while within the timeout window.
     * Calls terminate_experiment() if TERMINATION_TIMEOUT is exceeded.
     * Does nothing and returns ESP_OK if timer is not active.
     */
    esp_err_t timer_status(int64_t *loss_timestamp_us, int *mode, bool *terminated);

    /*
     * Terminates the experiment: sets mode = 1 (power-save loop) and terminated = true.
     * Called internally by timer_status(); may also be called directly.
     */
    esp_err_t terminate_experiment(int64_t elapsed, int *mode, bool *terminated);

#ifdef __cplusplus
}
#endif
