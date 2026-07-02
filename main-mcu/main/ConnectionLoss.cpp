//!!!!!!!!!!!!!!!!!
//To do: add error handling
//Make sure termination function logic is correct.
//Should we go back to main and then call in again instead?

//us = microseconds

#include "esp_log.h"
#include "Settings.h"

/* Use this instead of external RTC since big time span
 and it will still work in case something would happen to
 the external RTC. 
*/ 
#include "esp_timer.h"   // for esp_timer_get_time()

esp_err_t terminate_experiment(int64_t elapsed, int *mode, bool *terminated);

static const char *TAG = "Connection loss / termination timer";

esp_err_t connection_lost(bool *con_lost, int64_t *loss_timestamp_us){
    if (*loss_timestamp_us == -1) //Timer not running
    {  
        *loss_timestamp_us = esp_timer_get_time(); //in us
        ESP_LOGI(TAG, "Termination timer started at %lld us", *loss_timestamp_us);
    }
    
    *con_lost = true;
    ESP_LOGI(TAG, "Connection lost %d", *con_lost);

    return ESP_OK;
}

esp_err_t connection_reestablished(bool *con_lost, int64_t *loss_timestamp_us){
    int64_t elapsed = esp_timer_get_time() - *loss_timestamp_us;
    ESP_LOGI(TAG, "Termination timer stoped at %lld us", elapsed);
    
    *loss_timestamp_us = -1;  //Reset timer

    *con_lost = false;    
    ESP_LOGI(TAG, "Connection reesteablished %d", *con_lost);

    return ESP_OK;
}

esp_err_t timer_status(int64_t *loss_timestamp_us, int *mode, bool *terminated){
    if (*loss_timestamp_us == -1){
        ESP_LOGI(TAG, "Termination timer not active");
        return ESP_OK;
    }
    else{
        int64_t elapsed = esp_timer_get_time() - *loss_timestamp_us;
        if (elapsed > TERMINATION_TIMEOUT){
            ESP_LOGI(TAG, "Termination timer exceeded 30 min = %lld us", TERMINATION_TIMEOUT);
            
            *terminated = true;
            return terminate_experiment(elapsed, mode, terminated);
        }
        else{
            ESP_LOGI(TAG, "Termination timer status: %lld", elapsed);
            return ESP_OK;
        }
    }
}

esp_err_t terminate_experiment(int64_t elapsed, int *mode, bool *terminated){
    ESP_LOGI(TAG, "Experiment terminated after %lld us", elapsed);
    *mode = 1; //choose powr save loop
    *terminated = true;

    return ESP_OK;
}