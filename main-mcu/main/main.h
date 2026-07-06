#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief  Main application entry point for the ESP-IDF app.
     */
    void app_main(void);

    /**
     * @brief  Main loop executed repeatedly by the application.
     */
    void loop(void);

    /**
     * @brief  Process received Ethernet data and react to the receive status.
     *
     * @param[in] esp_err_status  ESP-IDF error status returned by wiz_receive().
     */
    void handle_ethernet_data(esp_err_t esp_err_status);

    /**
     * @brief  Interpret and execute a received command from the Ethernet buffer.
     */
    void handle_command(void);

#ifdef __cplusplus
}
#endif

static void print_sensor_data(const SensorData *data)
{
    if (data == nullptr)
    {
        return;
    }

    printf("Time: %02u:%02u:%02u\n", data->hours, data->minutes, data->seconds);
    printf("Temps: Tp1=%.2f C Tp2=%.2f C Tp3=%.2f C Tp4=%.2f C Tp5=%.2f C Tp6=%.2f C Tt1=%.2f C Tt2=%.2f C Tt3=%.2f C Ta1=%.2f C Ta2=%.2f C\n",
           data->Tp1, data->Tp2, data->Tp3, data->Tp4, data->Tp5, data->Tp6,
           data->Tt1, data->Tt2, data->Tt3, data->Ta1, data->Ta2);
    printf("Pressures: Pa1=%.2f Pp1=%.2f Pp2=%.2f Pp3=%.2f\n",
           data->Pa1, data->Pp1, data->Pp2, data->Pp3);
    printf("Ambient: Ha1=%.2f %% RH\n", data->Ha1);
    printf("K96: CO2=%.2f ppm pressure=%.2f hPa temp=%.2f C humidity=%.2f RH error=%u\n",
           data->K96_CO2, data->K96_pressure, data->K96_temperature,
           data->K96_humidity, data->K96_error);
}

//esp_err_t sd_err;