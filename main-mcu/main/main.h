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
