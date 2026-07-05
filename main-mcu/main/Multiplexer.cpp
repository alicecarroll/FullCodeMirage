#include "driver/i2c.h"
#include "Settings.h"
#include "Multiplexer.h"

esp_err_t sel_mux_channel(uint8_t channel)
{
    // First, send 0x00 to turn OFF all channels and clear the mux state
    uint8_t disable_data = 0x00;
    i2c_master_write_to_device(
        I2C_master,
        multiplex_addr,
        &disable_data,
        1,
        100 / portTICK_PERIOD_MS);

    // Short delay to let the bus settle (optional but safe)
    vTaskDelay(pdMS_TO_TICKS(1));

    // Now, open ONLY the requested channel
    uint8_t data = (1 << channel);
    return i2c_master_write_to_device(
        I2C_master,
        multiplex_addr,
        &data,
        1,
        100 / portTICK_PERIOD_MS);
}