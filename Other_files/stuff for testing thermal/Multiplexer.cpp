#include "driver/i2c.h"
#include "Settings.h"
#include "Multiplexer.h"

esp_err_t select_mux_channel(uint8_t channel)
{
    
    
    //uint8_t data = (1 << channel); // Replaced this lane with line below. As channel is defined as bits already the bit shifting isnt needed and might cause errors (Atleast according to Ai)
    uint8_t data = channel;

    return i2c_master_write_to_device(
        I2C_master,
        multiplex_addr,
        &data,
        1,
        100 / portTICK_PERIOD_MS);
}