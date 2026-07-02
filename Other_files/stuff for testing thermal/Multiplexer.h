#pragma once

#include "esp_err.h" //I dunno if this is needed

esp_err_t select_mux_channel(uint8_t channel); //This was void in here before but defined as esp_err_t in the cpp file