#pragma once
#include <stdint.h>
#include "esp_err.h"

#pragma pack(push, 1)
struct SensorData 
{
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;

    // hej. Hallo. 

    float Tp1;           // Vacum pump 1 temperature
    float Tp2;           // Vacum pump 2 temperature
    float Tp3;           // Compressor temperature
    float Tp6;           // Pipe pump/pump temperature
    float Pp3;           // Pipe pump/pump preassure
    float Tp4;           // Pipe pump/compressor temperature
    float Pp1;           // Pipe pump/compressor preassure
    float Pa1;           // Pressure in MS580314BA01
    float Ta1;           // Temperature in ambient
    float Ta2;           // Temperature in ambient from MS580314BA01
    float Ta3;           // Temperature in ambient from SHT45
    float Ha1;           // Humidity in ambient
    float Tp5;           // Temperature in meassurment chamber
    float Pp2;           // Pressure in meassurment chamber
    float Tt1;           // Temperature in outlet air
    float Tt2;           // Temperature in SD-card
    float Tt3;           // Temperature in inlet air

    // ----- K96 Sensor Fields -----
    int32_t  K96_LPL_Signal;
    double   K96_LPL_Signal_filtered;
    int32_t  K96_SPL_Signal;
    double   K96_SPL_Signal_filtered;
    int32_t  K96_MPL_Signal;
    double   K96_MPL_Signal_filtered;

    float    K96_ADuCdie_Temp;
    float    K96_ADuCdie_Temp_filtered;
    float    K96_NTC0_Temp;
    float    K96_NTC0_Temp_filtered;
    float    K96_NTC1_Temp;
    float    K96_NTC1_Temp_filtered;
    
    float    K96_RH;
    float    K96_RH_Temp;

    uint16_t K96_MPL_uflt_IR_Signal;
    uint16_t K96_MPL_flt_IR_Signal;
    float    K96_MPL_uflt_Conc;
    float    K96_MPL_flt_Conc;
    uint16_t K96_MPL_uflt_Error;

    uint16_t K96_LPL_uflt_IR_Signal;
    uint16_t K96_LPL_flt_IR_Signal;
    float    K96_LPL_uflt_Conc;
    uint16_t K96_LPL_uflt_Error;
    uint16_t K96_LPL_flt_Error;

    uint16_t K96_SPL_uflt_IR_Signal;
    uint16_t K96_SPL_flt_IR_Signal;
    float    K96_SPL_uflt_Conc;
    uint16_t K96_SPL_uflt_Error;
    uint16_t K96_SPL_flt_Error;

    uint16_t K96_error;  // General error code for K96 sensor
};
#pragma pack(pop)

// This is part of the on-wire status protocol. Keep the groundstation format
// in sync with this exact packed size.
static_assert(sizeof(SensorData) == 179, "SensorData wire layout changed");

extern SensorData sensor_data;

enum class MS5803Model : uint8_t {
    MS5803_01BA,
    MS5803_14BA,
};
struct MS5803_Calibration
{
    uint16_t C[7];
    MS5803Model model;
};

extern MS5803_Calibration pa1_cal;
extern MS5803_Calibration pp2_cal;

void read_ms5803(
    MS5803_Calibration *cal,
    float *pressure,
    float *temperature);


void read_sensors();


    /**
 * @brief  Creating a csv file header
 *
 * @param[in] file name   
 *
 * @return void
 */
void write_csv_header(FILE *f);

    /**
 * @brief  writing sensor data into line of file
 *
 * @param[in] file name    
 * @param[in] sensor data
 *
 * @return void
 */
void write_csv_row(FILE *f, const SensorData *data);

    /**
 * @brief  logging sensor data into file
 *
 * @param[in] sensor data    
 * @param[in] file path
 *
 * @return void
 */
void log_sensor_data(const SensorData *data, const char *filepath);
