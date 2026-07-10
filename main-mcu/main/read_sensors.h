#pragma once
#include <stdint.h>

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
    float Ta2;           // Temperature in ambient from SHT45
    float Ha1;           // Humidity in ambient
    float Tp5;           // Temperature in meassurment chamber
    float Pp2;           // Pressure in meassurment chamber
    float Tt1;           // Temperature in outlet air
    float Tt2;           // Temperature in SD-card
    float Tt3;           // Temperature in inlet air

    float K96_CO2;           // CO2 concentration [ppm]
    float K96_pressure;      // Internal pressure [hPa]
    float K96_temperature;   // Internal temperature [°C]
    float K96_humidity;      // Internal humidity [%RH]
    uint16_t K96_error;      // K96 error/status flags
};

extern SensorData sensor_data;

struct MS5803_Calibration
{
    uint16_t C[7];
};

extern MS5803_Calibration pa1_cal;
extern MS5803_Calibration pp2_cal;

static void read_ms5803(MS5803_Calibration* cal, float* pressure);

void read_sensors();

void write_csv_header(FILE *f);
void write_csv_row(FILE *f, const SensorData *data);
void log_sensor_data(const SensorData *data, const char *filepath);
