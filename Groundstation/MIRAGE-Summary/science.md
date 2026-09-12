# MIRAGE Scientific Background

## Why the science matters
MIRAGE is driven by the need for better in-situ methane measurements in the stratosphere. Methane is a major climate-relevant gas, and satellite remote sensing still depends heavily on atmospheric models to infer vertical concentration profiles from column measurements. MIRAGE is meant to help close that gap with direct balloon-borne measurements.

## Why NDIR was chosen
Conventional methane profiling often uses laser-based techniques such as TDLAS or FTIR. Those methods can be highly accurate, but they are expensive, complex, and sensitive to alignment, vibration, and shock. NDIR sensing is much simpler and cheaper, which is why MIRAGE tests whether a modern NDIR sensor can be used for atmospheric methane profiling instead.

The K96 sensor is especially important because it can measure CH4, CO2, and H2O in the same core. That helps compensate for water and carbon dioxide interference, which are major problems for methane sensing with broadband infrared methods.

## Why pressurisation is needed
The sensor limit of detection is around 3 ppm at atmospheric conditions, but methane concentration effectively becomes harder to detect at balloon altitudes because the air density drops rapidly with pressure. MIRAGE therefore pressurises the sample to 3 bar so that the methane signal remains measurable even when the ambient atmosphere is much thinner. The concept is not to change the gas composition, but to increase the particle concentration in the optical path.

## Scientific objectives
The experiment is trying to:

- measure atmospheric CH4, CO2, and H2O up to at least 20 km
- demonstrate that a 3 bar pressurisation scheme is feasible at stratospheric altitude
- validate the measured profiles against chemical transport models, satellite data, and previous in-situ measurements
- evaluate whether AI-based calibration can reduce uncertainty in the field

## Calibration and validation context
The SED explicitly treats calibration as part of the science case. The K96 sensor is sensitive to temperature, humidity, and pressure drift, so the experiment plans to use calibration models, including machine-learning approaches, to correct the raw output. Reference data from Norunda, Picarro measurements, satellite products, and prior balloon or aircraft datasets are intended for validation.

In short, the science is not only about making one methane profile. It is about proving that a simple NDIR-based system can produce usable stratospheric greenhouse-gas data if the pressure, thermal, and calibration problems are handled correctly.
