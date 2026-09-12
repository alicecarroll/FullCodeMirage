# MIRAGE Data Analysis Plan

## Analysis goal
The output of MIRAGE is not just raw sensor data. The intended result is a quality-controlled vertical methane concentration profile with uncertainty bounds as a function of altitude. That profile is the main scientific product of the mission.

## Inputs to the pipeline
The analysis pipeline uses all time-stamped sensor data stored on the SD card, with E-Link serving as a secondary copy and redundancy channel. The inputs include K96 gas measurements, internal and ambient pressure, internal and ambient humidity, temperature channels, pump and heater status, and the housekeeping data needed to interpret the flight context.

## Processing stages
The plan is to process the data in six stages:

- ingest and reconcile SD-card and E-Link data
- align everything to a common 10 Hz timeline
- calibrate the K96 output for methane and carbon dioxide using the calibration model
- derive altitude from pressure and cross-check it with GPS
- apply quality flags to mark or exclude bad measurements
- prepare the cleaned profile for comparison with independent references

## Quality control
The analysis assigns explicit quality flags for conditions such as warm-up, flushing, chamber pressure out of target range, ambient pressure saturation, high humidity, calibration-range mismatch, and E-Link data gaps. Some flagged data are excluded completely, while other points are retained with inflated uncertainty.

## Calibration plan
Calibration is a major part of the analysis because the K96 sensor is sensitive to temperature and humidity and has limited intrinsic accuracy. The team plans to train machine-learning models such as neural networks, LSTM, GRU, XGBoost, random forest, linear regression, or k-NN on calibration data gathered at Norunda. Interpolation-based alternatives are also being considered.

The calibration dataset will span the expected environmental range of the mission, especially humidity, temperature, and pressure. Norunda provides calibration gases, a dew point generator, and reference measurements through a Picarro system.

## Validation and comparison
The cleaned MIRAGE profile will be compared with radiative transfer models, chemical transport models, satellite retrievals, and older in-situ datasets such as the Esrange TDLAS measurements and AirCore profiles. The plan also distinguishes between a preliminary on-site overview mode and a later full validation mode.

## Design implication
The data analysis plan is tightly linked to the hardware design. Sampling rate, storage size, thermal and pressure control, and calibration strategy all need to support the final altitude profile and uncertainty calculation. If the data pipeline is not designed early, the science result will suffer even if the hardware works.
