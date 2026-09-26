# MIRAGE Experiment Overview

## What MIRAGE is
MIRAGE stands for Methane InfraRed Absorption Gas Experiment. It is a student experiment for BX39 that aims to demonstrate a low-cost, low-complexity alternative to laser-based methane sensing by using an off-the-shelf K96 non-dispersive infrared (NDIR) sensor core.

The experiment measures methane, carbon dioxide, and water vapor in the stratosphere. To make the sensor usable at low ambient pressure, MIRAGE pressurises ambient air to 3 bar before it enters the measurement chamber and keeps the sample temperature under control.

## Core experiment concept
The experiment is built around a pressurised measurement chamber containing the K96 sensor and supporting pressure, temperature, and humidity sensors. Ambient air is drawn in by vacuum pumps, pre-pressurised, compressed, and then regulated inside the chamber while the gas sample is continuously exchanged.

The main system blocks are:

- a pressurisation system with two vacuum pumps, a compressor, and a chamber outlet valve
- a thermal system with heaters, a cooler, and insulation
- a main MCU that coordinates data flow and mode switching
- dedicated thermal and pressure MCUs for subsystem control
- on-board data storage and E-Link telemetry

## Scientific aim
MIRAGE is intended to show that atmospheric methane measurements can be made with cheaper and simpler hardware than laser-based systems, while still producing scientifically useful data up to at least 20 km altitude. The team also wants to evaluate how far the sensor can be pushed using pressure control and machine-learning-based calibration.

## Main design targets
- measure CH4, CO2, and H2O with an NDIR sensor up to at least 20 km
- maintain about 3 bar inside the chamber up to at least 20 km
- keep the chamber temperature in a usable range for the sensor and electronics
- send data via E-Link while also storing all data on board
- operate autonomously if the ground link is lost

## Key project characteristics
The current design is compact, with an estimated mass of about 3 kg and an overall envelope of 200 x 200 x 150 mm. The team is still refining the design, but the current architecture already defines the main mechanical, electrical, thermal, software, and data-analysis flows needed for a BX39 flight.
