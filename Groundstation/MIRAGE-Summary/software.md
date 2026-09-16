# MIRAGE Software Design

## What the software must do
The on-board software is responsible for collecting sensor data, storing it on the SD card, handling communication with ground, and controlling the pressure and thermal subsystems. It is one of the most important parts of MIRAGE because it ties the experiment together and keeps the system safe when conditions change.

The implementation is planned in C++ so the team can get deterministic timing, low memory overhead, interrupt support, and direct hardware control.

## Top-level operating modes
The software is organized into two top-level modes:

- Manual mode for commands, debugging, and subsystem overrides
- Autonomous mode for normal flight operation

Autonomous mode contains Standby, Measurements, and Humidity loops. A Test Loop can be entered manually while keeping the safety-critical background services active, such as pre-charge, watchdog, and telemetry heartbeat.

If the ground link is lost, manual overrides are cleared and the controller falls back to autonomous behavior. Ambient pressure then decides whether the system should stay in Standby or move to Measurements.

## Main MCU responsibilities
The main MCU is the master controller. It manages mode transitions, command handling, telemetry packaging, SD-card scheduling, and plausibility checks on critical sensor data. It also keeps a watchdog heartbeat alive and uses queue-based producer-consumer patterns to decouple sensing, control, and data transfer.

## Pressure control software
The pressure MCU regulates the two vacuum pumps, the compressor, and the outlet valve to keep chamber pressure near 3 bar. It has separate behavior for nominal control, overpressure, and underpressure. If pressure cannot be recovered safely, the software triggers an emergency stop and reports the fault to ground.

## Thermal control software
The thermal MCU manages the chamber heater, intake-air heater, outlet heater, SD-card heater, and Peltier cooler. PID control is used for the thermal loops, with anti-windup and tunable gains. The humidity protection loop can close shutters, restrict pumping, and reheat the system until humidity returns to a safe level.

## Data handling and communications
All transmissions are timestamped and stored on board. The expected science data stream is about 40 kbit/s, which is small enough for the SD card and within the downlink budget. The design still stores everything locally and uses E-Link as a live telemetry channel and redundancy path.

The software uses I2C for most sensors, UART/Modbus for the K96 core, and UDP for the experiment-side communication logic. The SED also includes explicit handling for E-Link dropouts, reconnects, and bandwidth management.

## Manual operations and fault handling
The manual procedures are not an afterthought. The software defines actions for repeated pressure faults, frozen outlets, sensor failures, and MCU restarts. In other words, the code must be able to continue operating safely even when one sensor or one subsystem is not behaving normally.

## Design implication
Software is central to MIRAGE because the payload is not just a sensor in a box. It is a closed-loop chemical sampling and measurement system whose success depends on mode logic, fault handling, pressure control, thermal regulation, and robust telemetry.
