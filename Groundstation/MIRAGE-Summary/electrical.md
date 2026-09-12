# MIRAGE Electrical Design and Interfaces

## Power architecture
MIRAGE expects to use the 28.8 V SAFT MP176065 battery supply from the gondola. A Power Distribution Board handles regulation, current monitoring, pre-charge, and distribution to the experiment subsystems.

The power rails are derived for different loads: 24 V for the pumps and compressor, 12 V for heating elements, 8 V for the K96 sensor, and 3.3 V for sensors, MCUs, and logic.

## Main electronics architecture
The electronics are centered around an ESP32-based main MCU. Two dedicated Arduino Nano ESP32 boards act as subsystem controllers for thermal and pressure control. The main MCU handles sensor acquisition, mode control, telemetry, SD-card storage, and safety supervision.

The sensor and control stack includes:

- K96 NDIR sensor for CH4, CO2, and H2O
- SHT45 environmental sensors for temperature and humidity
- MS5803 and ABP2 pressure sensors
- TMP1075 and TMP117 temperature monitoring
- ACS712 current sensing
- RTC and microSD storage
- TCA9548A I2C multiplexer

## Interfaces to BEXUS
The power connector is the BEXUS MIL-C 26482P series 1 interface, with the team using pins A and B for supply. The data connector is the Amphenol RJF21B used for E-Link Ethernet communication.

The experiment communicates with the K96 core over UART/Modbus, while most other sensors are on I2C. The software layer then moves the data over UDP and through the E-Link link to the ground station.

## Safety and electrical protection
The design includes a pre-charge circuit, an LC filter to limit EMI from switching converters, overcurrent protection, and solid-state relay control of the heavier loads. The PDB grounds all subsystems to the battery negative line.

The SED also flags the need to manage battery protection, current peaks, thermal hot spots around pumps and heaters, and condensation risk around the electronics and storage media.

## Electrical performance targets
The current design estimates about 0.46 A average current and about 98.45 Wh total use for the flight, with a peak current of about 2.30 A when thermal control and pressurisation are active during ascent. The system is designed to stay under the 150 Wh mission limit and to avoid exceeding the connector and battery current constraints.

## Practical design implication
Electrical design is not just a wiring exercise here. It drives the thermal budget, the safety case, the telemetry performance, and even the mechanical layout, because the pressure system, heaters, sensors, and storage all compete for space, power, and connector access.
