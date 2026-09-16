# Telemetry Systems

## The two telemetry paths
BEXUS uses two telemetry systems:

- E-Link for experiment data to and from the ground
- EMPIRE, or EBASS if EMPIRE is unavailable, for SSC piloting and housekeeping data only

## E-Link
E-Link is the experiment-facing telemetry system. It provides a simplified Ethernet interface to the payload, so experiments can use a LAN-like connection without needing a custom radio system.

### Architecture and interface
The E-Link ground station contains an antenna, antenna controller, and Monitor and Control Unit. The airborne unit includes the main unit, antenna, battery, and RF interface unit. One connection is available to each experimenter, although teams may add an internal Ethernet switch if they need multiple connections.

The interface is standard Ethernet 10/100 Base-T and uses MIL-C-26482-MS3116F-12-10P connectors. The experiment-side panel connector is an Amphenol RJF21B with Insert Code A and a standard RJ45 inside.

### Main characteristics

- 2 Mbps duplex nominal bandwidth, decreasing with range
- S-band operation
- peak output power of 10 W on the airborne unit
- fixed IP address allocation
- FCC and ETSI-approved electrical parts
- 20 to 38 V DC supply
- nominal operation time greater than 11 hours
- nominal weight around 20 kg including batteries

### Operational constraints
The available bandwidth is shared by E-Link and all experiments, so there is no prioritisation. If too much data is sent at once, communication can be lost temporarily. Downlink coordination is therefore essential.

The system should also be treated as range-limited: performance declines with increasing distance from the gondola, even though the nominal line-of-sight range is up to 500 km at 30 km altitude.

## EMPIRE
EMPIRE is SSC’s balloon flight termination and housekeeping system. It is not used by experiments, and interference with it must be avoided. Its main functions are flight termination, a switchable ATC transponder, global tracking coverage, housekeeping data transmission, and Iridium SBD communication. It uses 1616-1626.5 MHz and has a peak power of 12 W when transmitting.

## EBASS fallback
EBASS is the older Balloon Service System and may still be used if EMPIRE is unavailable. It operates on 400-405 MHz downlink and 449.95 MHz uplink, with 2.5-5 W output power and a cross broadband dipole antenna.
