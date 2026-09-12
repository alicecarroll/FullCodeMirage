# Electrical and Electronics System

## Power interface
Each experiment must have an external power connector, even if it primarily uses internal batteries. The required connector is a 4-pin male box-mount receptacle, MIL-C-26482P series 1, with an 8-4 insert arrangement.

Pin assignments are:

- Pin A: positive
- Pin B: negative, not connected to chassis or ground
- Pin C: empty
- Pin D: empty

## Flight power and batteries
SSC can supply a 28.8 V battery pack if needed. The standard pack uses eight SAFT LSH20 cells in series, includes a built-in 5 A fuse on each cell, and has a recommended continuous maximum current draw of 1.8 A. The packs are only loosely thermally insulated and have been measured as low as -40 C during float.

If a team needs an alternative battery system or a second battery pack, it must be discussed with SSC before the Critical Design Review.

The team must budget for power during at least 2 hours of testing, 2 hours on the ground, and 6 hours of flight, with extra margin for repeated countdown attempts and periods when the experiment is powered but inaccessible.

## E-Link experiment interface
The experiment-side E-Link connector is a panel-mounted Amphenol RJF21B. SSC supplies the RJ45 link to the experiment, and the connection is transparent from the user point of view. Since E-Link is Ethernet-based, experiment software should be able to recover from Ethernet timeouts and reconnect cleanly.

## Ground station constraints
Experimenters’ ground stations are restricted to the DOME network layout:

- Guest Net provides internet access on the ground floor and assembly area, but it may not be redistributed wirelessly
- E-Net is tied to the experiment network and E-Link during flight, is only available in the ground station area, and must not be distributed
- during testing and flight, the experimenters’ ground station computer must not have any physical internet connection

## Grounding and EMC
The manual recommends a documented grounding concept to provide an equipotential reference plane, minimise common-mode issues, avoid ground loops, and protect against ESD-related hazards. Single-point, multi-point, hybrid, or total isolation approaches may be appropriate depending on the experiment. Distributed Single Point Grounding is recommended for complex payloads, and grounding to the gondola chassis can be provided if needed.

## RF and electronics validation
Before flight, teams must test reconnect behaviour for E-Link timeouts, monitor dropped packets, and inspect network traffic to ensure the payload does not exceed its bandwidth allocation. Wireshark or similar tools can be used for these checks.

The manual also requires frequency permission for every transmitter or receiver used at Esrange, and the team must provide the radio parameters in advance.
