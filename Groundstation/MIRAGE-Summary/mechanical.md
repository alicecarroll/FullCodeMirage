# MIRAGE Mechanical Design and Interfaces

## Overall mechanical concept
MIRAGE is built as a compact, multi-layer enclosure that holds a pressurised measurement chamber, pumps, valves, electronics, and thermal hardware. The outer envelope is constrained to 200 x 200 x 150 mm, and the total system mass is about 3 kg.

The design goal is to keep the measurement volume small, isolate the gas path, protect the components from landing loads and cold, and fit cleanly inside the ESCARGO gondola without modifying the vehicle structure.

## Enclosure and mounting
The enclosure has three layers:

- an outer protective shell
- a thermal insulation layer
- an inner COTS aluminum housing that protects the critical hardware and provides the sealed internal volume

The experiment mounts to the gondola using four bottom mounting points and standard M6 hardware on Bosch Rexroth aluminum T-slot profiles. Rubber spacers or dampers are used to reduce vibration coupling to the gondola.

## Pressurised chamber
The pressure chamber is a custom-machined aluminum vessel with a copper gasket and a ridge-and-groove sealing concept. It is designed for about 3 bar operation and is tested up to 4.5 bar. The chamber houses the K96 sensor plus internal pressure and temperature sensing, and it uses hermetic feedthroughs for electrical connections.

The SED reports finite-element analysis with a large safety margin, so the chamber itself is not the weak point of the design. The more difficult part is integrating the chamber with the flow system while keeping dead volume and leakage low.

## Pumping and flow system
The gas handling concept uses two diaphragm vacuum pumps and one piston compressor in a staged configuration. Ambient air is preheated, drawn through the first pump, further pressurised by the second pump, then compressed to 3 bar before entering the chamber.

Important mechanical elements of the flow path are:

- a normally closed outlet valve near the exhaust
- short, compact tubing to reduce dead volume
- polyurethane tubing with quick-connect fittings
- a micro-orifice outlet concept to control exhaust flow

The inlet air is meant to be fresh outside air, and the outlet and inlet should be oriented away from the gondola and other experiments to avoid contamination or interference.

## Mechanical constraints and interfaces
The design must survive vibration, transport, launch handling, shock at landing, and possible water exposure after recovery. The SED explicitly treats outgassing, water ingress, landing shock, and the possible need for sacrificial joints as mechanical concerns.

From an interface perspective, the key mechanical dependencies are the ESCARGO mounting frame, the power and data connector positions, and the need to place the whole assembly in a gondola corner where the air inlets and outlets remain unobstructed.

## Mechanical validation
The current verification plan includes static load testing, vibration testing, drop testing, pressure-vessel testing, and full-system fit checks. The mechanical design is still evolving, but the central question is already clear: can the compact pressurised gas system survive the flight environment while remaining leak-tight and serviceable?
