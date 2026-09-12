# MIRAGE Thermal Analysis and Thermal System Design

## Why thermal control is central
MIRAGE depends on temperature control for both science and survival. The K96 sensor is sensitive to temperature drift, the pressurised gas can freeze or condense water, and the electronics need to stay within safe limits while the balloon moves from warm ground handling to a very cold stratosphere.

## Thermal environment
The SED uses a wide flight envelope:

- ground preparation around room temperature
- launch pad exposure at about 0 to -15 C
- stratospheric flight down to about -80 C outside the gondola
- possible post-flight exposure to snow, ice, and cold air for one or two days

The chamber itself is targeted to stay near 19 to 24 C during active operation, with a broader acceptable active range of 15 to 40 C. Electronics in the enclosure have their own allowable range and need dedicated protection.

## Thermal design approach
The current design combines passive insulation and active thermal control. The enclosure is surrounded by a thermal layer, with the inner housing protected by insulation and the gas path thermally managed so that the pumps and chamber do not freeze or overheat.

The active thermal system includes:

- an intake air preheater before the first vacuum pump
- a chamber heater for the pressurised measurement volume
- a Peltier cooler between the chamber and the enclosure wall
- an outlet heater to prevent freezing at the exhaust
- an SD-card heater to protect storage

## Thermal control logic
The thermal MCU runs PID control loops for the thermal subsystems. The design uses sensor feedback from the chamber and electronics to decide when to heat, cool, flush, or hold. Humidity is treated as a thermal problem as well, because too much water vapor can cause condensation or ice formation.

The software also supports a protective humidity loop: if humidity is too high, shutters stay closed and pump actions are restricted until the system is safe again.

## Analysis results and design implications
The current thermal analysis suggests that insulation is necessary, but the exact insulation material and thickness are still being refined. The pumps generate useful heat during ascent, while high-altitude sunlight and low convection can cause overheating if the pumps remain active too long.

The key takeaway is that MIRAGE does not have a single fixed thermal solution. It needs a flight-phase-dependent thermal strategy that combines insulation, controlled pump operation, heaters, and possibly cooling to keep both the chamber and the electronics in range.
