# MIRAGE Ground Station GUI Agent

Build the MIRAGE mission-control ground station GUI for BX39.

## Mission
Design a ground station interface that receives E-Link telemetry, visualizes live system health, and provides command controls for the experiment. No flight software or live data stream exists yet, so implement the GUI with mock telemetry, mock command handlers, and clean adapter layers that can later be replaced by real E-Link uplink/downlink code.

## Design Goal
The UI must help an operator quickly answer one question: is MIRAGE healthy right now?

Use the MIRAGE summaries as the system reference. The experiment is a pressurised methane sensing payload with thermal control, pressure control, onboard logging, and E-Link telemetry. The GUI should reflect those realities and make failures obvious.

## Required Layout
Use a fixed four-quadrant layout:

- Top left: logs and updates
- Top right: live plot area
- Bottom left: command panel
- Bottom right: built-in terminal for urgent commands

## Quadrant Requirements

### Top Left: Logs and Updates
Show a timestamped activity feed of the latest events.

- include telemetry receipt events
- include command acknowledgements and failures
- include mode changes, warnings, and state transitions
- show clear timestamps on every entry
- make the newest entry visually prominent

This panel should behave like an operations log, not a generic chat window.

### Top Right: Live Plot Area
Show live telemetry updated every second using mock data.

Plot the most important signals for MIRAGE, such as:

- methane, carbon dioxide, and water measurements
- chamber pressure and target pressure band
- chamber temperature and electronics temperature
- chamber humidity
- ambient pressure and ambient temperature
- pump or compressor activity
- link quality or telemetry status

Prefer a clear operator view over a decorative one. If space is limited, use tabs or stacked charts, but keep the telemetry readable at a glance.

### Bottom Left: Command Panel
Provide button-based mission commands that an operator can use quickly.

Use mock commands for now. The buttons should later be wired to the real uplink function, but for the current implementation they only need to trigger local state updates, log entries, and simulated acknowledgements.

Include commands that match a realistic MIRAGE ground-station workflow, such as:

- start experiment
- enter standby
- start or stop pressurisation
- open or close outlet valve
- enable or disable heating
- enable or disable cooling
- flush chamber
- request status update
- restart main controller
- emergency stop or safe shutdown
- ping experiment

Use confirmation dialogs for disruptive commands. Mark safety-critical commands clearly.

### Bottom Right: Built-in Terminal
Provide a terminal-style command entry area for urgent or non-button commands.

- accept typed commands
- echo the command locally
- show simulated output and acknowledgements
- support future replacement with real uplink routing
- keep the terminal visually separate from the button panel so operators know it is for manual intervention

## Mock Data Requirements
Because no flight software exists yet, build the entire interface against mock data.

- generate synthetic telemetry at 1 Hz
- keep mock values realistic for MIRAGE flight conditions
- simulate healthy, warning, and fault states
- simulate telemetry dropouts and recovery
- simulate command acknowledgements, delays, and failures

The mock layer should be isolated behind a small adapter so the future real data source can be swapped in without redesigning the UI.

## MIRAGE-Specific Context To Reflect
The GUI should feel like a BEXUS mission-control tool for a pressurised gas experiment, not a generic dashboard.

Use the MIRAGE operating context:

- E-Link is the downlink/uplink path for the experiment
- telemetry is expected roughly once per second for the operator view
- the payload has a pressurised measurement chamber
- thermal control is important and must be visible
- humidity is operationally critical because condensation can affect the sensor and chamber
- local storage exists, so the UI should acknowledge both live telemetry and onboard logging

## Visual and Interaction Guidance

- make the interface look like an intentional operations tool
- keep the most important status indicators highly visible
- use strong color coding for nominal, warning, and fault states
- make connection loss obvious
- avoid hiding critical state behind menus
- keep the layout responsive enough to work on the launch-site laptop

## Suggested Application Structure
If you build the application, separate it into these logical layers:

- UI layout and rendering
- telemetry simulation and mock data generation
- command simulation and command routing
- shared MIRAGE state model

The code should be ready for later integration with real E-Link and flight software, but should not depend on them now.

## Acceptance Criteria
The result should be good enough that an operator can:

- see the latest status at a glance
- follow live telemetry trends
- issue common mission commands with one click
- type urgent manual commands in the terminal
- understand whether the system is healthy, warning, or faulted

Do not invent a real backend. Use mock telemetry and mock commands only, with clear seams for replacement later.
