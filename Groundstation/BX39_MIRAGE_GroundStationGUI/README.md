# MIRAGE Ground Station GUI

The live console uses the local Python gateway; no package installation is required.

For live ground-station operation, run the local gateway:

```sh
python3 groundstation_gateway.py
```

Then open `http://127.0.0.1:8080`. The gateway serves the GUI, listens for the main MCU's TCP status stream on port `5001`, decodes the packed 216-byte `MainSystemStatusPacket`, streams telemetry to the browser, and sends GUI/terminal commands back over the active payload TCP connection. A command is acknowledged only after a newer payload status packet reports that a recognized command was received.

Each gateway run creates `logs/YYYY-MM-DD_HH-MM-SS.jsonl`. The file records every decoded telemetry frame, command lifecycle event, and payload connection event with a local ISO timestamp. The `logs/` directory is ignored by Git.

## Replacement Points

The current app uses mock layers in `src/app.js`:

- `MockTelemetrySource` emits 1 Hz telemetry frames and dropout events.
- `MockMirageSimulator` owns the synthetic MIRAGE state model.
- `MockCommandRouter` simulates uplink queueing, ACK/NACK delays, and command effects.
- `GatewayTelemetrySource` and `GatewayCommandRouter` connect the same UI to `groundstation_gateway.py` when served over HTTP.

When flight software and E-Link routing exist, replace those classes with real downlink/uplink adapters while keeping the UI rendering functions and command IDs stable.

## Hardware Command Contract

The gateway sends the `wireCommand` string from each frontend command definition. The current main MCU firmware already accepts heater commands such as `HEATER ON 1`, `HEATER OFF 1`, `HEATER ALL ON`, and `HEATER ALL OFF`. The GUI also defines the relay/peripheral command strings that the main/pressure MCU command parser should support:

- `RELAY 1 ON/OFF` through `RELAY 4 ON/OFF`
- `PUMP 1 ON/OFF` and `PUMP 2 ON/OFF`
- `COMPRESSOR ON/OFF`
- `VALVE OPEN/CLOSE`
- `PRESSURE ON/OFF`
- `FLUSH CHAMBER`
- `MODE STANDBY` and `MODE MEASUREMENTS`
- `REBOOT`
- `EMERGENCY STOP`

The status packet reports SD free percentage, main-controller state, pressure task state, individual pump/compressor PWM, relay/peripheral state, and the eight-bit heater mask. Gas plots use the K96 unfiltered raw IR detector channels rather than calculated ppm concentrations.

## Operator Scope

The console is arranged as the required four-quadrant tool:

- top left: timestamped operations log
- top right: live telemetry plots and health metrics
- bottom left: explicit mission command buttons
- bottom right: manual command terminal

## Fake Checks

Run the local checks with:

```sh
python3 -B -m unittest discover -s tests
node --check src/app.js  # optional if Node.js is installed
```
