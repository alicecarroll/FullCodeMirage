import importlib.util
import json
import struct
import tempfile
import unittest
from pathlib import Path


GUI_ROOT = Path(__file__).resolve().parents[1]
GATEWAY_PATH = GUI_ROOT / "groundstation_gateway.py"

spec = importlib.util.spec_from_file_location("groundstation_gateway", GATEWAY_PATH)
gateway = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gateway)


def make_status_packet(
    *,
    mode=3,
    connection_lost=0,
    status_ok=1,
    pressure_system_on=1,
    heater_mask=0x0D,
    thermal_online=1,
    thermal_error=0,
    pressure_state=2,
    command_received=0,
    onboard_logging=1,
    storage_free_pct=73,
    controller_state=1,
    captured_errors=0,
):
    floats = [
        31.0,   # Tp1
        32.0,   # Tp2
        33.0,   # Tp3
        24.0,   # Tp6
        0.8,    # Pp3
        25.0,   # Tp4
        1.1,    # Pp1
        0.9,    # Pa1, bar
        -12.0,  # Ta1
        -11.5,  # Ta2
        -12.2,  # Ta3
        41.0,   # Ha1
        21.5,   # Tp5
        3.0,    # Pp2
        18.0,   # Tt1
        27.0,   # Tt2
        19.0,   # Tt3
        416,    # K96_CO2
        1.86,   # K96_CH4
        3100,   # K96_H2O
        3000.0, # K96 pressure hPa
        21,     # K96 temp
        39.0,   # K96 humidity
        25.0, 25.0, 20.0, 20.0, 21.0, 21.0, 39.0, 21.0,  # K96 temperature/humidity
        1100, 1101, 3100.0, 3100.0, 0,  # MPL
        2200, 2201, 1.86, 0, 0,          # LPL
        3300, 3301, 416.0, 0, 0,         # SPL
        0,              # K96_error
    ]
    return struct.pack(
        gateway.SENSOR_STRUCT_FORMAT,
        1,
        2,
        3,
        *floats,
        mode,
        command_received,
        connection_lost,
        status_ok,
        pressure_system_on,
        heater_mask,
        thermal_online,
        2,
        thermal_error,
        pressure_state,
        0,              # pressure error
        0x0B,           # relay mask
        80,             # pump 1 PWM
        0,              # pump 2 PWM
        60,             # compressor PWM
        1,              # manual relay override
        1,              # valve open
        onboard_logging,
        storage_free_pct,
        controller_state,
        int(captured_errors).to_bytes(16, byteorder="little"),
    )


class StatusPacketParserTest(unittest.TestCase):
    def test_error_manifest_is_shared_and_contiguous(self):
        self.assertEqual(len(gateway.ERROR_MESSAGES), 75)
        self.assertEqual(gateway.ERROR_MESSAGES[0], "Ethernet SPI read transaction failed")
        self.assertEqual(gateway.ERROR_MESSAGES[74], "RTC Read failure")
        self.assertTrue(gateway.ERROR_MANIFEST_PATH.exists())

    def test_decodes_main_system_status_packet(self):
        frame = gateway.parse_status_packet(make_status_packet(), seq=42, timestamp_ms=123456)

        self.assertTrue(frame["valid"])
        self.assertEqual(frame["seq"], 42)
        self.assertEqual(frame["timestamp"], 123456)
        self.assertEqual(frame["mode"], "MEASUREMENTS")
        self.assertEqual(frame["health"], "healthy")
        self.assertEqual(frame["linkStatus"], "ONLINE")
        self.assertEqual(frame["methaneRaw"], 2200)
        self.assertEqual(frame["co2Raw"], 3300)
        self.assertEqual(frame["waterRaw"], 1100)
        self.assertAlmostEqual(frame["chamberPressureBar"], 3.0, places=2)
        self.assertAlmostEqual(frame["ambientPressureHpa"], 900.0, places=1)
        self.assertTrue(frame["pressureSystemOn"])
        self.assertTrue(frame["peripherals"]["pump1"])
        self.assertEqual(frame["pump1DutyPct"], 80)
        self.assertTrue(frame["relayLines"]["relay1"])
        self.assertTrue(frame["relayLines"]["relay4"])
        self.assertTrue(frame["peripherals"]["outletValve"])
        self.assertTrue(frame["pressureValveOpen"])
        self.assertEqual(frame["heaterMask"], 0x0D)
        self.assertTrue(frame["thermalOnline"])
        self.assertTrue(frame["onboardLogging"])
        self.assertEqual(frame["storageFreePct"], 73)
        self.assertTrue(frame["controllerReady"])
        self.assertEqual(frame["controller"], "MAIN_MCU_READY")
        self.assertEqual(frame["activeTask"], "PRESSURISATION")
        self.assertEqual(frame["errors"], [])

    def test_flush_state_is_reported_as_active_task(self):
        frame = gateway.parse_status_packet(make_status_packet(pressure_state=4))
        self.assertEqual(frame["pressureStateName"], "FLUSH_CHAMBER")
        self.assertEqual(frame["activeTask"], "FLUSH_CHAMBER")
        self.assertEqual(frame["heater1ActuationPct"], 100)
        self.assertEqual(frame["heater2ActuationPct"], 0)

    def test_decodes_captured_error_bits(self):
        frame = gateway.parse_status_packet(
            make_status_packet(captured_errors=(1 << 0) | (1 << 55)),
        )

        self.assertEqual(frame["health"], "fault")
        self.assertEqual([error["bit"] for error in frame["errors"]], [0, 55])
        self.assertIn("Ethernet SPI read", frame["errors"][0]["message"])

    def test_decodes_high_captured_error_bits(self):
        frame = gateway.parse_status_packet(make_status_packet(captured_errors=1 << 74))
        self.assertEqual(frame["errors"], [{"bit": 74, "message": "RTC Read failure"}])

    def test_connection_lost_packet_becomes_dropout(self):
        frame = gateway.parse_status_packet(make_status_packet(connection_lost=1), seq=7)

        self.assertFalse(frame["valid"])
        self.assertEqual(frame["linkStatus"], "DROPOUT")
        self.assertEqual(frame["linkQuality"], 0)
        self.assertIn("connection_lost", frame["dropoutReason"])

    def test_bad_packet_size_is_rejected(self):
        with self.assertRaises(ValueError):
            gateway.parse_status_packet(b"short")


class FrontendCommandContractTest(unittest.TestCase):
    def test_frontend_contains_hardware_command_strings(self):
        app_js = (GUI_ROOT / "src" / "app.js").read_text(encoding="utf-8")

        expected_wire_commands = [
            "RELAY 1 ON",
            "RELAY 4 OFF",
            "PUMP 1 ON",
            "PUMP 2 OFF",
            "COMPRESSOR ON",
            "VALVE OPEN",
            "HEATER ALL ON",
            "MODE MEASUREMENTS",
            "PRESSURE ON",
            "PRESSURE OFF",
            "FLUSH CHAMBER",
            "REBOOT",
            "EMERGENCY STOP",
        ]

        for command in expected_wire_commands:
            with self.subTest(command=command):
                self.assertIn(f'wireCommand: "{command}"', app_js)

        expected_toggle_ids = [
            'data-toggle="relay1"',
            'data-toggle="relay4"',
            'data-toggle="pump1"',
            'data-toggle="pump2"',
            'data-toggle="compressor"',
            'data-toggle="outletValve"',
        ]
        index_html = (GUI_ROOT / "index.html").read_text(encoding="utf-8")
        for toggle_id in expected_toggle_ids:
            with self.subTest(toggle_id=toggle_id):
                self.assertIn(toggle_id, index_html)

        self.assertNotIn("pingExperiment", app_js)
        self.assertNotIn("ping experiment", index_html.lower())
        self.assertIn('data-task="FLUSH_CHAMBER"', index_html)
        self.assertIn('id="heaterActuationChart"', index_html)
        self.assertIn('data-heater-checkbox="1"', index_html)
        self.assertIn('data-heater-checkbox="8"', index_html)
        self.assertIn('data-heater-action="on"', index_html)
        self.assertIn('data-heater-action="off"', index_html)
        self.assertIn('wireCommand: "HEATER " + action.toUpperCase()', app_js)
        self.assertIn('heaterBit: channel.bit', app_js)


class CommandAcknowledgementTest(unittest.TestCase):
    def test_disconnected_payload_cannot_acknowledge(self):
        state = gateway.GroundStationState()
        ok, message = state.send_command("PRESSURE ON", ack_timeout=0.01)
        self.assertFalse(ok)
        self.assertIn("no payload", message)

    def test_ack_requires_new_command_received_telemetry(self):
        state = gateway.GroundStationState()

        class FakePayload:
            def __init__(self):
                self.sent = b""

            def sendall(self, payload):
                self.sent += payload
                state.next_frame(make_status_packet(command_received=1))

        payload = FakePayload()
        state.attach_payload(payload, ("127.0.0.1", 1234))
        ok, message = state.send_command("PRESSURE ON", ack_timeout=0.1)

        self.assertTrue(ok)
        self.assertEqual(payload.sent, b"PRESSURE ON")
        self.assertIn("telemetry frame 1", message)

    def test_disconnect_after_send_is_not_an_ack(self):
        state = gateway.GroundStationState()

        class DisconnectingPayload:
            def sendall(self, _payload):
                state.detach_payload(self)

        payload = DisconnectingPayload()
        state.attach_payload(payload, ("127.0.0.1", 1234))
        ok, message = state.send_command("PRESSURE ON", ack_timeout=0.1)

        self.assertFalse(ok)
        self.assertIn("disconnected before acknowledging", message)


class SessionLogTest(unittest.TestCase):
    def test_session_log_records_timestamped_json_lines(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            session_log = gateway.SessionLog(Path(temp_dir) / "logs")
            session_log.record("command", {"wireCommand": "PRESSURE OFF"})

            entries = [json.loads(line) for line in session_log.path.read_text().splitlines()]
            self.assertRegex(session_log.path.name, r"^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}\.jsonl$")
            self.assertEqual(entries[-1]["event"], "command")
            self.assertIn("capturedAt", entries[-1])
            self.assertEqual(entries[-1]["data"]["wireCommand"], "PRESSURE OFF")


if __name__ == "__main__":
    unittest.main()
