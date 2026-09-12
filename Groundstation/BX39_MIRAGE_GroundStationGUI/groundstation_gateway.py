#!/usr/bin/env python3
"""Local MIRAGE ground-station gateway.

The flight code sends a packed MainSystemStatusPacket over a TCP connection to
the ground laptop. Browsers cannot receive that raw TCP stream directly, so this
gateway bridges the existing payload protocol to the static GUI:

- TCP :5001 receives payload status packets and sends text commands back.
- HTTP :8080 serves the GUI.
- GET /api/telemetry streams decoded status frames as server-sent events.
- POST /api/command sends a command string to the connected payload socket.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import re
import socketserver
import struct
import threading
import time
from datetime import datetime
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any


SENSOR_STRUCT_FORMAT = (
    "<3B"        # seconds, minutes, hours
    "17f"       # Tp1 through Tt3
    "ididid"    # K96 signals & filtered doubles (int32, double, int32, double, int32, double)
    "8f"        # K96 temperatures & humidity floats
    "HHffH"     # MPL block: uflt_ir, flt_ir, uflt_conc, flt_conc, uflt_error
    "HHfHH"     # LPL block: uflt_ir, flt_ir, uflt_conc, uflt_error, flt_error
    "HHfHH"     # SPL block: uflt_ir, flt_ir, uflt_conc, uflt_error, flt_error
    "H"         # K96_error (uint16_t)
    "5BH14B16s" # Flags, subsystem status, SD/controller status, 128-bit captured_errors
)

STATUS_PACKET_SIZE = struct.calcsize(SENSOR_STRUCT_FORMAT)
EXPECTED_STATUS_PACKET_SIZE = 216
if STATUS_PACKET_SIZE != EXPECTED_STATUS_PACKET_SIZE:
    raise RuntimeError(
        f"groundstation packet layout is {STATUS_PACKET_SIZE} bytes; "
        f"main MCU transmits {EXPECTED_STATUS_PACKET_SIZE} bytes"
    )

MODE_NAMES = {
    1: "TEST_LOOP",
    2: "STANDBY",
    3: "MEASUREMENTS",
    4: "HUMIDITY",
}

CONTROLLER_STATES = {
    0: "MAIN_MCU_BOOTING",
    1: "MAIN_MCU_READY",
    2: "MAIN_MCU_SAFE_SHUTDOWN",
    3: "MAIN_MCU_RESTARTING",
}

PRESSURE_STATES = {
    0: "STANDBY",
    1: "PREPRESSURISATION",
    2: "AIR_EXCHANGE",
    3: "ERROR",
    4: "FLUSH_CHAMBER",
}

ERROR_MANIFEST_PATH = Path(__file__).resolve().parents[2] / "main-mcu" / "main" / "ErrorBits.def"


def load_error_messages() -> list[str]:
    messages: dict[int, str] = {}
    for line in ERROR_MANIFEST_PATH.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("//"):
            continue
        match = re.fullmatch(r'ERROR_BIT\((\d+),\s*(".*")\)', line)
        if not match:
            raise ValueError(f"invalid error manifest line: {line}")
        bit = int(match.group(1))
        message = json.loads(match.group(2))
        if bit in messages:
            raise ValueError(f"duplicate error bit in manifest: {bit}")
        messages[bit] = message

    expected_bits = list(range(len(messages)))
    if sorted(messages) != expected_bits:
        raise ValueError("error manifest bits must be contiguous from zero")
    return [messages[bit] for bit in expected_bits]


ERROR_MESSAGES = load_error_messages()


def decode_captured_errors(captured_errors: int) -> list[dict[str, Any]]:
    errors = []
    known_mask = 0
    for bit, message in enumerate(ERROR_MESSAGES):
        bit_mask = 1 << bit
        known_mask |= bit_mask
        if captured_errors & bit_mask:
            errors.append({"bit": bit, "message": message})

    unknown = captured_errors & ~known_mask
    bit = len(ERROR_MESSAGES)
    while unknown:
        if unknown & 1:
            errors.append({"bit": bit, "message": f"Unknown captured error (bit {bit})"})
        unknown >>= 1
        bit += 1
    return errors


def recv_exact(sock, size: int) -> bytes:
    chunks: list[bytes] = []
    remaining = size

    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("MCU closed TCP connection")
        chunks.append(chunk)
        remaining -= len(chunk)

    return b"".join(chunks)


def finite_number(value: Any, fallback: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return fallback

    if math.isnan(number) or math.isinf(number):
        return fallback
    return number


def pressure_to_hpa(value: float) -> float:
    pressure = finite_number(value, 1013.0)
    if pressure < 20:
        return pressure * 1000.0
    return pressure


def evaluate_health(frame: dict[str, Any]) -> str:
    if frame.get("connectionLost") or not frame.get("statusOk", True):
        return "fault"
    if frame.get("capturedErrors", 0):
        return "fault"
    if frame.get("thermalError", 0):
        return "warning"
    if frame.get("controller") == "MAIN_MCU_SAFE_SHUTDOWN":
        return "fault"
    if frame.get("controller") in {"MAIN_MCU_BOOTING", "MAIN_MCU_RESTARTING"}:
        return "warning"

    chamber_pressure = finite_number(frame.get("chamberPressureBar"), 3.0)
    chamber_temp = finite_number(
        frame.get("chamberTempC_K96", frame.get("chamberTempC_MS")),
        21.0,
    )
    humidity = finite_number(frame.get("humidityRh_ambient"), 38.0)

    if (
        chamber_pressure > 3.55
        or chamber_pressure < 2.35
        or chamber_temp > 50
        or chamber_temp < 5
        or humidity > 84
    ):
        return "fault"

    if (
        chamber_pressure > 3.22
        or chamber_pressure < 2.74
        or chamber_temp > 40
        or chamber_temp < 15
        or humidity > 65
    ):
        return "warning"

    return "healthy"


def parse_status_packet(data: bytes, seq: int = 0, timestamp_ms: int | None = None) -> dict[str, Any]:
    if len(data) != STATUS_PACKET_SIZE:
        raise ValueError(f"expected {STATUS_PACKET_SIZE} bytes, got {len(data)}")

    values = struct.unpack(SENSOR_STRUCT_FORMAT, data)
    (
        seconds,
        minutes,
        hours,
        tp1,
        tp2,
        tp3,
        tp6,
        pp3,
        tp4,
        pp1,
        pa1,
        ta1,
        ta2,
        ta3,
        ha1,
        tp5,
        pp2,
        tt1,
        tt2,
        tt3,
        k96_lpl_signal,
        k96_lpl_signal_filtered,
        k96_spl_signal,
        k96_spl_signal_filtered,
        k96_mpl_signal,
        k96_mpl_signal_filtered,
        k96_aducdie_temp,
        k96_aducdie_temp_filtered,
        k96_ntc0_temp,
        k96_ntc0_temp_filtered,
        k96_ntc1_temp,
        k96_ntc1_temp_filtered,
        k96_rh,
        k96_rh_temp,
        k96_mpl_uflt_ir_signal,
        k96_mpl_flt_ir_signal,
        k96_mpl_uflt_conc,
        k96_mpl_flt_conc,
        k96_mpl_uflt_error,
        k96_lpl_uflt_ir_signal,
        k96_lpl_flt_ir_signal,
        k96_lpl_uflt_conc,
        k96_lpl_uflt_error,
        k96_lpl_flt_error,
        k96_spl_uflt_ir_signal,
        k96_spl_flt_ir_signal,
        k96_spl_uflt_conc,
        k96_spl_uflt_error,
        k96_spl_flt_error,
        k96_error,
        operating_mode,
        command_received,
        connection_lost,
        status_ok,
        pressure_system_on,
        heater_mask,
        thermal_online,
        thermal_state,
        thermal_error,
        pressure_state,
        pressure_error,
        pressure_relay_mask,
        pressure_pump1_pwm,
        pressure_pump2_pwm,
        pressure_compressor_pwm,
        pressure_manual_override,
        pressure_valve_open,
        onboard_logging,
        storage_free_pct,
        controller_state,
        captured_errors_bytes,
    ) = values

    captured_errors = int.from_bytes(captured_errors_bytes, byteorder="little")
    timestamp = timestamp_ms if timestamp_ms is not None else int(time.time() * 1000)
    link_status = "DROPOUT" if connection_lost else "ONLINE"
    link_quality = 0 if connection_lost else 100

    frame = {
        "valid": not bool(connection_lost),
        "timestamp": timestamp,
        "seq": seq,
        "mode": MODE_NAMES.get(operating_mode, f"MODE_{operating_mode}"),
        "linkStatus": link_status,
        "linkQuality": link_quality,
        "latencyMs": 0,
        "methaneRaw": int(k96_lpl_uflt_ir_signal),
        "co2Raw": int(k96_spl_uflt_ir_signal),
        "waterRaw": int(k96_mpl_uflt_ir_signal),
        "chamberPressureBar": finite_number(pp2),
        "chamberTempC_MS": finite_number(tp5),
        "chamberTempC_K96": finite_number(k96_rh_temp, 0.0),
        "electronicsTempC": finite_number(tt2, 25.0),
        "humidityRh_ambient": finite_number(ha1),
        "humidityRh_k96": finite_number(k96_rh),
        "ambientPressureBar": finite_number(pa1),
        "ambientPressureHpa": finite_number(pa1) * 1000.0,
        "Interstage_1Bar": finite_number(pp3) + finite_number(pa1),
        "Interstage_2Bar": finite_number(pp1) + finite_number(pa1),
        "pump1C": finite_number(tp1),
        "pump2C": finite_number(tp2),
        "compressorC": finite_number(tp3),
        "Interstage1_C": finite_number(tp6),
        "Interstage2_C": finite_number(tp4),
        "outletC": finite_number(tt1),
        "sdCardC": finite_number(tt2),
        "inletC": finite_number(tt3),
        "ambientTempC_TMP": finite_number(ta1),
        "ambientTempC_MS": finite_number(ta3),
        "ambientTempC_SHT": finite_number(ta2),
        "pump1DutyPct": int(pressure_pump1_pwm),
        "pump2DutyPct": int(pressure_pump2_pwm),
        "compressorDutyPct": int(pressure_compressor_pwm),
        "heaterDutyPct": 100 if heater_mask else 0,
        "coolerDutyPct": 0,
        "outletValveOpen": bool(pressure_valve_open),
        "pressureSystemOn": bool(pressure_system_on),
        "heaterMask": int(heater_mask),
        "peripherals": {
            "pump1": bool(pressure_pump1_pwm),
            "pump2": bool(pressure_pump2_pwm),
            "compressor": bool(pressure_compressor_pwm),
            "outletValve": bool(pressure_valve_open),
        },
        "relayLines": {
            "relay1": bool(pressure_relay_mask & 0x01),
            "relay2": bool(pressure_relay_mask & 0x02),
            "relay3": bool(pressure_relay_mask & 0x04),
            "relay4": bool(pressure_relay_mask & 0x08),
        },
        "heater1ActuationPct": 100 if heater_mask & (1 << 0) else 0,
        "heater2ActuationPct": 100 if heater_mask & (1 << 1) else 0,
        "heater3ActuationPct": 100 if heater_mask & (1 << 2) else 0,
        "heater4ActuationPct": 100 if heater_mask & (1 << 3) else 0,
        "heater5ActuationPct": 100 if heater_mask & (1 << 4) else 0,
        "heater6ActuationPct": 100 if heater_mask & (1 << 5) else 0,
        "heater7ActuationPct": 100 if heater_mask & (1 << 6) else 0,
        "heater8ActuationPct": 100 if heater_mask & (1 << 7) else 0,
        "onboardLogging": bool(onboard_logging),
        "storageFreePct": int(storage_free_pct),
        "controller": CONTROLLER_STATES.get(controller_state, f"MAIN_MCU_STATE_{controller_state}"),
        "controllerReady": controller_state == 1,
        "thermalOnline": bool(thermal_online),
        "thermalState": int(thermal_state),
        "thermalError": int(thermal_error),
        "pressureState": int(pressure_state),
        "pressureStateName": PRESSURE_STATES.get(pressure_state, f"STATE_{pressure_state}"),
        "pressureError": int(pressure_error),
        "pressureRelayMask": int(pressure_relay_mask),
        "pressurePump1Pwm": int(pressure_pump1_pwm),
        "pressurePump2Pwm": int(pressure_pump2_pwm),
        "pressureCompressorPwm": int(pressure_compressor_pwm),
        "pressureValveOpen": bool(pressure_valve_open),
        "pressureManualOverride": bool(pressure_manual_override),
        "capturedErrors": int(captured_errors),
        "errors": decode_captured_errors(int(captured_errors)),
        "commandReceived": bool(command_received),
        "connectionLost": bool(connection_lost),
        "statusOk": bool(status_ok),
        "payloadClock": f"{hours:02}:{minutes:02}:{seconds:02}",
        "rawPressures": {
            "k96Hpa": finite_number(k96_ntc0_temp),
        },
        "k96Error": int(k96_error),
    }

    frame["missionMode"] = frame["mode"]
    if pressure_state == 4:
        frame["activeTask"] = "FLUSH_CHAMBER"
    elif pressure_state in {1, 2}:
        frame["activeTask"] = "PRESSURISATION"
    else:
        frame["activeTask"] = None
    frame["health"] = evaluate_health(frame)
    if frame["connectionLost"]:
        frame["dropoutReason"] = "payload reported connection_lost in status packet"
        frame["statusText"] = frame["dropoutReason"]
    else:
        frame["statusText"] = "payload status packet decoded from TCP stream"

    return frame


class SessionLog:
    def __init__(self, logs_dir: Path) -> None:
        logs_dir.mkdir(parents=True, exist_ok=True)
        started_at = datetime.now().astimezone()
        self.path = logs_dir / f"{started_at.strftime('%Y-%m-%d_%H-%M-%S')}.jsonl"
        self._lock = threading.Lock()
        self.record("session", {"phase": "started"})

    def record(self, event: str, data: dict[str, Any]) -> None:
        entry = {
            "capturedAt": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "event": event,
            "data": data,
        }
        encoded = json.dumps(entry, allow_nan=False, separators=(",", ":"))
        with self._lock:
            with self.path.open("a", encoding="utf-8") as log_file:
                log_file.write(encoded + "\n")


class GroundStationState:
    def __init__(self, session_log: SessionLog | None = None) -> None:
        self._lock = threading.RLock()
        self._frame_condition = threading.Condition(self._lock)
        self._command_lock = threading.Lock()
        self._clients: list[queue.Queue[dict[str, Any]]] = []
        self._payload_conn = None
        self._payload_addr = None
        self._seq = 0
        self._last_frame: dict[str, Any] | None = None
        self._session_log = session_log

    def log(self, event: str, data: dict[str, Any]) -> None:
        if self._session_log:
            self._session_log.record(event, data)

    def attach_payload(self, conn, addr) -> None:
        with self._frame_condition:
            self._payload_conn = conn
            self._payload_addr = addr
            self._frame_condition.notify_all()
        self.log("connection", {"phase": "payload_connected", "address": f"{addr[0]}:{addr[1]}"})
        self.broadcast_gateway_status()

    def detach_payload(self, conn) -> None:
        detached = False
        with self._frame_condition:
            if self._payload_conn is conn:
                self._payload_conn = None
                self._payload_addr = None
                detached = True
                self._frame_condition.notify_all()
        if detached:
            self.log("connection", {"phase": "payload_disconnected"})
        self.broadcast_gateway_status()

    def add_client(self) -> queue.Queue[dict[str, Any]]:
        client_queue: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=100)
        with self._lock:
            self._clients.append(client_queue)
            last_frame = self._last_frame

        if last_frame:
            client_queue.put({"event": "telemetry", "data": last_frame})
        client_queue.put({"event": "gateway", "data": self.gateway_status()})
        return client_queue

    def remove_client(self, client_queue: queue.Queue[dict[str, Any]]) -> None:
        with self._lock:
            if client_queue in self._clients:
                self._clients.remove(client_queue)

    def next_frame(self, packet: bytes) -> dict[str, Any]:
        with self._frame_condition:
            self._seq += 1
            seq = self._seq

        frame = parse_status_packet(packet, seq=seq)
        with self._frame_condition:
            self._last_frame = frame
            self._frame_condition.notify_all()
        self.log("telemetry", frame)
        self.broadcast("telemetry", frame)
        return frame

    def send_command(
        self,
        command: str,
        metadata: dict[str, Any] | None = None,
        ack_timeout: float = 3.5,
    ) -> tuple[bool, str]:
        payload = command.strip()
        if not payload:
            return False, "empty command"

        command_log = dict(metadata or {})
        command_log["wireCommand"] = payload

        with self._command_lock:
            with self._frame_condition:
                conn = self._payload_conn
                starting_seq = self._seq

            if conn is None:
                message = "no payload TCP connection is active"
                self.log("command", {**command_log, "phase": "rejected", "message": message})
                return False, message

            self.log("command", {**command_log, "phase": "sent", "afterSeq": starting_seq})
            try:
                conn.sendall(payload.encode("utf-8"))
            except OSError as exc:
                message = f"payload command send failed: {exc}"
                self.log("command", {**command_log, "phase": "failed", "message": message})
                return False, message

            deadline = time.monotonic() + ack_timeout
            with self._frame_condition:
                while True:
                    frame = self._last_frame
                    if frame and frame["seq"] > starting_seq and frame.get("commandReceived"):
                        message = f"payload acknowledged '{payload}' in telemetry frame {frame['seq']}"
                        self.log(
                            "command",
                            {**command_log, "phase": "acknowledged", "ackSeq": frame["seq"], "message": message},
                        )
                        return True, message

                    if self._payload_conn is not conn:
                        message = "payload disconnected before acknowledging command"
                        self.log("command", {**command_log, "phase": "failed", "message": message})
                        return False, message

                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        message = "payload did not acknowledge command before timeout"
                        self.log("command", {**command_log, "phase": "timeout", "message": message})
                        return False, message
                    self._frame_condition.wait(timeout=remaining)

    def gateway_status(self) -> dict[str, Any]:
        with self._lock:
            payload_connected = self._payload_conn is not None
            payload_addr = self._payload_addr
            client_count = len(self._clients)
            last_frame = self._last_frame

        return {
            "payloadConnected": payload_connected,
            "payloadAddress": f"{payload_addr[0]}:{payload_addr[1]}" if payload_addr else None,
            "browserClients": client_count,
            "lastSeq": last_frame["seq"] if last_frame else None,
            "packetSize": STATUS_PACKET_SIZE,
            "logFile": str(self._session_log.path) if self._session_log else None,
        }

    def broadcast_gateway_status(self) -> None:
        self.broadcast("gateway", self.gateway_status())

    def broadcast(self, event: str, data: dict[str, Any]) -> None:
        with self._lock:
            clients = list(self._clients)

        message = {"event": event, "data": data}
        for client_queue in clients:
            try:
                client_queue.put_nowait(message)
            except queue.Full:
                pass


class PayloadTCPHandler(socketserver.BaseRequestHandler):
    state: GroundStationState

    def setup(self) -> None:
        self.state.attach_payload(self.request, self.client_address)

    def handle(self) -> None:
        while True:
            packet = recv_exact(self.request, STATUS_PACKET_SIZE)
            if not packet:
                break
            self.state.next_frame(packet)

    def finish(self) -> None:
        self.state.detach_payload(self.request)


class ReusableThreadingTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def make_http_handler(static_root: Path, state: GroundStationState):
    class GroundStationHTTPHandler(SimpleHTTPRequestHandler):
        server_version = "MIRAGEGroundStation/1.0"

        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(static_root), **kwargs)

        def end_headers(self) -> None:
            # Prevent browser caching for all served static assets
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            super().end_headers()

        def do_GET(self) -> None:
            if self.path == "/api/status":
                self.send_json(HTTPStatus.OK, state.gateway_status())
                return

            if self.path == "/api/telemetry":
                self.stream_telemetry()
                return

            if self.path == "/":
                self.path = "/index.html"

            super().do_GET()

        def do_POST(self) -> None:
            if self.path != "/api/command":
                self.send_error(HTTPStatus.NOT_FOUND)
                return

            length = int(self.headers.get("Content-Length", "0"))
            body = self.rfile.read(length)

            try:
                payload = json.loads(body.decode("utf-8"))
            except json.JSONDecodeError:
                self.send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": "invalid JSON body"})
                return

            command = str(payload.get("wireCommand", "")).strip()
            metadata = {
                key: payload[key]
                for key in ("requestId", "commandId", "label", "origin")
                if key in payload
            }
            ok, message = state.send_command(command, metadata=metadata)
            status = HTTPStatus.OK if ok else HTTPStatus.SERVICE_UNAVAILABLE
            self.send_json(status, {"ok": ok, "message": message})

        def send_json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
            encoded = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(encoded)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(encoded)

        def stream_telemetry(self) -> None:
            client_queue = state.add_client()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()

            try:
                while True:
                    try:
                        message = client_queue.get(timeout=15)
                    except queue.Empty:
                        self.wfile.write(b": keepalive\n\n")
                        self.wfile.flush()
                        continue

                    event_name = message["event"]
                    event_payload = json.dumps(message["data"], allow_nan=False)
                    self.wfile.write(f"event: {event_name}\n".encode("utf-8"))
                    self.wfile.write(f"data: {event_payload}\n\n".encode("utf-8"))
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                state.remove_client(client_queue)

        def log_message(self, fmt: str, *args) -> None:
            print(f"[http] {self.address_string()} - {fmt % args}")

    return GroundStationHTTPHandler


def run_servers(host: str, http_port: int, payload_port: int, static_root: Path) -> None:
    session_log = SessionLog(static_root / "logs")
    state = GroundStationState(session_log)

    PayloadTCPHandler.state = state
    payload_server = ReusableThreadingTCPServer((host, payload_port), PayloadTCPHandler)

    http_handler = make_http_handler(static_root, state)
    http_server = ReusableThreadingHTTPServer((host, http_port), http_handler)

    payload_thread = threading.Thread(target=payload_server.serve_forever, name="payload-tcp", daemon=True)
    payload_thread.start()

    print(f"MIRAGE GUI: http://127.0.0.1:{http_port}")
    print(f"Payload TCP listener: {host}:{payload_port} expecting {STATUS_PACKET_SIZE} byte status packets")
    print(f"Session log: {session_log.path}")

    try:
        http_server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down MIRAGE ground-station gateway")
    finally:
        session_log.record("session", {"phase": "stopped"})
        http_server.shutdown()
        payload_server.shutdown()
        http_server.server_close()
        payload_server.server_close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the MIRAGE local ground-station gateway")
    parser.add_argument("--host", default="0.0.0.0", help="host/IP for HTTP and payload TCP listeners")
    parser.add_argument("--http-port", default=int(os.environ.get("MIRAGE_HTTP_PORT", "8080")), type=int)
    parser.add_argument("--payload-port", default=int(os.environ.get("MIRAGE_PAYLOAD_PORT", "5001")), type=int)
    parser.add_argument("--static-root", default=Path(__file__).resolve().parent, type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    run_servers(args.host, args.http_port, args.payload_port, args.static_root.resolve())
