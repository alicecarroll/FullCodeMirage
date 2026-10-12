#!/usr/bin/env python3
"""Measure MIRAGE TCP payload rates via dumpcap and the local GroundStation API.

By default this program only prints its plan. Passing --execute sends the
explicitly listed OFF/STANDBY commands. It does not issue any ON, relay, valve,
or pressurisation command. MODE STANDBY may close the outlet valve and disable
relays 2/3, so the operator must explicitly acknowledge that side effect.

Pressure is deliberately not decoded from telemetry: the observed wire framing
does not currently agree with the 216-byte decoder. The operator must check an
independent gauge and keep watching it while this test runs.
"""

from __future__ import annotations

import argparse
import json
import shutil
import signal
import socket
import struct
import subprocess
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterator


PRESSURE_ABORT_BAR = 2.90  # retains 0.10 bar margin to the user's 3.00 bar limit
SINGLE_COMMANDS = (
    "MODE STANDBY",
    "HEATER ALL OFF",
    "HEATER OFF 1",
    "PUMP 1 OFF",
    "PUMP 2 OFF",
    "COMPRESSOR OFF",
)
BURST_COMMAND = "MODE STANDBY"


@dataclass(frozen=True)
class PayloadPacket:
    timestamp: float
    direction: str  # "downlink" or "uplink"
    length: int


def iter_tcp_payloads(
    capture: Path, experiment_ip: str, laptop_ip: str, groundstation_port: int
) -> Iterator[PayloadPacket]:
    """Read IPv4/TCP payload lengths from a dumpcap-created pcapng file."""
    data = capture.read_bytes()
    pos = 0
    byte_order = "<"
    interface_types: list[int] = []
    timestamp_resolutions: list[float] = []

    while pos + 12 <= len(data):
        if data[pos : pos + 4] == b"\x0a\x0d\x0d\x0a":
            if pos + 12 > len(data):
                break
            bom = data[pos + 8 : pos + 12]
            if bom == b"\x4d\x3c\x2b\x1a":
                byte_order = "<"
            elif bom == b"\x1a\x2b\x3c\x4d":
                byte_order = ">"
            else:
                raise ValueError("Invalid pcapng byte-order marker")
            block_length = struct.unpack_from(byte_order + "I", data, pos + 4)[0]
        else:
            _, block_length = struct.unpack_from(byte_order + "II", data, pos)

        if block_length < 12 or pos + block_length > len(data):
            break  # tolerate a partially written final block

        block_type = struct.unpack_from(byte_order + "I", data, pos)[0]

        if block_type == 1 and block_length >= 20:  # Interface Description Block
            link_type = struct.unpack_from(byte_order + "H", data, pos + 8)[0]
            resolution = 1e-6  # pcapng default: microsecond timestamps
            option_pos = pos + 16
            option_end = pos + block_length - 4
            while option_pos + 4 <= option_end:
                code, option_length = struct.unpack_from(
                    byte_order + "HH", data, option_pos
                )
                option_pos += 4
                if code == 0:
                    break
                option = data[option_pos : option_pos + option_length]
                if code == 9 and option:  # if_tsresol
                    value = option[0]
                    resolution = (
                        2.0 ** -(value & 0x7F)
                        if value & 0x80
                        else 10.0 ** -value
                    )
                option_pos += (option_length + 3) & ~3
            interface_types.append(link_type)
            timestamp_resolutions.append(resolution)

        elif block_type == 6 and block_length >= 32:  # Enhanced Packet Block
            interface_id, ts_high, ts_low, captured_len, _ = struct.unpack_from(
                byte_order + "IIIII", data, pos + 8
            )
            if interface_id >= len(interface_types):
                raise ValueError("Capture references an unknown interface")
            if interface_types[interface_id] != 1:
                raise ValueError("Capture interface is not Ethernet (DLT_EN10MB)")

            raw_timestamp = (ts_high << 32) | ts_low
            timestamp = raw_timestamp * timestamp_resolutions[interface_id]
            frame = data[pos + 28 : pos + 28 + captured_len]
            packet = parse_ethernet_ipv4_tcp(
                frame, timestamp, experiment_ip, laptop_ip, groundstation_port
            )
            if packet is not None:
                yield packet

        pos += block_length


def parse_ethernet_ipv4_tcp(
    frame: bytes,
    timestamp: float,
    experiment_ip: str,
    laptop_ip: str,
    groundstation_port: int,
) -> PayloadPacket | None:
    if len(frame) < 14:
        return None
    network_offset = 14
    ether_type = struct.unpack_from("!H", frame, 12)[0]
    while ether_type in (0x8100, 0x88A8):  # optional VLAN tag
        if len(frame) < network_offset + 4:
            return None
        ether_type = struct.unpack_from("!H", frame, network_offset + 2)[0]
        network_offset += 4
    if ether_type != 0x0800 or len(frame) < network_offset + 20:
        return None  # this test's filter/addresses are IPv4

    ip_header_len = (frame[network_offset] & 0x0F) * 4
    ip_total_len = struct.unpack_from("!H", frame, network_offset + 2)[0]
    if (
        frame[network_offset + 9] != 6
        or ip_header_len < 20
        or ip_total_len < ip_header_len + 20
        or len(frame) < network_offset + ip_header_len + 20
    ):
        return None

    ip_end = min(len(frame), network_offset + ip_total_len)
    src_ip = socket.inet_ntoa(frame[network_offset + 12 : network_offset + 16])
    dst_ip = socket.inet_ntoa(frame[network_offset + 16 : network_offset + 20])
    tcp_offset = network_offset + ip_header_len
    src_port, dst_port = struct.unpack_from("!HH", frame, tcp_offset)
    tcp_header_len = (frame[tcp_offset + 12] >> 4) * 4
    payload_start = tcp_offset + tcp_header_len
    if tcp_header_len < 20 or payload_start > ip_end:
        return None

    if (
        src_ip == experiment_ip
        and dst_ip == laptop_ip
        and dst_port == groundstation_port
    ):
        direction = "downlink"
    elif (
        src_ip == laptop_ip
        and dst_ip == experiment_ip
        and src_port == groundstation_port
    ):
        direction = "uplink"
    else:
        return None

    payload_len = ip_end - payload_start  # excludes Ethernet padding and headers
    if payload_len <= 0:
        return None
    return PayloadPacket(timestamp, direction, payload_len)


def bytes_in_window(
    packets: list[PayloadPacket], direction: str, start: float, duration: float
) -> int:
    end = start + duration
    return sum(
        packet.length
        for packet in packets
        if packet.direction == direction and start <= packet.timestamp < end
    )


def max_rolling_one_second(packets: list[PayloadPacket], direction: str) -> int:
    times = [
        packet.timestamp
        for packet in packets
        if packet.direction == direction and packet.length > 0
    ]
    lengths = [
        packet.length
        for packet in packets
        if packet.direction == direction and packet.length > 0
    ]
    best = current = left = 0
    for right, timestamp in enumerate(times):
        current += lengths[right]
        while timestamp - times[left] >= 1.0:
            current -= lengths[left]
            left += 1
        best = max(best, current)
    return best


def post_command(api_url: str, command: str, timeout: float) -> tuple[int, dict]:
    body = json.dumps(
        {"wireCommand": command, "origin": "rate-test-python"}
    ).encode("utf-8")
    request = urllib.request.Request(
        api_url,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        response = urllib.request.urlopen(request, timeout=timeout)
    except urllib.error.HTTPError as error:
        response = error
    with response:
        try:
            result = json.loads(response.read().decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            result = {"message": "Gateway returned a non-JSON response"}
        return response.code, result


def get_gateway_status(status_url: str, timeout: float) -> dict:
    with urllib.request.urlopen(status_url, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def require_independent_pressure(stage: str) -> float:
    while True:
        raw = input(
            f"Independent gauge reading before {stage} (bar; stop at "
            f"{PRESSURE_ABORT_BAR:.2f}): "
        ).strip()
        try:
            value = float(raw)
        except ValueError:
            print("Enter the numeric pressure shown on the physical gauge.")
            continue
        if value < 0:
            print("Pressure cannot be negative; refusing to continue.")
            continue
        if value >= PRESSURE_ABORT_BAR:
            raise RuntimeError(
                f"Gauge reads {value:.3f} bar; {PRESSURE_ABORT_BAR:.2f}-bar "
                "guard reached, so no further commands will be sent."
            )
        return value


def start_capture(
    dumpcap: str, interface: str, capture_filter: str, output: Path
) -> subprocess.Popen:
    process = subprocess.Popen(
        [
            dumpcap,
            "-q",
            "-i",
            interface,
            "-f",
            capture_filter,
            "-w",
            str(output),
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    time.sleep(0.75)
    if process.poll() is not None:
        detail = process.stderr.read().strip() if process.stderr else ""
        raise RuntimeError(f"dumpcap failed to start: {detail or process.returncode}")
    return process


def stop_capture(process: subprocess.Popen) -> str:
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    try:
        _, stderr = process.communicate(timeout=10)
    except subprocess.TimeoutExpired:
        process.terminate()
        _, stderr = process.communicate(timeout=5)
    return stderr.strip()


def analyze_capture(
    capture: Path,
    experiment_ip: str,
    laptop_ip: str,
    groundstation_port: int,
    singles: list[tuple[str, float]],
    burst: tuple[float, float] | None,
    burst_count: int,
) -> None:
    packets = list(
        iter_tcp_payloads(capture, experiment_ip, laptop_ip, groundstation_port)
    )
    print(f"\nCapture: {capture}")
    print("Rates count TCP payload bytes only (not Ethernet/IP/TCP headers).")
    print("\nSingle-command windows (1.000 s from send start):")
    print(f"{'Command':<22} {'Uplink B/s':>12} {'Downlink B/s':>14}")
    for command, started in singles:
        up = bytes_in_window(packets, "uplink", started, 1.0)
        down = bytes_in_window(packets, "downlink", started, 1.0)
        print(f"{command:<22} {up:>12} {down:>14}")

    print(
        "\nMaximum rolling 1-second TCP-payload rate in entire capture: "
        f"downlink {max_rolling_one_second(packets, 'downlink')} B/s; "
        f"uplink {max_rolling_one_second(packets, 'uplink')} B/s"
    )
    if burst is not None and burst[1] > burst[0]:
        start, end = burst
        duration = end - start
        up = bytes_in_window(packets, "uplink", start, duration + 1e-6)
        down = bytes_in_window(packets, "downlink", start, duration + 1e-6)
        print(
            f"\nBurst: {burst_count} requested {BURST_COMMAND!r} commands, "
            f"{duration:.3f} s; uplink {up} B ({up / duration:.1f} B/s avg), "
            f"downlink {down} B ({down / duration:.1f} B/s avg)."
        )
        in_burst = [
            packet
            for packet in packets
            if start <= packet.timestamp <= end
        ]
        print(
            "Maximum rolling 1-second rate during burst: "
            f"uplink {max_rolling_one_second(in_burst, 'uplink')} B/s; "
            f"downlink {max_rolling_one_second(in_burst, 'downlink')} B/s"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="en12")
    parser.add_argument("--experiment-ip", default="192.168.0.2")
    parser.add_argument("--laptop-ip", default="192.168.0.3")
    parser.add_argument("--groundstation-port", type=int, default=5001)
    parser.add_argument("--gateway-http", default="http://127.0.0.1:8001")
    parser.add_argument("--burst-count", type=int, default=20)
    parser.add_argument("--ack-timeout", type=float, default=5.0)
    parser.add_argument("--capture", type=Path)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="send the listed OFF/STANDBY commands; without this, only print the plan",
    )
    parser.add_argument(
        "--analyze",
        type=Path,
        help="analyze an existing pcapng file only; sends no commands and starts no capture",
    )
    args = parser.parse_args()

    if args.burst_count < 1:
        parser.error("--burst-count must be at least 1")

    if args.analyze:
        analyze_capture(
            args.analyze,
            args.experiment_ip,
            args.laptop_ip,
            args.groundstation_port,
            [],
            None,
            args.burst_count,
        )
        return 0

    print("Commands in this test:")
    for command in SINGLE_COMMANDS:
        print(f"  {command}")
    print(f"  {args.burst_count} x {BURST_COMMAND}")
    print("No ON, relay, valve-open, or pressurisation command is included.")
    print(
        "WARNING: MODE STANDBY may close the outlet valve and turn relays 2/3 off."
    )
    print(
        "Pressure is NOT read from telemetry. Keep watching an independent gauge; "
        f"the script aborts before the next phase at {PRESSURE_ABORT_BAR:.2f} bar."
    )
    if not args.execute:
        print("Dry run only. Add --execute to capture and send these commands.")
        return 0

    if not sys.stdin.isatty():
        print("Refusing --execute without an interactive terminal for pressure checks.", file=sys.stderr)
        return 2

    ack = input(
        "Type exactly `STANDBY MAY CLOSE VALVE` to acknowledge that side effect: "
    ).strip()
    if ack != "STANDBY MAY CLOSE VALVE":
        print("Acknowledgement did not match; no commands sent.")
        return 2

    dumpcap = shutil.which("dumpcap")
    if not dumpcap:
        print("dumpcap was not found; install Wireshark/dumpcap first.", file=sys.stderr)
        return 2

    api_url = args.gateway_http.rstrip("/") + "/api/command"
    status_url = args.gateway_http.rstrip("/") + "/api/status"
    try:
        status = get_gateway_status(status_url, args.ack_timeout)
    except Exception as error:
        print(f"Cannot reach the local GroundStation gateway: {error}", file=sys.stderr)
        return 2
    if not status.get("payloadConnected"):
        print("Gateway has no active experiment TCP connection; no commands sent.", file=sys.stderr)
        return 2
    print(
        f"Gateway connected to {status.get('payloadAddress')}; its decoder reports "
        f"{status.get('packetSize')} bytes. Pressure telemetry is still not trusted."
    )
    print(
        "Gateway command acknowledgements may also be affected by the framing "
        "mismatch; do not treat them as independent hardware verification."
    )

    try:
        require_independent_pressure("the single-command tests")
    except (RuntimeError, KeyboardInterrupt) as error:
        print(f"Stopping before commands: {error}", file=sys.stderr)
        return 2

    capture = args.capture or Path.cwd() / (
        "mirage-rate-test-" + datetime.now().strftime("%Y%m%d-%H%M%S") + ".pcapng"
    )
    capture_filter = (
        f"tcp port {args.groundstation_port} and host {args.experiment_ip}"
    )
    print(f"Starting capture on {args.interface}: {capture_filter}")
    try:
        capture_process = start_capture(
            dumpcap, args.interface, capture_filter, capture
        )
    except Exception as error:
        print(f"Capture did not start; no commands sent: {error}", file=sys.stderr)
        return 2

    single_windows: list[tuple[str, float]] = []
    burst_window: tuple[float, float] | None = None
    burst_sent = 0
    singles_ok = True
    try:
        for command in SINGLE_COMMANDS:
            started = time.time()
            try:
                code, result = post_command(api_url, command, args.ack_timeout)
            except Exception as error:
                print(f"{command}: request error ({error}); stopping; do not retry automatically.")
                singles_ok = False
                break
            single_windows.append((command, started))
            print(f"{command}: HTTP {code}: {result.get('message', result)}")
            if code != 200 or not result.get("ok"):
                print("Gateway did not acknowledge; stopping further singles without retry.")
                singles_ok = False
                break
            time.sleep(max(0.0, started + 1.10 - time.time()))

        if not singles_ok:
            print("Skipping burst because the single-command phase did not complete.")
        else:
            try:
                require_independent_pressure("the repeated-standby burst")
            except (RuntimeError, KeyboardInterrupt) as error:
                print(f"Skipping burst: {error}")
                singles_ok = False

        if singles_ok:
            print("Keep the independent pressure gauge in view during the burst.")
            burst_start = time.time()
            for index in range(args.burst_count):
                try:
                    code, result = post_command(
                        api_url, BURST_COMMAND, args.ack_timeout
                    )
                except Exception as error:
                    print(f"Burst stopped at command {index + 1}: {error}; not retrying.")
                    break
                burst_sent += 1
                print(
                    f"Burst {index + 1}/{args.burst_count}: HTTP {code}: "
                    f"{result.get('message', result)}"
                )
                if code != 200 or not result.get("ok"):
                    print("Gateway did not acknowledge; stopping burst without retry.")
                    break
            burst_end = time.time()
            if burst_sent:
                burst_window = (burst_start, burst_end)
    except KeyboardInterrupt:
        print("\nInterrupted; stopping command sends and closing capture.")
    finally:
        summary = stop_capture(capture_process)
        if summary:
            print(summary)

    if capture.exists():
        analyze_capture(
            capture,
            args.experiment_ip,
            args.laptop_ip,
            args.groundstation_port,
            single_windows,
            burst_window,
            burst_sent,
        )
    else:
        print("No capture file was produced; no rate summary available.", file=sys.stderr)
        return 1

    try:
        require_independent_pressure("after the test")
    except (RuntimeError, KeyboardInterrupt) as error:
        print(f"Pressure check requires attention: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
