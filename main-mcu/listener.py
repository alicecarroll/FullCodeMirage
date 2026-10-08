"""Console listener sharing the GUI gateway's packed telemetry contract."""
import argparse
import importlib.util
import json
from pathlib import Path
import socket
import threading


GATEWAY_PATH = Path(__file__).resolve().parents[1] / "Groundstation" / "BX39_MIRAGE_GroundStationGUI" / "groundstation_gateway.py"
spec = importlib.util.spec_from_file_location("groundstation_gateway", GATEWAY_PATH)
gateway = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gateway)


def command_sender(conn, stop_event):
    print("Type payload commands; quit/exit ends this connection.")
    while not stop_event.is_set():
        try:
            command = input().strip()
        except EOFError:
            stop_event.set()
            break
        if not command:
            continue
        if command.lower() in {"quit", "exit"}:
            stop_event.set()
            break
        try:
            conn.sendall(command.encode("utf-8"))
            print(f"Sent command: {command}")
        except OSError as exc:
            print(f"Failed to send command: {exc}")
            stop_event.set()
            break


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--legacy-status", action="store_true", help="receive older 230-byte status without current telemetry")
    args = parser.parse_args()
    packet_size = gateway.LEGACY_STATUS_PACKET_SIZE if args.legacy_status else gateway.STATUS_PACKET_SIZE
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.host, args.port))
        server.listen(5)
        print(f"Listening on {args.host}:{args.port}, expecting {packet_size}-byte telemetry...")
        while True:
            conn, addr = server.accept()
            conn.settimeout(5.0)
            print("Connected by", addr)
            stop_event = threading.Event()
            sender = threading.Thread(target=command_sender, args=(conn, stop_event), daemon=True)
            sender.start()
            try:
                seq = 0
                while not stop_event.is_set():
                    data = gateway.recv_exact(conn, packet_size)
                    seq += 1
                    frame = gateway.parse_status_packet(data, seq=seq)
                    current = frame["pressureCurrentA"]
                    print(f"Current: {current:.3f} A" if current is not None else "Current: unavailable")
                    if frame["pressureOvercurrentTripped"]:
                        print("OVERCURRENT LATCHED: all four pressure relays OFF; pressure MCU restart required")
                    print(json.dumps(frame, indent=2, allow_nan=False))
            except (ConnectionError, OSError, ValueError) as exc:
                print(f"Connection ended: {exc}")
            finally:
                stop_event.set()
                conn.close()


if __name__ == "__main__":
    main()
