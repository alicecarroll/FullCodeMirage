import socket
import struct
import threading


HOST = "0.0.0.0"
PORT = 5001


def recv_exact(conn, size):
    chunks = []
    remaining = size

    while remaining > 0:
        chunk = conn.recv(remaining)
        if not chunk:
            return b""
        chunks.append(chunk)
        remaining -= len(chunk)

    return b"".join(chunks)


def describe_heaters(mask):
    names = [f"H{index + 1}" for index in range(8) if mask & (1 << index)]
    return ", ".join(names) if names else "none"


MODE_NAMES = {
    1: "Test Loop",
    2: "Standby",
    3: "Measurement",
    4: "Humidity",
}


def command_sender(conn, stop_event):
    print("Type heater commands like: HEATER ON 1, HEATER OFF 1, HEATER ALL ON, HEATER ALL OFF")
    while not stop_event.is_set():
        try:
            command = input()
        except EOFError:
            stop_event.set()
            break

        command = command.strip()
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

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(1)
print(f"Listening on {HOST}:{PORT}...")

while True:  # outer loop: always wait for a (new) connection
    print("Waiting for connection...")
    conn, addr = server.accept()
    print("Connected by", addr)
    stop_event = threading.Event()
    sender_thread = threading.Thread(target=command_sender, args=(conn, stop_event), daemon=True)
    sender_thread.start()

    try:
        while not stop_event.is_set():  # inner loop: handle this connection until it drops
            fmt = '<BBB' + 'f' * 23 + 'H' + '9B'
            size = struct.calcsize(fmt)

            data = recv_exact(conn, size)

            if not data:
                # Peer closed the connection gracefully (0 bytes = clean close)
                print("Connection closed by peer.")
                break

            if len(data) == size:
                values = struct.unpack(fmt, data)
                (seconds, minutes, hours,
                Tp1, Tp2, Tp3, Tp6, Pp3, Tp4, Pp1, Pa1, Ta1, Ta2,Ta3, Ha1, Tp5, Pp2,
                Tt1, Tt2, Tt3,
                K96_CO2, K96_CH4, K96_H2O, K96_pressure, K96_temperature, K96_humidity,
                K96_error,
                operating_mode, command_received, connection_lost, status_ok,
                pressure_system_on, heater_mask, thermal_online, thermal_state,
                thermal_error) = values

                print(f"Mode: {operating_mode} ({MODE_NAMES.get(operating_mode, 'Unknown')})")
                print(f"Ethernet command received: {'yes' if command_received else 'no'}")
                print(f"Connection lost: {'yes' if connection_lost else 'no'} | Status OK: {'yes' if status_ok else 'no'}")
                print(f"Pressure system active: {'yes' if pressure_system_on else 'no'} | Active heaters: {describe_heaters(heater_mask)}")
                print(f"Thermal MCU: {'online' if thermal_online else 'offline'} | state={thermal_state} | error={thermal_error}")

                print(f"Time: {hours:02}:{minutes:02}:{seconds:02}")
                print(f"Ambient: Ta1(Tmp117)= {Ta1:.2}, Pa1(MS5803)={Pa1:.6f}, Ta2(MS5803)={Ta2:.6f}, Ha1={Ha1:.2f}, Ta3(SHT45)={Ta3:.2f}") 
                print(f"Temperatures of the pumps: Tp1={Tp1:.2f}, Tp2={Tp2:.2f}, Tp3={Tp3:.2f}")
                print(f"Temperatures and pressures between the pumps: \n Pipe2: Tp6={Tp6:.2f}, Pp3={Pp3:.6f}, Pipe3: Tp4={Tp4:.2f}, Pp1={Pp1:.6f}")
                print(f"Temperature and pressure in the chamber Tp5={Tp5:.2f}, Pp2={Pp2:.6f}")
                print(f"Temperatures of outlet and inlet: Tt1={Tt1:.2f}, Tt3={Tt3:.2f}")
                print(f"SD card temperature: Tt2={Tt2:.2f}")
                print(f"K96_CO2={K96_CO2:.2f}, K96_CH4={K96_CH4:.2f}, K96_H2O={K96_H2O:.2f}")

    except ConnectionResetError:
        # Peer closed the connection abruptly (RST packet) — e.g. ESP32 rebooted/crashed
        print("Connection was reset by peer.")

    except OSError as e:
        # Catch other socket-related errors so the whole program doesn't crash
        print(f"Socket error: {e}")

    finally:
        stop_event.set()
        conn.close()
        # loop goes back to server.accept() and waits for the ESP32 to reconnect