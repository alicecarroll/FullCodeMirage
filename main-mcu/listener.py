import socket
import struct

HOST = "0.0.0.0"
PORT = 5000

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(1)
print(f"Listening on {HOST}:{PORT}...")

while True:  # outer loop: always wait for a (new) connection
    print("Waiting for connection...")
    conn, addr = server.accept()
    print("Connected by", addr)

    try:
        while True:  # inner loop: handle this connection until it drops
            #data = conn.recv(1024)
            fmt = '<BBB' + 'f' * 22 + 'H'
            size = struct.calcsize(fmt)  # should be 85

            data = conn.recv(size)
            if not data:
                # Peer closed the connection gracefully (0 bytes = clean close)
                print("Connection closed by peer.")
                break

            print("Received:", data)

            
            if len(data) == size:
                values = struct.unpack(fmt, data)
                (seconds, minutes, hours,
                Tp1, Tp2, Tp3, Tp6, Pp3, Tp4, Pp1, Pa1, Ta1, Ta2, Ha1, Tp5, Pp2,
                Tt1, Tt2, Tt3,
                K96_CO2, K96_CH4, K96_H2O, K96_pressure, K96_temperature, K96_humidity,
                K96_error) = values

                print(f"Time: {hours:02}:{minutes:02}:{seconds:02}")
                print(f"Ambient: Pa1={Pa1:.6f}, Ha1={Ha1:.2f}, Ta1(Tmp117)= {Ta1:.2}, Ta2(SHT45)={Ta2:.2f} \n") 
                print(f"Temperatures of the pumps: Tp1={Tp1:.2f}, Tp2={Tp3:.2f}, Tp2={Tp3:.2f}\n")
                print(f"Temperatures and pressures between the pumps: \n Pipe2: Tp6={Tp6:.2f}, Pp3={Pp3:.6f}, Pipe3: Tp4={Tp4:.2f}, Pp1={Pp1:.6f}\n")
                print(f"Temperature and pressure in the chamber Tp5={Tp5:.2f}, Pp2={Pp2:.6f}\n")
                print(f"Temperatures of outlet and inlet: Tt1={Tt1:.2f}, Tt2={Tt2:.2f}\n")
                print(f"SD card temperature: Tt2={Tt2:.2f}\n")
                print(f"K96_CO2={K96_CO2:.2f}, K96_CH4={K96_CH4:.2f}, K96_H2O={K96_H2O:.2f}\n")

    except ConnectionResetError:
        # Peer closed the connection abruptly (RST packet) — e.g. ESP32 rebooted/crashed
        print("Connection was reset by peer.")

    except OSError as e:
        # Catch other socket-related errors so the whole program doesn't crash
        print(f"Socket error: {e}")

    finally:
        conn.close()
        # loop goes back to server.accept() and waits for the ESP32 to reconnect