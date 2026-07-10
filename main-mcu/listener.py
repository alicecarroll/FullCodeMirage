import socket

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
            data = conn.recv(1024)
            if not data:
                # Peer closed the connection gracefully (0 bytes = clean close)
                print("Connection closed by peer.")
                break

            print("Received:", data)
            # process data here...

    except ConnectionResetError:
        # Peer closed the connection abruptly (RST packet) — e.g. ESP32 rebooted/crashed
        print("Connection was reset by peer.")

    except OSError as e:
        # Catch other socket-related errors so the whole program doesn't crash
        print(f"Socket error: {e}")

    finally:
        conn.close()
        # loop goes back to server.accept() and waits for the ESP32 to reconnect