import socket
import os

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(('0.0.0.0', 9999))
server_socket.listen(1)

print("Relay Active on 9999. Waiting for Simulink/MATLAB...")

while True:
    conn, addr = server_socket.accept()
    try:
        data = conn.recv(1024).decode().strip()
        if data:
            print(f"Received: '{data}'")
            # Split by semicolon so we can send "rl_drive_j 10;rr_drive_j 10"
            commands = data.split(';')
            for cmd_str in commands:
                if cmd_str:
                    joint, value = cmd_str.strip().split()
                    cmd = f'export GZ_PARTITION=default && gz topic -t "/model/dog_bot/joint/{joint}/cmd_vel" -m gz.msgs.Double -p "data: {value}"'
                    os.system(cmd)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        conn.close()
