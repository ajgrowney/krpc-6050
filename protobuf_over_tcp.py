"""Simple server/client with protobuf over tcp.

Pre-requisites: compile messages.proto into messages_pb2.
"""
import time
import socket
import struct

import messages_pb2  # ConnectionRequest, RealTelemetryPayload

connections = {}

def send_message(sock, message):
    payload = message.SerializeToString()
    header = struct.pack("!I", len(payload)) # Network - Unsigned Int
    sock.sendall(header + payload)

def recv_exact(sock, size):
    chunks = []
    remaining = size
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("socket closed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)

def recv_message(sock, message_cls):
    header = recv_exact(sock, 4)
    print("[recv_message] header", header)
    (size,) = struct.unpack("!I", header)
    payload = recv_exact(sock, size)
    print("[recv_message] payload: ", payload)
    message = message_cls()
    message.ParseFromString(payload)
    return message

def handle_client(conn, addr):
    req = recv_message(conn, messages_pb2.ConnectionRequest)
    cid = req.client_identifier.hex()
    connections[cid] = {
        "name": req.client_name,
        "type": req.type,
        "addr": addr
    }
    print(f"{req.client_name} connected from {addr}, type={req.type}")
    if req.type == messages_pb2.ConnectionRequest.STREAM:
        while True:
            tel = recv_message(conn, messages_pb2.RealTelemetryPayload)
            print(
                f"telemetry from {req.client_name}: "
                f"version={tel.packet_version} seq={tel.sequence} "
                f"status=0x{tel.status_mask:02X} yaw_cdeg={tel.yaw_cdeg} "
                f"pitch_cdeg={tel.pitch_cdeg} roll_cdeg={tel.roll_cdeg} "
                f"encoder_ticks={tel.encoder_ticks}"
            )
    else:
        print("Client connected")


def server(port):
    """Open a simple blocking ipv4 tcp listener on this port
    receive protocol buffer messages
    - Connection: 
    - Telemetry:
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        sock.bind(("0.0.0.0", port))
        sock.listen()

        print(f"listening on {port}")
        while True:
            conn, addr = sock.accept()
            try:
                handle_client(conn, addr)
            except Exception as exc:
                print(f"client error from {addr}: {exc}")
            finally:
                conn.close()
    return

def client(server_port):
    """Send a connection request
    - Flight Controller (Will send telemetry payloads)
    - Mission Control (Will render UI / alert)
    """
    with socket.create_connection(("127.0.0.1", server_port)) as sock:
        request = messages_pb2.ConnectionRequest()
        request.type = messages_pb2.ConnectionRequest.STREAM
        request.client_name = "Bird"
        request.client_identifier = b"bird-001"
        send_message(sock, request)

        for value in range(5):
            telemetry = messages_pb2.RealTelemetryPayload()
            telemetry.packet_version = 1
            telemetry.sequence = value
            telemetry.status_mask = 0x03
            telemetry.yaw_cdeg = value * 300
            telemetry.pitch_cdeg = value * 200
            telemetry.roll_cdeg = value * 100
            telemetry.encoder_ticks = value
            send_message(sock, telemetry)
            print(
                f"sent telemetry version={telemetry.packet_version} "
                f"seq={telemetry.sequence} status=0x{telemetry.status_mask:02X} "
                f"yaw_cdeg={telemetry.yaw_cdeg} pitch_cdeg={telemetry.pitch_cdeg} "
                f"roll_cdeg={telemetry.roll_cdeg} encoder_ticks={telemetry.encoder_ticks}"
            )
            time.sleep(1)
        print("client disconnecting")

if __name__ == "__main__":
    import sys
    mode = sys.argv[1]
    if mode not in ('client', 'server'):
        print("invalid mode")
        exit(1)
    
    if mode == "server":
        server(5001)
    else:
        client(5001)