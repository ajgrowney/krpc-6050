"""Simple server/client with protobuf over tcp 
to show the kRPC design
Pre-requisites: compile the messages.proto into messages_pb2

"""
import time
import socket
import messages_pb2 # ConnectionRequest, TelemetryPayload
import struct

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
            tel = recv_message(conn, messages_pb2.TelemetryPayload)
            print(
                f"telemetry from {req.client_name}: "
                f"status={tel.status} roll={tel.roll} pitch={tel.pitch} "
                f"yaw={tel.yaw} encoder={tel.encoder_count}"
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
            telemetry = messages_pb2.TelemetryPayload()
            telemetry.status = 3
            telemetry.roll = value * 1.0
            telemetry.pitch = value * 2.0
            telemetry.yaw = value * 3.0
            telemetry.encoder_count = value
            send_message(sock, telemetry)
            print(
                f"sent telemetry status={telemetry.status} roll={telemetry.roll} "
                f"pitch={telemetry.pitch} yaw={telemetry.yaw} "
                f"encoder={telemetry.encoder_count}"
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