"""Mission control proof-of-concept for the Pi-side bridge.

This module reads the Arduino's framed binary telemetry stream, validates each
packet, converts it into RealTelemetryPayload, and derives a simple normalized
control state that can later be mapped into kRPC vessel controls.

Runtime dependency: pyserial
"""

from __future__ import annotations

import argparse
import struct
import time
from dataclasses import dataclass

import messages_pb2


SYNC0 = 0xAA
SYNC1 = 0x55
PACKET_VERSION = 0x01
PACKET_SIZE = 17
PAYLOAD_OFFSET = 2
PAYLOAD_SIZE = 14
CRC_INDEX = 16
PAYLOAD_STRUCT = struct.Struct("<BHBhhhi")


class PacketError(ValueError):
    """Raised when a framed telemetry packet is malformed."""


@dataclass
class ControlState:
    roll: float
    pitch: float
    yaw: float
    throttle: float


@dataclass
class KSPControlAdapter:
    connection: object
    vessel: object

    def apply(self, controls: ControlState) -> None:
        self.vessel.control.roll = controls.roll
        self.vessel.control.pitch = controls.pitch
        self.vessel.control.yaw = controls.yaw
        self.vessel.control.throttle = controls.throttle

    def safe_reset(self, throttle: float = 0.0) -> None:
        self.vessel.control.roll = 0.0
        self.vessel.control.pitch = 0.0
        self.vessel.control.yaw = 0.0
        self.vessel.control.throttle = throttle

    def close(self) -> None:
        self.connection.close()


def clamp(value: float, lower: float, upper: float) -> float:
    # Simple double bounded func for value
    return max(lower, min(upper, value))


def crc8_maxim(data: bytes) -> int:
    crc = 0x00

    for input_byte in data:
        working_byte = input_byte
        for _ in range(8):
            mix = (crc ^ working_byte) & 0x01
            crc >>= 1
            if mix:
                crc ^= 0x8C
            working_byte >>= 1

    return crc


def decode_packet(packet: bytes) -> messages_pb2.RealTelemetryPayload:
    if len(packet) != PACKET_SIZE:
        raise PacketError(f"expected {PACKET_SIZE} bytes, got {len(packet)}")

    if packet[0] != SYNC0 or packet[1] != SYNC1:
        raise PacketError("sync word mismatch")

    payload = packet[PAYLOAD_OFFSET:CRC_INDEX]
    expected_crc = crc8_maxim(payload)
    packet_crc = packet[CRC_INDEX]
    if packet_crc != expected_crc:
        raise PacketError(
            f"crc mismatch: expected 0x{expected_crc:02X}, got 0x{packet_crc:02X}"
        )

    packet_version, sequence, status_mask, yaw_cdeg, pitch_cdeg, roll_cdeg, encoder_ticks = (
        PAYLOAD_STRUCT.unpack(payload)
    )

    if packet_version != PACKET_VERSION:
        raise PacketError(f"unsupported packet version {packet_version}")

    telemetry = messages_pb2.RealTelemetryPayload()
    telemetry.packet_version = packet_version
    telemetry.sequence = sequence
    telemetry.status_mask = status_mask
    telemetry.yaw_cdeg = yaw_cdeg
    telemetry.pitch_cdeg = pitch_cdeg
    telemetry.roll_cdeg = roll_cdeg
    telemetry.encoder_ticks = encoder_ticks
    return telemetry


def telemetry_to_controls(
    telemetry: messages_pb2.RealTelemetryPayload,
    angle_full_scale_deg: float,
    encoder_full_scale_ticks: int,
) -> ControlState:
    angle_full_scale_cdeg = angle_full_scale_deg * 100.0

    roll = clamp(telemetry.roll_cdeg / angle_full_scale_cdeg, -1.0, 1.0)
    pitch = clamp(telemetry.pitch_cdeg / angle_full_scale_cdeg, -1.0, 1.0)
    yaw = clamp(telemetry.yaw_cdeg / angle_full_scale_cdeg, -1.0, 1.0)
    throttle = clamp(telemetry.encoder_ticks / float(encoder_full_scale_ticks), 0.0, 1.0)

    return ControlState(roll=roll, pitch=pitch, yaw=yaw, throttle=throttle)


class FramedPacketParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def push(self, chunk: bytes) -> None:
        if chunk:
            self.buffer.extend(chunk)

    def pop_packets(self) -> list[bytes]:
        packets: list[bytes] = []

        while True:
            sync_index = self.buffer.find(bytes((SYNC0, SYNC1)))
            if sync_index == -1:
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                break

            if sync_index > 0:
                del self.buffer[:sync_index]

            if len(self.buffer) < PACKET_SIZE:
                break

            packets.append(bytes(self.buffer[:PACKET_SIZE]))
            del self.buffer[:PACKET_SIZE]

        return packets


def format_payload(
    telemetry: messages_pb2.RealTelemetryPayload,
    controls: ControlState,
) -> str:
    return (f"seq={telemetry.sequence:5d} "
f"status=0x{telemetry.status_mask:02X} "
f"roll={telemetry.roll_cdeg / 100.0:7.2f}deg "
f"pitch={telemetry.pitch_cdeg / 100.0:7.2f}deg "
f"yaw={telemetry.yaw_cdeg / 100.0:7.2f}deg "
f"encoder={telemetry.encoder_ticks:6d} "
f"controls=(roll={controls.roll:+.2f}, pitch={controls.pitch:+.2f}, "
f"yaw={controls.yaw:+.2f}, throttle={controls.throttle:.2f})")


def open_krpc_adapter(host: str, rpc_port: int, stream_port: int, client_name: str) -> KSPControlAdapter:
    try:
        import krpc
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "krpc is required for live KSP output. Install it with 'python3 -m pip install krpc'."
        ) from exc

    connection = krpc.connect(
        name=client_name,
        address=host,
        rpc_port=rpc_port,
        stream_port=stream_port,
    )
    vessel = connection.space_center.active_vessel
    return KSPControlAdapter(connection=connection, vessel=vessel)


def open_serial_port(
    port: str,
    baud_rate: int,
    startup_wait_seconds: float,
):
    try:
        import serial
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "pyserial is required. Install it with 'python3 -m pip install pyserial'."
        ) from exc

    serial_port = serial.Serial(port=port, baudrate=baud_rate, timeout=0.0)
    if startup_wait_seconds > 0.0:
        time.sleep(startup_wait_seconds)
        serial_port.reset_input_buffer()
    return serial_port


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Serial device, e.g. /dev/ttyACM0")
    parser.add_argument("--baud-rate", type=int, default=115200)
    parser.add_argument(
        "--timeout-seconds",
        type=float,
        default=0.25,
        help="How long a valid control sample remains fresh before output is reset",
    )
    parser.add_argument("--startup-wait-seconds", type=float, default=4.0,
        help="How long to wait after opening the serial port before reading packets",
    )
    parser.add_argument(
        "--enable-krpc",
        action="store_true",
        help="Write normalized controls to the active KSP vessel via kRPC",
    )
    parser.add_argument("--krpc-host", default="192.168.0.240")
    parser.add_argument("--krpc-rpc-port", type=int, default=50000)
    parser.add_argument("--krpc-stream-port", type=int, default=50001)
    parser.add_argument("--krpc-client-name", default="mission-control")
    parser.add_argument(
        "--control-tick-hz",
        type=float,
        default=20.0,
        help="Fixed host-side rate for pushing controls to kRPC",
    )
    parser.add_argument("--angle-full-scale-deg",type=float,default=45.0,
        help="Angle that maps to full-scale manual control input",
    )
    parser.add_argument("--encoder-full-scale-ticks",type=int,default=128,
        help="Encoder tick value that maps to full throttle",
    )
    parser.add_argument(
        "--max-packets",
        type=int,
        default=0,
        help="Stop after N packets; 0 means run until interrupted",
    )
    return parser


def main() -> int:
    args = build_argument_parser().parse_args()
    packet_count = 0
    last_timeout_report = 0.0
    last_controls = ControlState(roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0)
    last_valid_packet_time = 0.0
    adapter = None

    if args.control_tick_hz <= 0.0:
        raise ValueError("--control-tick-hz must be greater than 0")

    control_tick_period = 1.0 / args.control_tick_hz
    next_control_tick = time.monotonic()

    if args.enable_krpc:
        adapter = open_krpc_adapter(
            host=args.krpc_host,
            rpc_port=args.krpc_rpc_port,
            stream_port=args.krpc_stream_port,
            client_name=args.krpc_client_name,
        )

    try:
        with open_serial_port(
            args.port,
            args.baud_rate,
            args.startup_wait_seconds,
        ) as serial_port:
            parser = FramedPacketParser()

            while args.max_packets == 0 or packet_count < args.max_packets:
                bytes_available = max(1, getattr(serial_port, "in_waiting", 0))
                chunk = serial_port.read(bytes_available)
                parser.push(chunk)

                saw_valid_packet = False
                for packet in parser.pop_packets():
                    try:
                        telemetry = decode_packet(packet)
                    except PacketError as exc:
                        print(f"discarded packet: {exc}")
                        continue

                    controls = telemetry_to_controls(
                        telemetry,
                        angle_full_scale_deg=args.angle_full_scale_deg,
                        encoder_full_scale_ticks=args.encoder_full_scale_ticks,
                    )
                    last_controls = controls
                    last_valid_packet_time = time.monotonic()
                    saw_valid_packet = True
                    if adapter is None:
                        print(format_payload(telemetry, controls))
                    packet_count += 1
                    if args.max_packets != 0 and packet_count >= args.max_packets:
                        break

                now = time.monotonic()
                if adapter is not None and now >= next_control_tick:
                    is_fresh = (now - last_valid_packet_time) <= args.timeout_seconds
                    if is_fresh:
                        adapter.apply(last_controls)
                    else:
                        adapter.safe_reset(throttle=last_controls.throttle)
                        if now - last_timeout_report >= 2.0:
                            print("waiting for telemetry: latest control sample is stale")
                            last_timeout_report = now

                    while next_control_tick <= now:
                        next_control_tick += control_tick_period

                if not saw_valid_packet and not chunk:
                    time.sleep(min(control_tick_period, 0.005))
    finally:
        if adapter is not None:
            adapter.safe_reset(throttle=last_controls.throttle)
            adapter.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())