"""Mission control proof-of-concept for the Pi-side bridge.

This module reads the Arduino's framed binary telemetry stream, validates each
packet, converts it into RealTelemetryPayload, and derives a simple normalized
control state that can later be mapped into kRPC vessel controls.

"""

import argparse
import struct
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from threading import Event, Lock, Thread
from typing import Optional

try:
    import serial
except ModuleNotFoundError as exc:
    raise RuntimeError(
        "pyserial is required. Install it with 'python3 -m pip install pyserial'."
    ) from exc

import messages_pb2


SYNC0 = 0xAA
SYNC1 = 0x55
PACKET_VERSION = 0x01
PACKET_SIZE = 17
PAYLOAD_OFFSET = 2
PAYLOAD_SIZE = 14
CRC_INDEX = 16
PAYLOAD_STRUCT = struct.Struct("<BHBhhhi")
VALIDATION_PACKET_COUNT = 20
VALIDATION_TIMEOUT_SECONDS = 10.0
CALIBRATION_TIMEOUT_SECONDS = 10.0
CALIBRATION_SAMPLE_COUNT = 20
NEUTRAL_CAPTURE_MAX_RETRIES = 5
NEUTRAL_MAX_SPREAD_CDEG = 250
MOTION_MIN_DELTA_CDEG = 500
MOTION_DOMINANCE_RATIO = 1.5


class PacketError(ValueError):
    """Raised when a framed telemetry packet is malformed."""


@dataclass
class ControlState:
    roll: float
    pitch: float
    yaw: float
    throttle: float


@dataclass(frozen=True)
class PhysicalAxes:
    yaw_cdeg: int
    pitch_cdeg: int
    roll_cdeg: int


class AxisName(str, Enum):
    YAW = "yaw"
    PITCH = "pitch"
    ROLL = "roll"


@dataclass(frozen=True)
class AxisMapping:
    source_axis: AxisName
    sign: int = 1


@dataclass(frozen=True)
class CalibrationProfile:
    neutral_yaw_cdeg: int = 0
    neutral_pitch_cdeg: int = 0
    neutral_roll_cdeg: int = 0
    yaw_mapping: AxisMapping = AxisMapping(source_axis=AxisName.YAW)
    pitch_mapping: AxisMapping = AxisMapping(source_axis=AxisName.PITCH)
    roll_mapping: AxisMapping = AxisMapping(source_axis=AxisName.ROLL)


@dataclass(frozen=True)
class TelemetrySample:
    telemetry: messages_pb2.RealTelemetryPayload
    received_at: float


@dataclass(frozen=True)
class TelemetrySnapshot:
    latest_sample: Optional[TelemetrySample] = None
    latest_accepted_at: Optional[float] = None
    consecutive_valid_packets: int = 0
    discarded_packets: int = 0
    last_error: Optional[str] = None
    reader_faulted: bool = False


class TelemetryStore:
    def __init__(self, max_history: int = 256) -> None:
        self._lock = Lock()
        self._history: deque[TelemetrySample] = deque(maxlen=max_history)
        self._latest_sample: Optional[TelemetrySample] = None
        self._latest_accepted_at: Optional[float] = None
        self._consecutive_valid_packets = 0
        self._discarded_packets = 0
        self._last_error: Optional[str] = None
        self._reader_faulted = False

    def record_valid_sample(self, sample: TelemetrySample) -> None:
        with self._lock:
            self._latest_sample = sample
            self._latest_accepted_at = sample.received_at
            self._history.append(sample)
            self._consecutive_valid_packets += 1
            self._last_error = None

    def record_invalid_packet(self, error_text: str, received_at: float) -> None:
        del received_at
        with self._lock:
            self._consecutive_valid_packets = 0
            self._discarded_packets += 1
            self._last_error = error_text

    def record_reader_fault(self, error_text: str, received_at: float) -> None:
        del received_at
        with self._lock:
            self._reader_faulted = True
            self._last_error = error_text

    def get_snapshot(self) -> TelemetrySnapshot:
        with self._lock:
            return TelemetrySnapshot(
                latest_sample=self._latest_sample,
                latest_accepted_at=self._latest_accepted_at,
                consecutive_valid_packets=self._consecutive_valid_packets,
                discarded_packets=self._discarded_packets,
                last_error=self._last_error,
                reader_faulted=self._reader_faulted,
            )

    def get_recent_samples(
        self,
        max_age_seconds: Optional[float] = None,
        max_count: Optional[int] = None,
    ) -> list[TelemetrySample]:
        with self._lock:
            samples = list(self._history)

        if max_age_seconds is not None and samples:
            newest_received_at = samples[-1].received_at
            oldest_allowed = newest_received_at - max_age_seconds
            samples = [sample for sample in samples if sample.received_at >= oldest_allowed]

        if max_count is not None:
            samples = samples[-max_count:]

        return samples

    def get_samples_since(
        self,
        received_after: Optional[float],
    ) -> list[TelemetrySample]:
        with self._lock:
            samples = list(self._history)

        if received_after is None:
            return samples

        return [sample for sample in samples if sample.received_at > received_after]


class SerialReader:
    def __init__(
        self,
        serial_port: object,
        telemetry_store: TelemetryStore,
        idle_sleep_seconds: float = 0.005,
    ) -> None:
        self._serial_port = serial_port
        self._telemetry_store = telemetry_store
        self._idle_sleep_seconds = idle_sleep_seconds
        self._parser = FramedPacketParser()
        self._stop_event = Event()
        self._thread = Thread(target=self._run, name="serial-reader", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()

    def join(self, timeout: Optional[float] = None) -> None:
        self._thread.join(timeout=timeout)

    def _run(self) -> None:
        try:
            while not self._stop_event.is_set():
                bytes_available = max(1, getattr(self._serial_port, "in_waiting", 0))
                chunk = self._serial_port.read(bytes_available)
                self._parser.push(chunk)

                for packet in self._parser.pop_packets():
                    try:
                        telemetry = decode_packet(packet)
                    except PacketError as exc:
                        self._telemetry_store.record_invalid_packet(
                            str(exc),
                            time.monotonic(),
                        )
                        print(f"discarded packet: {exc}")
                        continue

                    self._telemetry_store.record_valid_sample(
                        TelemetrySample(
                            telemetry=telemetry,
                            received_at=time.monotonic(),
                        )
                    )

                if not chunk:
                    time.sleep(self._idle_sleep_seconds)
        except Exception as exc:
            self._telemetry_store.record_reader_fault(
                str(exc),
                time.monotonic(),
            )


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


def extract_physical_axes(
    telemetry: messages_pb2.RealTelemetryPayload,
) -> PhysicalAxes:
    return PhysicalAxes(
        yaw_cdeg=telemetry.yaw_cdeg,
        pitch_cdeg=telemetry.pitch_cdeg,
        roll_cdeg=telemetry.roll_cdeg,
    )


def get_axis_value(axes: PhysicalAxes, axis_name: AxisName) -> int:
    if axis_name is AxisName.YAW:
        return axes.yaw_cdeg
    if axis_name is AxisName.PITCH:
        return axes.pitch_cdeg
    return axes.roll_cdeg


def calibration_profile_to_neutral_axes(
    calibration_profile: CalibrationProfile,
) -> PhysicalAxes:
    return PhysicalAxes(
        yaw_cdeg=calibration_profile.neutral_yaw_cdeg,
        pitch_cdeg=calibration_profile.neutral_pitch_cdeg,
        roll_cdeg=calibration_profile.neutral_roll_cdeg,
    )


def average_physical_axes(samples: list[TelemetrySample]) -> PhysicalAxes:
    if not samples:
        raise ValueError("expected at least one telemetry sample")

    yaw_total = 0
    pitch_total = 0
    roll_total = 0
    for sample in samples:
        axes = extract_physical_axes(sample.telemetry)
        yaw_total += axes.yaw_cdeg
        pitch_total += axes.pitch_cdeg
        roll_total += axes.roll_cdeg

    sample_count = len(samples)
    return PhysicalAxes(
        yaw_cdeg=round(yaw_total / sample_count),
        pitch_cdeg=round(pitch_total / sample_count),
        roll_cdeg=round(roll_total / sample_count),
    )


def physical_axes_spread(samples: list[TelemetrySample]) -> PhysicalAxes:
    if not samples:
        raise ValueError("expected at least one telemetry sample")

    yaw_values = []
    pitch_values = []
    roll_values = []
    for sample in samples:
        axes = extract_physical_axes(sample.telemetry)
        yaw_values.append(axes.yaw_cdeg)
        pitch_values.append(axes.pitch_cdeg)
        roll_values.append(axes.roll_cdeg)

    return PhysicalAxes(
        yaw_cdeg=max(yaw_values) - min(yaw_values),
        pitch_cdeg=max(pitch_values) - min(pitch_values),
        roll_cdeg=max(roll_values) - min(roll_values),
    )


def map_axis_delta(
    physical_axes: PhysicalAxes,
    neutral_axes: PhysicalAxes,
    axis_mapping: AxisMapping,
) -> int:
    measured = get_axis_value(physical_axes, axis_mapping.source_axis)
    neutral = get_axis_value(neutral_axes, axis_mapping.source_axis)
    return axis_mapping.sign * (measured - neutral)


def telemetry_to_controls(
    telemetry: messages_pb2.RealTelemetryPayload,
    angle_full_scale_deg: float,
    encoder_full_scale_ticks: int,
    calibration_profile: Optional[CalibrationProfile] = None,
) -> ControlState:
    if calibration_profile is None:
        calibration_profile = CalibrationProfile()

    angle_full_scale_cdeg = angle_full_scale_deg * 100.0
    physical_axes = extract_physical_axes(telemetry)
    neutral_axes = calibration_profile_to_neutral_axes(calibration_profile)

    roll = clamp(
        map_axis_delta(physical_axes, neutral_axes, calibration_profile.roll_mapping)
        / angle_full_scale_cdeg,
        -1.0,
        1.0,
    )
    pitch = clamp(
        map_axis_delta(physical_axes, neutral_axes, calibration_profile.pitch_mapping)
        / angle_full_scale_cdeg,
        -1.0,
        1.0,
    )
    yaw = clamp(
        map_axis_delta(physical_axes, neutral_axes, calibration_profile.yaw_mapping)
        / angle_full_scale_cdeg,
        -1.0,
        1.0,
    )
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
    

    serial_port = serial.Serial(port=port, baudrate=baud_rate, timeout=0.0)
    if startup_wait_seconds > 0.0:
        time.sleep(startup_wait_seconds)
        serial_port.reset_input_buffer()
    return serial_port


def wait_for_valid_telemetry(
    telemetry_store: TelemetryStore,
    min_packets: int = VALIDATION_PACKET_COUNT,
    timeout_seconds: float = VALIDATION_TIMEOUT_SECONDS,
) -> TelemetrySnapshot:
    deadline = time.monotonic() + timeout_seconds

    while True:
        snapshot = telemetry_store.get_snapshot()
        if snapshot.reader_faulted:
            raise RuntimeError(snapshot.last_error or "serial reader fault")

        if snapshot.consecutive_valid_packets >= min_packets:
            return snapshot

        if time.monotonic() >= deadline:
            raise RuntimeError(
                f"timed out waiting for valid telemetry after {timeout_seconds:.1f} seconds"
            )

        time.sleep(0.01)


def collect_calibration_samples(
    telemetry_store: TelemetryStore,
    sample_count: int,
    timeout_seconds: float,
    discarded_packets_at_start: int,
    received_after: float,
) -> list[TelemetrySample]:
    deadline = time.monotonic() + timeout_seconds

    while True:
        snapshot = telemetry_store.get_snapshot()
        if snapshot.reader_faulted:
            raise RuntimeError(snapshot.last_error or "serial reader fault")

        if snapshot.discarded_packets != discarded_packets_at_start:
            raise RuntimeError("telemetry became invalid during calibration")

        latest_accepted_at = snapshot.latest_accepted_at
        if latest_accepted_at is None or (time.monotonic() - latest_accepted_at) > timeout_seconds:
            raise RuntimeError("telemetry became invalid during calibration")

        samples = telemetry_store.get_samples_since(received_after)
        if len(samples) >= sample_count:
            return samples[:sample_count]

        if time.monotonic() >= deadline:
            raise RuntimeError("timed out collecting calibration samples")

        time.sleep(0.01)


def capture_neutral_axes(
    telemetry_store: TelemetryStore,
    timeout_seconds: float,
    discarded_packets_at_start: int,
) -> PhysicalAxes:
    for attempt in range(1, NEUTRAL_CAPTURE_MAX_RETRIES + 1):
        input(
            "Hold the controller in neutral and press Enter to capture the neutral pose... "
        )
        capture_started_at = time.monotonic()
        samples = collect_calibration_samples(
            telemetry_store,
            sample_count=CALIBRATION_SAMPLE_COUNT,
            timeout_seconds=timeout_seconds,
            discarded_packets_at_start=discarded_packets_at_start,
            received_after=capture_started_at,
        )
        spread = physical_axes_spread(samples)
        max_spread = max(spread.yaw_cdeg, spread.pitch_cdeg, spread.roll_cdeg)
        if max_spread <= NEUTRAL_MAX_SPREAD_CDEG:
            return average_physical_axes(samples)

        print(
            "Neutral capture was unstable "
            f"(yaw={spread.yaw_cdeg / 100.0:.2f}deg, "
            f"pitch={spread.pitch_cdeg / 100.0:.2f}deg, "
            f"roll={spread.roll_cdeg / 100.0:.2f}deg). "
            f"Retry {attempt}/{NEUTRAL_CAPTURE_MAX_RETRIES}."
        )

    raise RuntimeError("neutral capture failed 5 times")


def detect_axis_mapping(
    neutral_axes: PhysicalAxes,
    motion_axes: PhysicalAxes,
) -> Optional[AxisMapping]:
    deltas = {
        AxisName.YAW: motion_axes.yaw_cdeg - neutral_axes.yaw_cdeg,
        AxisName.PITCH: motion_axes.pitch_cdeg - neutral_axes.pitch_cdeg,
        AxisName.ROLL: motion_axes.roll_cdeg - neutral_axes.roll_cdeg,
    }
    ranked_axes = sorted(deltas.items(), key=lambda item: abs(item[1]), reverse=True)
    leading_axis, leading_delta = ranked_axes[0]
    second_delta = abs(ranked_axes[1][1])

    if abs(leading_delta) < MOTION_MIN_DELTA_CDEG:
        return None

    if second_delta > 0 and (abs(leading_delta) / second_delta) < MOTION_DOMINANCE_RATIO:
        return None

    return AxisMapping(
        source_axis=leading_axis,
        sign=1 if leading_delta >= 0 else -1,
    )


def capture_axis_mapping(
    telemetry_store: TelemetryStore,
    neutral_axes: PhysicalAxes,
    logical_axis_name: str,
    timeout_seconds: float,
    discarded_packets_at_start: int,
) -> Optional[AxisMapping]:
    input(
        f"Move the controller in the direction that should mean positive {logical_axis_name}, "
        "hold it there, and press Enter... "
    )
    capture_started_at = time.monotonic()
    samples = collect_calibration_samples(
        telemetry_store,
        sample_count=CALIBRATION_SAMPLE_COUNT,
        timeout_seconds=timeout_seconds,
        discarded_packets_at_start=discarded_packets_at_start,
        received_after=capture_started_at,
    )
    return detect_axis_mapping(neutral_axes, average_physical_axes(samples))


def format_axis_mapping(axis_mapping: AxisMapping) -> str:
    sign_text = "+" if axis_mapping.sign >= 0 else "-"
    return f"{sign_text}{axis_mapping.source_axis.value}"


def run_guided_calibration(
    telemetry_store: TelemetryStore,
    timeout_seconds: float,
) -> CalibrationProfile:
    calibration_snapshot = telemetry_store.get_snapshot()
    discarded_packets_at_start = calibration_snapshot.discarded_packets

    while True:
        neutral_axes = capture_neutral_axes(
            telemetry_store,
            timeout_seconds=timeout_seconds,
            discarded_packets_at_start=discarded_packets_at_start,
        )

        pitch_mapping = capture_axis_mapping(
            telemetry_store,
            neutral_axes,
            logical_axis_name="pitch",
            timeout_seconds=timeout_seconds,
            discarded_packets_at_start=discarded_packets_at_start,
        )
        if pitch_mapping is None:
            print("Pitch capture was ambiguous. Restarting full calibration.")
            continue

        roll_mapping = capture_axis_mapping(
            telemetry_store,
            neutral_axes,
            logical_axis_name="roll",
            timeout_seconds=timeout_seconds,
            discarded_packets_at_start=discarded_packets_at_start,
        )
        if roll_mapping is None:
            print("Roll capture was ambiguous. Restarting full calibration.")
            continue

        yaw_mapping = capture_axis_mapping(
            telemetry_store,
            neutral_axes,
            logical_axis_name="yaw",
            timeout_seconds=timeout_seconds,
            discarded_packets_at_start=discarded_packets_at_start,
        )
        if yaw_mapping is None:
            print("Yaw capture was ambiguous. Restarting full calibration.")
            continue

        print("Calibration summary:")
        print(f"  logical pitch <- {format_axis_mapping(pitch_mapping)}")
        print(f"  logical roll  <- {format_axis_mapping(roll_mapping)}")
        print(f"  logical yaw   <- {format_axis_mapping(yaw_mapping)}")
        response = input("Accept calibration? [Y/n]: ").strip().lower()
        if response in ("", "y", "yes"):
            return CalibrationProfile(
                neutral_yaw_cdeg=neutral_axes.yaw_cdeg,
                neutral_pitch_cdeg=neutral_axes.pitch_cdeg,
                neutral_roll_cdeg=neutral_axes.roll_cdeg,
                yaw_mapping=yaw_mapping,
                pitch_mapping=pitch_mapping,
                roll_mapping=roll_mapping,
            )

        print("Calibration rejected. Restarting full calibration.")


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
    parser.add_argument(
        "--calibration-timeout-seconds",
        type=float,
        default=CALIBRATION_TIMEOUT_SECONDS,
        help="How long to wait for each neutral or motion calibration capture",
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
    calibration_profile = CalibrationProfile()
    last_controls = ControlState(roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0)
    last_processed_sample_at: Optional[float] = None
    telemetry_store = TelemetryStore()
    adapter = None

    if args.control_tick_hz <= 0.0:
        raise ValueError("--control-tick-hz must be greater than 0")

    control_tick_period = 1.0 / args.control_tick_hz
    next_control_tick = time.monotonic()

    try:
        with open_serial_port(
            args.port,
            args.baud_rate,
            args.startup_wait_seconds,
        ) as serial_port:
            serial_reader = SerialReader(serial_port, telemetry_store)
            serial_reader.start()

            try:
                wait_for_valid_telemetry(telemetry_store)

                if args.enable_krpc:
                    calibration_profile = run_guided_calibration(
                        telemetry_store,
                        timeout_seconds=args.calibration_timeout_seconds,
                    )
                    adapter = open_krpc_adapter(
                        host=args.krpc_host,
                        rpc_port=args.krpc_rpc_port,
                        stream_port=args.krpc_stream_port,
                        client_name=args.krpc_client_name,
                    )

                while args.max_packets == 0 or packet_count < args.max_packets:
                    snapshot = telemetry_store.get_snapshot()
                    if snapshot.reader_faulted:
                        raise RuntimeError(snapshot.last_error or "serial reader fault")

                    saw_valid_packet = False
                    for sample in telemetry_store.get_samples_since(last_processed_sample_at):
                        telemetry = sample.telemetry
                        controls = telemetry_to_controls(
                            telemetry,
                            angle_full_scale_deg=args.angle_full_scale_deg,
                            encoder_full_scale_ticks=args.encoder_full_scale_ticks,
                            calibration_profile=calibration_profile,
                        )
                        last_controls = controls
                        last_processed_sample_at = sample.received_at
                        saw_valid_packet = True
                        if adapter is None:
                            print(format_payload(telemetry, controls))
                        packet_count += 1
                        if args.max_packets != 0 and packet_count >= args.max_packets:
                            break

                    now = time.monotonic()
                    if adapter is not None and now >= next_control_tick:
                        latest_accepted_at = snapshot.latest_accepted_at
                        is_fresh = (
                            latest_accepted_at is not None
                            and (now - latest_accepted_at) <= args.timeout_seconds
                        )
                        if is_fresh:
                            adapter.apply(last_controls)
                        else:
                            adapter.safe_reset(throttle=last_controls.throttle)
                            if now - last_timeout_report >= 2.0:
                                print("waiting for telemetry: latest control sample is stale")
                                last_timeout_report = now

                        while next_control_tick <= now:
                            next_control_tick += control_tick_period

                    if not saw_valid_packet:
                        time.sleep(min(control_tick_period, 0.005))
            finally:
                serial_reader.stop()
                serial_reader.join(timeout=1.0)
    finally:
        if adapter is not None:
            adapter.safe_reset(throttle=last_controls.throttle)
            adapter.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())