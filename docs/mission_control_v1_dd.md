# Mission Control V1 Design Document
## Arduino IMU + Encoder to KSP Control Bridge

1. Overview

This document defines Mission Control V1, the host-side control bridge that reads
framed telemetry from the Arduino controller, converts that telemetry into a
normalized control state, and applies those controls to Kerbal Space Program via
kRPC.

Mission Control V1 is intentionally narrow in scope. It is a manual control
bridge, not an autopilot. Its job is to turn operator motion and encoder input
into stable, bounded realtime control commands for roll, pitch, yaw, and
throttle.

2. Goals

Mission Control V1 must:

- read the Arduino framed binary telemetry stream reliably
- validate packet integrity before using any control sample
- define a stable neutral pose through IMU calibration
- convert raw telemetry into normalized manual control commands
- drive KSP controls at a fixed host-side control rate
- fail safe on stale or invalid telemetry
- remain simple enough to port from Python to C++ on the Raspberry Pi

Mission Control V1 does not attempt to:

- perform autonomous guidance or autopilot functions
- estimate absolute heading from the IMU
- fuse external navigation sources
- support multiple input devices or multiple vessels

3. System Context

The control path is:

```text
Arduino controller
-> USB serial framed telemetry
-> Mission Control
-> kRPC client
-> active KSP vessel
```

The Arduino is responsible for sensor sampling and packet framing.
Mission Control is responsible for packet validation, calibration, control
mapping, freshness handling, arming, and KSP output.

4. Control Philosophy

Mission Control V1 is a manual input adapter.

The operator defines attitude intent through handheld motion:

- tilt-right means positive roll
- tilt-up means positive pitch
- clockwise twist means positive yaw

Throttle is not an absolute position sensor. It is a stateful control derived
from incremental rotary encoder movement and is clamped to the range 0.0 to 1.0.

The system interprets the IMU relative to a calibrated neutral pose rather than
assuming the hardware is perfectly mounted or aligned at manufacture time.

5. Telemetry Input Contract

Mission Control V1 consumes the framed telemetry packet defined in the packet
design documents.

Expected packet layout:

```text
Byte 0   : SYNC0 = 0xAA
Byte 1   : SYNC1 = 0x55
Byte 2   : packet_version
Bytes 3-4: sequence (uint16, little-endian)
Byte 5   : status_mask
Bytes 6-7: yaw_cdeg (int16, little-endian)
Bytes 8-9: pitch_cdeg (int16, little-endian)
Bytes 10-11: roll_cdeg (int16, little-endian)
Bytes 12-15: encoder_ticks (int32, little-endian)
Byte 16  : crc8
```

Packet acceptance rules:

- sync word must match
- packet version must be supported
- CRC must validate
- packet must arrive before stale timeout expires

Sequence gaps are logged as health information but do not invalidate a newer
packet by themselves.

6. Control State Contract

Mission Control V1 defines an internal normalized control state with these
fields:

- roll: float in the range -1.0 to +1.0
- pitch: float in the range -1.0 to +1.0
- yaw: float in the range -1.0 to +1.0
- throttle: float in the range 0.0 to 1.0
- sequence: latest accepted telemetry sequence number
- status_mask: latest accepted telemetry status bits
- is_fresh: true only when derived from a recent valid packet
- is_armed: true only when output to KSP is allowed
- faulted: true when output should be suppressed due to stale or invalid input

This control state is the boundary between input decoding and KSP output.
It is not raw telemetry and it is not a kRPC API object.

7. Neutral Calibration

Neutral is established during calibration.

At calibration time the operator holds the controller in the intended neutral
pose. Mission Control records:

- neutral_roll_cdeg
- neutral_pitch_cdeg
- neutral_yaw_cdeg

All later manual attitude commands are computed as offsets from these neutral
values.

This allows the handheld device to feel natural even if the IMU is not mounted
perfectly square within the enclosure.

Calibration assumptions:

- the controller is physically still during calibration
- the pose held at calibration is the desired no-input pose
- calibration completes before controls are armed

8. Axis Mapping and Sign Convention

Mission Control V1 uses the following operator conventions:

- tilt-right produces positive roll
- tilt-up produces positive pitch
- clockwise twist produces positive yaw

These are behavioral requirements. If the IMU mounting or enclosure orientation
causes a sign inversion or axis swap, the mapping layer must correct that before
values enter the normalized control state.

The convention is defined by operator experience, not by raw sensor axis labels.

9. Normalization Rules

Each attitude axis is computed from the difference between the current telemetry
value and the calibrated neutral value.

The normalization rule is:

```text
command = clamp((measured - neutral) / full_scale, -1.0, 1.0)
```

Recommended initial full-scale values:

- roll full-scale: 45 degrees
- pitch full-scale: 45 degrees
- yaw full-scale: 45 degrees of relative twist

Recommended initial deadband:

- roll deadband: 0.05
- pitch deadband: 0.05
- yaw deadband: 0.05

Deadband rule:

- if abs(command) is less than deadband, output 0.0
- otherwise pass the command through, optionally with later smoothing

10. Throttle Contract

Throttle is incremental and stateful.

Mission Control does not interpret encoder ticks as an absolute throttle position.
Instead, each accepted encoder change modifies a stored throttle state.

The update rule is:

```text
throttle_next = clamp(throttle_prev + gain_per_tick * delta_ticks, 0.0, 1.0)
```

Recommended starting value:

- gain_per_tick: 0.01 to 0.02 per encoder detent

Behavioral requirements:

- throttle starts at a configured initial value, normally 0.0
- throttle remains in the range 0.0 to 1.0
- stale or invalid packets do not modify throttle state
- encoder pushbutton behavior must be explicit if enabled

Recommended initial pushbutton behavior:

- reset throttle to 0.0

11. Freshness and Fault Handling

Mission Control must treat invalid or stale telemetry as unusable control input.

Recommended stale timeout:

- 150 ms to 250 ms

Invalid sample conditions:

- bad CRC
- unsupported packet version
- malformed frame

Handling rules:

- invalid packets are discarded
- stale packets are ignored
- sequence gaps are logged but are not fatal by themselves
- when valid packets resume, normal control updates resume immediately

When input is stale:

- is_fresh becomes false
- faulted becomes true
- roll, pitch, and yaw are driven to 0.0
- throttle holds its last accepted value

12. Arming Rules

Mission Control must separate observation from actuation.

The system may decode and display telemetry while disarmed, but it must not write
control values to KSP until explicitly armed.

KSP output is allowed only when all are true:

- the controller is armed
- the latest control state is fresh
- the system is not faulted
- a supported packet version is being received

When disarmed:

- telemetry parsing continues
- control-state updates continue
- KSP control outputs remain suppressed

13. KSP Output Contract

Mission Control writes one coherent set of control values on each host-side
control tick.

Required output fields:

- vessel.control.roll
- vessel.control.pitch
- vessel.control.yaw
- vessel.control.throttle

Recommended control-loop rate:

- 20 Hz to 50 Hz

Mission Control should not write to kRPC on every serial byte event. It should
use the latest accepted control state on each scheduled control update.

14. Initial Operating Policy

Mission Control V1 should start with the simplest viable behavior:

- direct manual control only
- no autopilot logic
- no heading-hold logic
- no absolute yaw interpretation
- no control mixing with SAS-dependent behavior

This keeps failures easy to reason about and makes later C++ porting much simpler.

15. Recommended Implementation Layers

Mission Control V1 should be implemented as separate layers:

```text
serial framing reader
-> packet validator
-> telemetry decoder
-> calibration and mapping layer
-> normalized control state
-> kRPC adapter
```

Responsibilities by layer:

- serial framing reader: locate sync and read complete frames
- packet validator: verify version, CRC, and packet shape
- telemetry decoder: populate RealTelemetryPayload
- calibration and mapping layer: convert telemetry to operator-intent controls
- normalized control state: store current accepted command set
- kRPC adapter: write bounded commands to the active vessel

16. V1 Acceptance Criteria

Mission Control V1 is considered ready for initial KSP integration when:

- it can read framed serial telemetry continuously
- it rejects corrupted packets without crashing
- it establishes a neutral pose through calibration
- it produces stable normalized roll, pitch, yaw, and throttle values
- it suppresses KSP output when stale or faulted
- it resumes control output automatically when valid packets return
- it drives KSP controls at a fixed host-side loop rate

17. Out of Scope for V1

The following are intentionally deferred:

- autonomous flight modes
- PID or guidance loops
- magnetometer or absolute heading integration
- multiple controller profiles
- persistent calibration storage
- bidirectional telemetry feedback from KSP to the controller

18. Conclusion

Mission Control V1 is a bounded, manual-control bridge from Arduino telemetry to
KSP controls. Its design favors predictability, operator intuition, and clean
layering over sophistication. That is the right choice for the first host-side
integration and for the later C++ port to the Raspberry Pi.