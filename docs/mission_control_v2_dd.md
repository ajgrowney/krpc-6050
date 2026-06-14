# Mission Control V2 Design Document
## Calibration-First Host Control Bridge

1. Overview

This document defines Mission Control V2, a host-side control bridge that keeps
a persistent serial reader active, validates framed telemetry from the Arduino
controller, guides the operator through an input-mapping calibration flow, and
only then enables output to Kerbal Space Program via kRPC.

Mission Control V2 keeps the V1 packet format and the V1 normalized control
model, but changes startup policy. In V2, the host does not connect to kRPC or
emit vessel controls until the controller is calibrated, telemetry is proven
valid, and the operator's axis conventions are known.

2. Goals

Mission Control V2 must:

- preserve the existing Arduino telemetry packet format
- keep serial intake running continuously once the port is opened
- separate telemetry observation from KSP actuation
- wait for validated telemetry before asking the operator to calibrate mappings
- allow host-side axis swapping and sign inversion without changing firmware
- arm KSP output only after calibration and mapping are complete
- remain simple enough to port from Python to C++ on the Raspberry Pi

Mission Control V2 does not attempt to:

- change IMU fusion or the Arduino-side sensor model
- redefine the wire protocol for controller-specific mounting quirks
- infer mappings from vessel behavior after kRPC output begins
- add autopilot, SAS logic, or closed-loop stabilization

3. System Context

The control path is:

```text
Arduino controller
-> USB serial framed telemetry
-> serial reader thread
-> shared telemetry snapshot and recent sample history
-> Mission Control observation and validation
-> host-side calibration and axis mapping
-> Mission Control arming gate
-> kRPC client
-> active KSP vessel
```

The Arduino remains responsible for sensor sampling and packet framing.
Mission Control V2 is responsible for continuous intake, validation, startup
sequencing, neutral capture, axis mapping, freshness handling, arming, and KSP
output.

4. V2 Control Philosophy

Mission Control V2 treats the Arduino telemetry as physical truth and the host
mapping layer as operator truth.

That distinction is important:

- physical truth: what the IMU and encoder actually measured
- operator truth: what the human expects roll, pitch, yaw, and throttle to do

If the handheld controller is mounted differently, the wire protocol should not
be rewritten to hide that fact. Instead, the host mapping layer should translate
physical telemetry into the operator's intended KSP control directions.

5. Telemetry Input Contract

Mission Control V2 consumes the same framed telemetry packet as V1.

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

6. Startup State Machine

Mission Control V2 startup should be organized as explicit states.

Recommended states:

- boot: process arguments and initialize host resources
- serial_connect: open the serial port, start the reader thread, and wait for the Arduino to settle
- observe: consume the shared telemetry stream but do not arm output
- telemetry_validated: confirm that a stable run of valid packets is arriving
- calibration_prompt: instruct the operator through neutral and axis mapping
- mapped_ready: mapping data is complete but KSP output is still disabled
- krpc_connect: establish the kRPC connection only after mapping succeeds
- armed: start scheduled control writes to the active vessel
- faulted: suppress KSP output until the fault clears or the operator restarts

This state machine makes startup intent explicit and prevents accidental output
to KSP before the controller has been characterized.

The reader thread remains active across all startup states after
serial_connect. Calibration and output code must not own the serial port
directly.

7. Validation Before Calibration

Mission Control V2 should not ask the operator to map controls until it has high
confidence that the packet stream is usable.

Recommended validation gate:

- receive a minimum run of valid packets before continuing
- require packet CRC to pass for that run
- require supported packet version for that run
- require freshness to remain within the stale timeout for that run
- require the initial validation gate to succeed within 10 seconds

Recommended initial threshold:

- 20 consecutive valid packets

If validation fails, the host exits without arming output.

Validation should consume data from the reader thread's published telemetry
state rather than pausing serial reads while the main thread waits.

8. Guided Calibration Flow

Mission Control V2 should guide the operator through a short calibration flow.

Recommended sequence:

- step 1: wait for the Arduino's IMU startup calibration to finish
- step 2: verify that valid packets are arriving continuously
- step 3: ask the operator to hold the controller in neutral
- step 4: record the neutral yaw, pitch, and roll telemetry values
- step 5: ask the operator to perform one deliberate motion for each axis
- step 6: infer host-side axis mapping and sign from observed telemetry changes
- step 7: show a summary and require confirmation before enabling kRPC output

This approach is better than asking the operator to manually type an axis table.
It reduces coordinate-frame mistakes and makes the system easier to use with
different physical controller orientations.

The calibration flow should rely on a recent sample window collected by the
reader thread. The main thread may block for prompts, but serial acquisition
must continue throughout the prompt cycle.

If telemetry becomes invalid during calibration, Mission Control V2 aborts
startup and exits without arming output.

9. Neutral Capture Contract

Neutral is established before output is armed.

At neutral capture time the operator holds the controller in the intended no-
input pose. Mission Control records:

- neutral_yaw_cdeg
- neutral_pitch_cdeg
- neutral_roll_cdeg

All later manual commands are computed as offsets from these neutral values.

Calibration assumptions:

- the controller is physically still during neutral capture
- the pose held at capture is the desired no-input pose
- neutral capture happens after telemetry has been validated
- KSP output remains disabled until neutral capture succeeds

Recommended startup policy:

- neutral capture may retry up to 5 times
- if all neutral-capture attempts fail, startup aborts and the program exits

10. Axis Discovery Contract

Mission Control V2 should derive mapping from observed motion, not from packet
field names alone.

Recommended guided prompts:

- move the controller in the direction that should mean positive pitch
- move the controller in the direction that should mean positive roll
- move the controller in the direction that should mean positive yaw

For each prompt, Mission Control observes which physical telemetry axis changes
most and whether that change is positive or negative.

Recommended observation method:

- record a short baseline window before the prompt
- record a short motion window after the operator performs the requested action
- compare the windows to find the dominant telemetry-axis delta
- reject ambiguous captures and retry the prompt

Recommended startup policy:

- ambiguous axis capture notifies the operator
- ambiguity restarts the full calibration flow, including neutral capture
- operator rejection of the calibration summary also restarts the full
  calibration flow

The mapping result for each logical control axis consists of:

- source telemetry axis: one of yaw, pitch, or roll
- sign: positive or inverted

Example outcomes:

- logical roll may come from physical pitch with positive sign
- logical pitch may come from physical roll with positive sign
- logical yaw may come from physical yaw with inverted sign

11. Mapping Layer Contract

Mission Control V2 should keep a dedicated mapping layer between decoded
telemetry and normalized control state.

Responsibilities of the mapping layer:

- subtract neutral offsets from telemetry
- apply discovered axis selection for each logical output axis
- apply sign inversion where needed
- normalize each mapped value into the range -1.0 to +1.0
- apply deadband if configured

This layer is the correct place to implement a pitch-to-roll swap or any other
mounting-dependent remapping.

The mapping layer must remain a pure consumer of telemetry state. It must not
interact with the serial port or perform blocking prompt logic.

12. Normalized Control State Contract

Mission Control V2 defines an internal normalized control state with these
fields:

- roll: float in the range -1.0 to +1.0
- pitch: float in the range -1.0 to +1.0
- yaw: float in the range -1.0 to +1.0
- throttle: float in the range 0.0 to 1.0
- sequence: latest accepted telemetry sequence number
- status_mask: latest accepted telemetry status bits
- is_fresh: true only when derived from a recent valid packet
- is_armed: true only when output to KSP is allowed
- mapping_ready: true only when neutral and axis mapping are complete
- faulted: true when output should be suppressed due to stale or invalid input

This state remains the boundary between input decoding and KSP output.

13. Throttle Contract

Throttle remains stateful and independent of the IMU axis-mapping flow.

Mission Control V2 should continue to interpret encoder ticks as a bounded host-
side throttle state.

Behavioral requirements:

- throttle starts at a configured initial value, normally 0.0
- throttle remains in the range 0.0 to 1.0
- stale or invalid packets do not modify throttle state
- neutral and axis mapping do not alter throttle semantics

Optional future extension:

- operator-confirmed throttle direction inversion if encoder installation varies

14. Arming and kRPC Connection Policy

Mission Control V2 must separate observation from actuation more strictly than
V1.

Recommended policy:

- serial input may begin immediately after startup
- telemetry decoding may begin before kRPC is connected and should continue in the background
- calibration may begin only after telemetry is validated
- kRPC connection may begin only after mapping_ready is true
- armed output may begin only after kRPC connection succeeds

KSP output is allowed only when all are true:

- the operator has completed calibration
- mapping_ready is true
- the latest control state is fresh
- the system is not faulted
- a supported packet version is being received
- the kRPC session is connected successfully

When disarmed:

- telemetry parsing continues
- calibration data may be collected or reviewed
- control-state updates continue
- KSP control outputs remain suppressed

15. Freshness and Fault Handling

Mission Control V2 must treat stale or invalid telemetry as unusable control
input even after mapping is complete.

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
- when valid packets resume, normal observation resumes immediately
- KSP output remains suppressed until freshness is restored

When input is stale while armed:

- is_fresh becomes false
- faulted becomes true
- roll, pitch, and yaw are driven to 0.0
- throttle holds its last accepted value

Reader-thread failures are treated as telemetry faults. If the reader stops,
the system must transition to faulted, suppress KSP output, and abort startup
if arming has not yet completed.

16. Recommended Implementation Layers

Mission Control V2 should be implemented as separate layers:

```text
serial port owner and reader thread
-> framed-packet parser
-> packet validator
-> telemetry decoder
-> shared latest-sample store and short history buffer
-> telemetry validation gate
-> neutral and axis calibration flow
-> mapping layer
-> normalized control state
-> kRPC adapter
```

Responsibilities by layer:

- serial port owner and reader thread: keep serial intake alive and publish decoded samples
- framed-packet parser: locate sync and read complete frames
- packet validator: verify version, CRC, and packet shape
- telemetry decoder: populate RealTelemetryPayload
- shared latest-sample store and short history buffer: provide synchronized access to current and recent telemetry
- telemetry validation gate: decide when startup may advance
- neutral and axis calibration flow: derive offsets, axis sources, and signs
- mapping layer: convert physical telemetry to operator-intent controls
- normalized control state: store the current accepted command set
- kRPC adapter: write bounded commands to the active vessel

17. Acceptance Criteria

Mission Control V2 is considered ready for integration when:

- it can read framed serial telemetry continuously
- it rejects corrupted packets without crashing
- it keeps serial reads alive while the main thread is waiting for operator input
- it waits for a stable valid telemetry run before prompting the operator
- it captures a neutral pose before arming
- it derives axis swaps and sign inversions on the host side
- it does not connect to kRPC until calibration succeeds
- it suppresses KSP output when stale, invalid, or uncalibrated
- it resumes normal output automatically when valid packets return

18. Out of Scope for V2

The following are intentionally deferred:

- persistent storage of calibration profiles
- multiple controller profiles or per-vessel presets
- automatic remapping from long-term behavior analysis
- bidirectional haptic or display feedback to the controller
- autonomous flight modes, PID loops, or SAS integration logic

19. Conclusion

Mission Control V2 keeps the existing telemetry protocol but improves operator
safety and usability by moving controller interpretation into an explicit host-
side startup and mapping flow. The Arduino continues to report physical motion.
Mission Control determines how that motion should drive KSP only after telemetry
has been validated and the operator's intended control directions are known.