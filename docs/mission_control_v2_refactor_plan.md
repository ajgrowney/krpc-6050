# Mission Control V2 Refactor Plan

## Objective

Refactor [mission_control.py](../mission_control.py) from a single-loop telemetry
reader into a V2 architecture with:

- a persistent serial reader thread
- shared latest-sample and recent-history telemetry state
- guided startup calibration and axis discovery
- delayed kRPC connection and arming
- host-side mapping from physical telemetry to logical vessel controls

The refactor should preserve packet decoding behavior and existing stale-input
fail-safe behavior while introducing the new startup flow in small, testable
steps.

## Current Structure

The current implementation has four main behaviors interleaved in one loop:

- serial port reads in [mission_control.py](../mission_control.py)
- framing and packet decode in [mission_control.py](../mission_control.py)
- direct telemetry-to-control mapping in [mission_control.py](../mission_control.py)
- timed kRPC output in [mission_control.py](../mission_control.py)

This structure is acceptable for V1, but it becomes brittle once the main
thread needs to block for operator prompts while telemetry intake continues.

## Target Structure

The V2 runtime should be split into these layers:

```text
serial port owner and reader thread
-> framed-packet parser
-> packet validator
-> telemetry decoder
-> synchronized telemetry store with recent history
-> validation gate
-> calibration flow
-> mapping layer
-> normalized control state
-> kRPC adapter
```

## Refactor Principles

- change one abstraction boundary at a time
- keep the program runnable after each phase
- avoid changing the wire format or Arduino assumptions
- keep the reader thread responsible only for telemetry acquisition
- keep kRPC writes and operator prompts in the main thread
- prefer adding new helpers before rewriting the main control loop

## Shared State Contract: `TelemetryStore`

### Purpose

`TelemetryStore` is the only shared boundary between the serial reader thread
and the main thread.

Its job is to publish accepted telemetry samples, expose a small recent-history
window for calibration, and report reader health without allowing either thread
to reach into the other's internals.

### Ownership Rules

- the serial reader thread is the only writer of accepted telemetry samples
- the main thread is the primary reader of snapshots and recent history
- configuration, calibration decisions, and kRPC output do not write directly
  into `TelemetryStore`
- the serial port handle, frame parser, and decode loop are never stored inside
  `TelemetryStore`

### Stored Data

`TelemetryStore` should hold at least:

- latest accepted `TelemetrySample`
- bounded history of recent accepted `TelemetrySample` values
- host timestamp of the most recent accepted sample
- count of consecutive valid packets
- count of discarded packets
- latest packet error text or category
- reader fault flag
- reader stop flag or closed flag if needed for shutdown visibility

### Sample Semantics

- a `TelemetrySample` represents one fully decoded, CRC-valid, version-valid
  packet plus a host-side monotonic receipt timestamp
- only accepted packets become `TelemetrySample` objects stored as latest sample
  or history entries
- invalid packets do not enter history
- packet timestamps must use host monotonic time, not wall-clock time

### History Semantics

- history stores accepted samples only
- history is bounded to a fixed maximum size
- history order is oldest to newest
- history must be sufficient for neutral averaging and short motion windows
- history reads should return a copy or immutable view, never a live internal
  buffer reference

### Counter Semantics

- `consecutive_valid_packets` increments only when an accepted sample is stored
- `consecutive_valid_packets` resets to zero on invalid packet receipt
- `discarded_packets` increments on every rejected packet
- `last_error` records the most recent rejection reason from decode or
  validation

### Reader Health Semantics

- `reader_faulted` becomes true when the reader thread encounters a fatal serial
  or runtime error and can no longer continue acquisition
- ordinary packet-level decode failures do not set `reader_faulted`; they only
  affect counters and `last_error`
- if `reader_faulted` is true, the main thread must treat telemetry as unusable
  for calibration or KSP output

### Snapshot API

`TelemetryStore` should expose a small, explicit API.

Recommended methods:

- `record_valid_sample(sample)`
- `record_invalid_packet(error_text, received_at)`
- `record_reader_fault(error_text, received_at)`
- `get_snapshot()`
- `get_recent_samples(max_age_seconds=None, max_count=None)`
- `reset_validation_counters()` only if startup flow needs an explicit phase reset

`get_snapshot()` should return one atomic `TelemetrySnapshot` object containing:

- latest accepted sample, if any
- latest accepted timestamp, if any
- consecutive valid count
- discarded packet count
- last error
- reader fault state

The main thread should be able to make freshness and startup decisions from a
single returned snapshot without performing multiple lock acquisitions.

### Atomicity Rules

- writes to latest sample, counters, timestamps, and history must occur under
  one lock-protected update path
- `get_snapshot()` must observe a self-consistent state from one logical moment
- `get_recent_samples()` must not interleave partial history mutation with a
  read result

### Freshness Rules

`TelemetryStore` should not decide whether telemetry is fresh relative to the
CLI timeout value.

Instead:

- the store publishes timestamps and raw health facts
- the main thread computes freshness using the latest accepted timestamp and the
  configured stale timeout

This keeps the store policy-light and easier to test.

### Fault Boundary

`TelemetryStore` is not responsible for:

- opening or closing the serial port
- prompting the user
- performing axis mapping
- applying neutral offsets
- writing to kRPC

It is strictly a synchronized telemetry publication layer.

### Initial Implementation Constraint

During phase 3, the store should be introduced even before the reader thread
exists. The current single-threaded main loop can write into it first.

That allows the API and invariants to stabilize before threading is added in
phase 4.

## Startup Failure Policy

The first implementation should use a strict fail-fast startup policy.

### Telemetry Validation

- the startup validation gate must time out after 10 seconds if no valid
  telemetry stream is established
- if validation times out, the program exits immediately with a non-zero status
- if the telemetry stream becomes invalid during calibration, the program exits
  immediately with a non-zero status

This keeps startup behavior simple and avoids partial calibration on an unstable
input stream.

### Reader Thread Failures

- if the serial reader thread faults, the program exits immediately with a
  non-zero status
- no automatic serial reconnect is attempted in the first implementation

Reader-thread failure is treated as a fatal acquisition failure, not a
recoverable packet-level issue.

### Neutral Capture Failure

- neutral capture may retry up to 5 times
- if all 5 neutral-capture attempts fail, the program exits immediately with a
  non-zero status

Neutral capture failure means the sampled neutral window is too unstable or
otherwise fails its acceptance criteria.

### Axis-Motion Ambiguity

- if any axis-motion capture is ambiguous, the user is notified
- ambiguity during axis discovery returns the operator to the start of the full
  calibration process
- the restart includes neutral capture again, not just the failed axis step

This avoids mixing a fresh axis capture with stale assumptions from an earlier
neutral or partial mapping attempt.

### Calibration Rejection

- after the calibration summary is shown, operator rejection returns the user
  to the start of the full calibration process
- rejection does not proceed to kRPC connection or arming

For the first implementation, calibration rejection is treated as a request to
redo calibration rather than to edit one axis interactively.

### Policy Summary

The startup path should therefore behave as follows:

- no valid telemetry within 10 seconds: exit immediately
- telemetry becomes invalid during calibration: exit immediately
- reader thread faults: exit immediately
- neutral capture fails 5 times: exit immediately
- ambiguous axis-motion capture: notify user and restart full calibration
- calibration summary rejected: restart full calibration

## Phase 1: Introduce Runtime Data Structures

### Purpose

Create the data model needed for V2 without changing runtime behavior yet.

### Add

- `PhysicalAxes`: yaw, pitch, roll in centidegrees extracted from telemetry
- `AxisName`: logical identifier for `yaw`, `pitch`, and `roll`
- `AxisMapping`: source axis plus sign for one logical control axis
- `CalibrationProfile`: neutral offsets and axis mappings
- `TelemetrySample`: decoded telemetry plus host timestamp
- `TelemetrySnapshot`: latest sample, freshness data, counters, and health state

### Change

- keep `ControlState` as the logical control output object
- keep `KSPControlAdapter` unchanged for now

### Touched Areas

- top-level dataclass section in [mission_control.py](../mission_control.py)

### Validation

- run a Python syntax check on [mission_control.py](../mission_control.py)
- confirm there is no change to current runtime behavior

## Phase 2: Isolate Physical Telemetry Extraction

### Purpose

Make physical telemetry explicit before introducing calibration or mapping.

### Add

- `extract_physical_axes(telemetry) -> PhysicalAxes`

### Change

- refactor `telemetry_to_controls` so it no longer reads protobuf fields
  directly
- initially keep identity mapping so logical roll/pitch/yaw still map straight
  from physical roll/pitch/yaw

### Touched Areas

- `telemetry_to_controls` in [mission_control.py](../mission_control.py)
- new helper near packet decode and mapping helpers in [mission_control.py](../mission_control.py)

### Validation

- run the existing script in non-kRPC mode against the live stream
- confirm printed normalized outputs match current behavior

## Phase 3: Introduce Shared Telemetry Store

### Purpose

Create a synchronized boundary between telemetry acquisition and control logic.

### Add

- `TelemetryStore` class
- lock-protected latest sample
- short rolling history buffer for calibration windows
- counters for consecutive valid packets, discarded packets, and last error

### Change

- no thread yet in this phase if you want the smallest safe step
- the current main loop writes accepted samples into the store instead of only
  local variables

### Touched Areas

- helper/class section in [mission_control.py](../mission_control.py)
- main loop sample handling in [mission_control.py](../mission_control.py)

### Validation

- run the live stream and confirm the program still prints payloads
- verify stale timeout behavior is unchanged

## Phase 4: Move Serial Intake Into Reader Thread

### Purpose

Separate persistent serial acquisition from prompt-driven main-thread logic.

### Add

- `SerialReader` class that owns:
  - serial port handle
  - framed packet parser
  - stop event
  - telemetry store reference
  - background thread lifecycle

### Change

- move byte reads, framing, CRC validation, and telemetry decode out of `main`
- reader thread publishes accepted `TelemetrySample` objects into `TelemetryStore`
- main thread stops touching the serial port directly

### Touched Areas

- serial helper section in [mission_control.py](../mission_control.py)
- `open_serial_port` usage in [mission_control.py](../mission_control.py)
- `main` control flow in [mission_control.py](../mission_control.py)

### Validation

- run against the live serial stream with kRPC disabled
- confirm packets continue arriving while the main thread is idle
- verify clean shutdown leaves the port closed

## Phase 5: Add Telemetry Validation Gate

### Purpose

Gate startup progression on a stable run of valid telemetry.

### Add

- `wait_for_valid_telemetry(store, min_packets, timeout_seconds)`

### Change

- main startup waits for a configured number of consecutive valid packets before
  calibration or kRPC setup begins

### CLI Additions

- `--validation-packets`
- optionally `--validation-timeout-seconds`

### Touched Areas

- argument parser in [mission_control.py](../mission_control.py)
- startup portion of `main` in [mission_control.py](../mission_control.py)

### Validation

- test with a healthy stream and verify the gate passes
- test with the serial device absent or corrupted input and verify startup does
  not arm output

## Phase 6: Add Neutral Capture

### Purpose

Capture a stable operator-defined no-input pose.

### Add

- `capture_neutral_profile(store, sample_count)`
- averaging over a short recent-history window

### Change

- no axis remap yet if you want a smaller first calibration step
- `telemetry_to_controls` begins subtracting neutral offsets before normalization

### CLI Additions

- `--calibration-sample-count`

### Touched Areas

- mapping helpers in [mission_control.py](../mission_control.py)
- startup/calibration section of `main` in [mission_control.py](../mission_control.py)

### Validation

- confirm neutral hold causes near-zero control outputs after calibration
- confirm off-neutral orientation produces expected signed output

## Phase 7: Add Guided Axis Discovery

### Purpose

Discover swaps and inversions from observed motion rather than hard-coded rules.

### Add

- `prompt_for_motion(prompt_text)`
- `capture_motion_window(store, duration_seconds)`
- `detect_axis_mapping(baseline_window, motion_window)`
- `run_guided_calibration(store, args) -> CalibrationProfile`

### Change

- calibration becomes a multi-step interactive flow:
  - neutral
  - positive pitch
  - positive roll
  - positive yaw
  - summary and confirmation

### Touched Areas

- calibration helper section in [mission_control.py](../mission_control.py)
- startup/calibration portion of `main` in [mission_control.py](../mission_control.py)

### Validation

- confirm prompts can block while the reader thread continues ingesting data
- confirm a deliberate pitch movement can map to logical roll when desired
- confirm ambiguous captures trigger a retry instead of silently accepting bad data

## Phase 8: Convert Mapping Layer To Profile-Driven Logic

### Purpose

Make control generation consume a `CalibrationProfile` instead of assuming
identity mapping.

### Add

- `map_physical_to_logical(physical_axes, calibration_profile)`
- optional deadband helper if needed during the same pass

### Change

- `telemetry_to_controls` becomes a profile-driven conversion function
- roll, pitch, and yaw each select a source axis and sign from the profile
- throttle remains based on encoder ticks and stays independent of IMU mapping

### Touched Areas

- `telemetry_to_controls` in [mission_control.py](../mission_control.py)
- formatting/diagnostic output in [mission_control.py](../mission_control.py)

### Validation

- confirm identity mappings reproduce V1 behavior
- confirm swapped mappings produce the requested control behavior
- confirm sign inversions only affect the targeted axis

## Phase 9: Delay kRPC Connection Until Calibration Completes

### Purpose

Align startup behavior with the V2 design document.

### Change

- move `open_krpc_adapter` later in `main`
- connect only after telemetry validation and calibration succeed
- keep the system disarmed if calibration fails or is rejected

### Touched Areas

- `main` startup sequence in [mission_control.py](../mission_control.py)

### Validation

- confirm startup can observe telemetry with no kRPC connection yet
- confirm kRPC is only opened after the calibration summary is accepted
- confirm calibration rejection exits cleanly without outputting vessel controls

## Phase 10: Rebuild Main As Explicit Startup And Runtime States

### Purpose

Make runtime control flow match the V2 design document and easier to reason
about during faults.

### Add

- explicit startup/runtime state tracking
- helper boundaries for:
  - startup and validation
  - calibration
  - active control loop
  - shutdown

### Change

- `main` becomes an orchestrator rather than a large mixed loop
- stale-input handling continues to reset roll, pitch, and yaw while holding
  throttle

### Touched Areas

- most of `main` in [mission_control.py](../mission_control.py)

### Validation

- run through normal startup
- unplug or interrupt telemetry and verify fail-safe reset still occurs
- confirm shutdown resets controls before closing kRPC

## Suggested Implementation Order

The safest order is:

1. phase 1: runtime data structures
2. phase 2: physical telemetry extraction
3. phase 3: shared telemetry store
4. phase 4: reader thread
5. phase 5: validation gate
6. phase 6: neutral capture
7. phase 8: profile-driven mapping with identity defaults
8. phase 7: guided axis discovery
9. phase 9: delayed kRPC connection
10. phase 10: explicit runtime states

Phase 8 intentionally comes before full guided mapping if you want the first
mapping-capable build to use a hand-constructed identity profile and stay easy
to debug.

## CLI Plan

Recommended new arguments for [mission_control.py](../mission_control.py):

- `--validation-packets`
- `--validation-timeout-seconds`
- `--calibration-sample-count`
- `--motion-capture-seconds`
- `--interactive-calibration`

Possible later additions, but not required for the first pass:

- `--skip-calibration`
- `--deadband`
- `--save-profile`
- `--load-profile`

## Risks To Watch

- reader-thread shutdown races can leave the serial port open
- sample-history windows can include stale data if timestamps are not enforced
- prompt flows can accept ambiguous motion unless dominance thresholds are explicit
- yaw detection can be noisier than roll and pitch due to gyro integration drift
- refactoring `main` too early can hide regressions in stale/fault handling

## First Executable Milestone

The first milestone worth shipping is:

- reader thread is active
- latest telemetry is published through `TelemetryStore`
- validation gate works
- neutral capture works
- identity mapping profile works
- kRPC connection happens after validation and neutral capture

That milestone gives you the safer startup model before full automatic axis
discovery is implemented.