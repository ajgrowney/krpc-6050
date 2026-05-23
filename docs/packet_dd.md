# Telemetry Packet Design Decision Document
## Fixed Binary Packet Format for IMU + Encoder Telemetry

1. Overview
This document defines the telemetry packet format used to transmit IMU (yaw, pitch, roll) and rotary encoder data from the Arduino‑based sensor module to the host system over a UART link. After evaluating multiple encoding strategies, we selected a fixed binary packet to maximize throughput and minimize overhead on a bandwidth‑limited serial channel.

2. System Context
The telemetry originates from:
IMU providing yaw, pitch, roll
Rotary encoder providing position/count
Status bitmask indicating system state (sensor health, flags, etc.)
Data is transmitted over:
Arduino USB CDC ACM → /dev/ttyACM0
Forwarded over SSH to the Mac
Consumed by a host application for visualization or logging
The UART link is the primary bottleneck, with effective throughput of ~11.25 kB/s at 115200 baud.

3. Requirements
Telemetry must be:
- High‑rate (IMU updates at 50–200 Hz)
- Low‑latency
- Compact (UART bandwidth is limited)
- Deterministic (fixed offsets for fast parsing)
- Robust (status bits included every packet)
- Flexibility (adding/removing fields dynamically) is not a primary requirement.

4. Packet Format Decision
We evaluated two packet strategies:

A. Fixed Binary Packet: A single, static struct sent every cycle:
```
    [status_mask][yaw][pitch][roll][encoder]
```

B. Tagged/ID‑Based Packet

Self‑describing mini‑packets:

```
[status][sensor_id][sensor_value]
```

5. Tradeoff Analysis
🟦 Fixed Binary Packet (Chosen)
Pros
- Highest bandwidth efficiency
- Minimal CPU overhead on both ends
- No per‑field metadata
- Perfect for high‑rate IMU streaming
- Simple, deterministic parsing
Cons
- Less flexible
- Adding fields requires versioning

🟧 Tagged Packet (Not Chosen)
Pros
- Extensible and self‑describing
- Can send only changed fields
- Similar to OPC UA’s NodeId/DataValue model
Cons
- Higher overhead (extra bytes per field)
- More parsing logic
- Wastes bandwidth on UART links

### Why fixed binary wins here
Telemetry set is stable, known, and always sent together.
Link is bandwidth‑constrained.
IMU benefits from tight, predictable packet timing.

6. Final Packet Specification
Binary Layout

```
Byte 0      : status_mask (uint8_t)
Bytes 1–2   : yaw        (int16_t, fixed‑point or scaled)
Bytes 3–4   : pitch      (int16_t)
Bytes 5–6   : roll       (int16_t)
Bytes 7–10  : encoder    (int32_t)
-----------------------------------
Total size: 11 bytes
```

Transmission Characteristics
11 bytes per packet
110 bits on the wire (11 UART frames × 10 bits)
At 100 Hz: 1.1 kB/s (well within UART limits)
At 500 Hz: 5.5 kB/s (still safe at 115200 baud)

7. Comparison to Industrial Protocols

The fixed binary packet resembles OPC DA: static layout, no per‑field metadata, high throughput.

The tagged packet resembles OPC UA: self‑describing, flexible, but with higher overhead.

8. Future Extensions
If new sensors are added, introduce:

[packet_version]
as the first byte, then extend the struct.
Older parsers can reject or adapt based on version.
9. Conclusion
The fixed binary packet format provides the best balance of speed, simplicity, and determinism for your IMU + encoder telemetry system. It minimizes UART overhead, supports high update rates, and keeps parsing trivial on both the Arduino and the Mac.
If you want, I can generate the Arduino struct, the serialization code, and a Mac‑side parser in Python, Swift, or C++—just tell me which language you want next.