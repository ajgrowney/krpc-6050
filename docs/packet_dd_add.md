
# 📄 **Telemetry Packet Design — Reliability & Interoperability Addendum**

This addendum addresses four critical issues identified in the initial packet design:  
**(1) framing/resynchronization, (2) field specification, (3) sequencing/timestamps, (4) versioning.**

The goal is to preserve the **fixed binary packet** (for efficiency) while adding the **minimum required structure** for correctness, robustness, and forward compatibility.

---

# 1. **Framing & Resynchronization (High Priority)**  
UART streams are byte streams with no inherent boundaries.  
A single dropped or inserted byte will corrupt parsing indefinitely unless the protocol includes a **framing mechanism**.

### ✔ Decision: Add a 2‑byte sync word + CRC‑8  
This adds only **3 bytes** of overhead and solves:

- packet boundary detection  
- resynchronization after corruption  
- validation of packet integrity  

### **Updated packet envelope**
```
[SYNC0][SYNC1][PAYLOAD...][CRC8]
```

### Recommended sync word
```
0xAA 0x55
```

These values have good bit‑transition properties and low accidental occurrence probability.

### CRC‑8 definition
Use CRC-8/MAXIM exactly:

- polynomial: 0x31
- init: 0x00
- refin: true
- refout: true
- xorout: 0x00
- coverage: bytes 2-15 inclusive (VERSION through ENCODER)

This removes ambiguity so every implementation produces the same CRC byte.

---

# 2. **Field Specification & Interoperability (High Priority)**  
The original spec left yaw/pitch/roll “fixed‑point or scaled,” which is too vague for independent implementations.

### ✔ Decision: Fully specify units, scale, and byte order

### **Final field definitions**
| Field | Type | Units | Scale | Range | Byte Order |
|------|------|--------|--------|--------|-------------|
| status_mask | uint8 | bitmask | 1:1 | 0–255 | little‑endian |
| yaw | int16 | degrees | ×100 | −327.68° to +327.67° | little‑endian |
| pitch | int16 | degrees | ×100 | same | little‑endian |
| roll | int16 | degrees | ×100 | same | little‑endian |
| encoder | int32 | ticks | 1:1 | full 32‑bit | little‑endian |

### Example  
Yaw = 25.34° -> stored as decimal `2534`, which is `0x09E6` as a 16-bit value and transmitted on the wire as bytes `E6 09` in little-endian order.

This guarantees that **any host implementation** will decode the same values.

---

# 3. **Sequence Number / Timestamp (Medium Priority)**  
Without a sequence number or timestamp:

- dropped packets cannot be detected  
- timing reconstruction on the host is impossible  
- logging becomes ambiguous  

### ✔ Decision: Add a 16‑bit sequence counter  
- increments every packet  
- wraps at 65535  
- adds only 2 bytes  
- enables dropped-packet detection and packet ordering  

Timing reconstruction still requires either a defined sample cadence or a timestamp. If you later need absolute or irregular timing, add a timestamp in a v2 packet or record host receive times.

---

# 4. **Versioning & Forward Compatibility (Medium Priority)**  
The original plan to replace `status_mask` with `packet_version` breaks fixed offsets and prevents old parsers from identifying new layouts.

### ✔ Decision: Add a dedicated version byte **inside the payload**, not at the front  
This preserves:

- sync word at byte 0  
- fixed offsets within v1 packets  
- safe rejection of unsupported packet versions  

### Placement  
Put `packet_version` immediately after the sync word:

```
[SYNC0][SYNC1][VERSION][SEQ][STATUS][YAW][PITCH][ROLL][ENCODER][CRC]
```

Updated parsers can:

- check version  
- reject unsupported versions  
- still resynchronize using sync word  

Note: parsers written for the original 11-byte unframed packet are not backward compatible with this v1 framed format and must be updated.

---

# 📦 **Final Robust Packet Layout (v1)**

```
Byte 0   : 0xAA          (SYNC0)
Byte 1   : 0x55          (SYNC1)
Byte 2   : 0x01          (VERSION)
Bytes 3–4: seq           (uint16, little-endian)
Byte 5   : status_mask   (uint8)
Bytes 6–7: yaw           (int16, deg ×100)
Bytes 8–9: pitch         (int16, deg ×100)
Bytes 10–11: roll        (int16, deg ×100)
Bytes 12–15: encoder     (int32, ticks)
Byte 16  : crc8          (CRC of bytes 2–15)
-------------------------------------------
Total: 17 bytes
```

### Overhead cost  
Original: **11 bytes**  
New: **17 bytes**  
Increase: **+6 bytes** (sync + version + seq + CRC)

At 100 Hz:  
```
17 bytes × 100 = 1700 bytes/sec
```

Still trivial for 115200 baud.

---

# 🧠 Summary of Improvements

| Issue | Fix | Cost | Benefit |
|-------|------|-------|----------|
| No framing | Sync word + CRC | +3 bytes | Resync + corruption detection |
| Underspecified fields | Units, scale, endian defined | 0 | Interoperability |
| No sequence/timestamp | 16‑bit seq | +2 bytes | Drop detection + timing |
| Versioning conflict | Version byte after sync | +1 byte | Forward compatibility |

This transforms your protocol from a “raw struct dump” into a **real telemetry protocol** suitable for logging, analysis, and long‑term evolution.

---

If you want, I can now generate:

- the **Arduino C struct + serialization code**  
- the **Mac‑side parser** (Python, Swift, C++, Rust)  
- a **sequence diagram** of the new framed packet flow  
- a **test harness** to validate sync/CRC behavior  

Just tell me which one you want next.