# GuessWork CAN protocol — controller ↔ Teensy ↔ host

Phase 5 wire contract. The robot controller (RoboRIO today, SystemCore from
2027) streams chassis speeds to the Teensy over CAN; the Teensy stamps each
sample on its wrap-extended 64-bit clock (the system-wide time base) and
forwards it to the Mac over the binary telemetry CDC. The reverse path carries
the fused field pose from the Mac through the Teensy back onto the bus.

Everything below is normative. The firmware-side encode/decode lives in
`firmware/src/can_payloads.h` (pure, host-unit-tested in
`tests/test_can_payloads.cpp`); change that header and this document together.

## Modes

| Mode | DB value | Bus | Frames |
|---|---|---|---|
| classic | `roborio` | CAN 2.0B @ 1 Mbps | 8-byte, int16 milli-units, two-frame splits |
| fd | `systemcore` | CAN FD, arbitration 1 Mbps / data 4 Mbps (BRS) | 24-byte, float32, single frames |

The mode is persisted in the `can_config` table (`off` | `roborio` |
`systemcore`), set via `PUT /api/can/config`, and pushed to the Teensy as the
ASCII command `CAN_MODE mode=off|classic|fd` (re-pushed on every reconnect).
The firmware boots with CAN off; the host is the source of truth.

Hardware: TJA1051T/3 transceiver on Teensy 4.1 CAN3 — pin 30 = CRX3, pin 31 =
CTX3 (CAN3 is the only FD-capable FlexCAN on Teensy 4.x). The TJA1051 is rated
for FD data rates up to 5 Mbps; if the 4 Mbps data phase shows errors on real
wiring, drop `kFdDataBaud` to 2 Mbps — no frame layout changes. 120 Ω
termination at both physical ends of the bus.

## Byte order

**Everything is little-endian** — CAN payloads, telemetry payloads, all of it.
Both ends are little-endian ARM. RIO-side Java must use
`ByteBuffer.order(ByteOrder.LITTLE_ENDIAN)` (Java defaults to big-endian).

## Arbitration IDs

FRC 29-bit extended IDs, per the WPILib team-use convention so the controller
side is plain WPILib `CAN`:

```
id = deviceType<<24 | manufacturer<<16 | apiId<<6 | deviceNumber
   = 10 (Miscellaneous) <<24 | 8 (TeamUse) <<16 | apiId<<6 | 33
```

`deviceNumber` is the firmware constant `kCanDeviceNumber = 33`
(`can_payloads.h`). API IDs:

| API ID | Name | Direction | Mode | Full ID (dev 33) |
|---|---|---|---|---|
| 0x110 | CHASSIS_SPEEDS | controller → Teensy | both | `0x0A084421` |
| 0x111 | CHASSIS_STAMP | controller → Teensy | classic only | `0x0A084461` |
| 0x120 | POSE | Teensy → controller | fd only | `0x0A084821` |
| 0x121 | POSE_XY | Teensy → controller | classic only | `0x0A084861` |
| 0x122 | POSE_THETA | Teensy → controller | classic only | `0x0A0848A1` |

## Uplink: chassis speeds (controller → Teensy)

Robot-frame velocities from the drivetrain's forward kinematics
(`drivetrain.getChassisSpeeds()` — drive-type-agnostic; GuessWork never sees
wheel/module math). 50–100 Hz. `status_flags`: bit 0 = stale encoder data,
bit 1 = wheel slip detected. `counter` increments by 1 per sample (wraps at
255 → 0).

### FD mode — one frame

**CHASSIS_SPEEDS (FD, DLC 24):**

| Offset | Field | Type |
|---|---|---|
| 0 | vx | f32, m/s |
| 4 | vy | f32, m/s |
| 8 | omega | f32, rad/s CCW+ |
| 12 | rio_time_us | u64, FPGA µs at wheel-speed sampling |
| 20 | status_flags | u16 |
| 22 | counter | u8 |
| 23 | reserved | u8 = 0 |

### Classic mode — two frames, STAMP first

**CHASSIS_STAMP (classic, DLC 8) — send FIRST:**

| Offset | Field | Type |
|---|---|---|
| 0 | rio_time_lo | u32, low 32 bits of FPGA µs |
| 4 | counter | u8 |
| 5 | reserved | u8[3] = 0 |

**CHASSIS_SPEEDS (classic, DLC 8) — send immediately after:**

| Offset | Field | Type |
|---|---|---|
| 0 | vx | i16, mm/s (clamped ±32767) |
| 2 | vy | i16, mm/s |
| 4 | omega | i16, mrad/s |
| 6 | status_flags | u8 (bits 0–1) |
| 7 | counter | u8 |

Pairing rule: the Teensy latches the most recent STAMP; when a SPEEDS frame
arrives it attaches the stamped time **iff the counters match**, otherwise the
forwarded `rio_time_us` is 0 (= unknown; the host falls back to the arrival
stamp). The Teensy wrap-extends `rio_time_lo` to 64 bits (FPGA µs wraps every
~71.6 min — mid-practice-session real).

## Downlink: fused pose (Teensy → controller)

Sent whenever the host pushes a POSE telemetry packet (placeholder data until
Phase 6 wires the GTSAM output). `counter` is a host-assigned rolling counter;
**controller-side staleness rule: treat the pose as stale if the counter has
not advanced for > 200 ms.** `quality` is 0–255 (semantics defined in Phase 6;
255 = bench test).

**POSE (FD, DLC 24):**

| Offset | Field | Type |
|---|---|---|
| 0 | rio_time_us | u64, pose sample time mapped into the FPGA clock by the host (0 = mapping unavailable) |
| 8 | x | f32, m (WPILib field frame) |
| 12 | y | f32, m |
| 16 | theta | f32, rad |
| 20 | quality | u8 |
| 21 | counter | u8 |
| 22 | reserved | u16 = 0 |

**POSE_XY (classic, DLC 8) — sent first:** x f32 @0, y f32 @4.

**POSE_THETA (classic, DLC 8):** theta f32 @0, quality u8 @4, counter u8 @5,
reserved u8[2] @6. The controller latches XY and publishes on THETA. Classic
pose carries no timestamp (no room) — staleness is counter-advancement only.

## Telemetry packets (Teensy ↔ host USB CDC)

Framing per `firmware/src/binary_proto.h`:
`[0xA5][0x5A][type u8][len u8][payload][crc16 u16 LE]`, CRC16-CCITT
(poly 0x1021, init 0xFFFF) over type + len + payload.

**ODOM (0x03, Teensy → host, payload 32 bytes)** — identical in both CAN
modes; the firmware converts classic milli-units to float:

| Offset | Field | Type |
|---|---|---|
| 0 | t_arrival_us | u64, Teensy clock at CAN RX ISR (SPEEDS frame) |
| 8 | rio_time_us | u64, 0 = unknown |
| 16 | vx | f32, m/s |
| 20 | vy | f32, m/s |
| 24 | omega | f32, rad/s |
| 28 | status_flags | u16 |
| 30 | counter | u8 |
| 31 | mode | u8: 0 = off, 1 = classic, 2 = fd |

**POSE (0x10, host → Teensy, payload 22 bytes)** — written by the host to the
telemetry CDC in the reverse direction:

| Offset | Field | Type |
|---|---|---|
| 0 | rio_time_us | u64 (host pre-converts via its RIO clock sync; 0 if sync unhealthy) |
| 8 | x | f32 |
| 12 | y | f32 |
| 16 | theta | f32 |
| 20 | quality | u8 |
| 21 | counter | u8 |

**HEARTBEAT (0x02) fw=3 extension** — payload grows 17 → 34 bytes. The
existing prefix is unchanged; `flags` gains bit 1 = can_ok (mode active, bus
configuration succeeded, not bus-off):

| Offset | Field | Type |
|---|---|---|
| 0 | t_us | u64 |
| 8 | flags | u8 (bit0 imu_ok, bit1 can_ok) |
| 9 | imu_samples | u32 |
| 13 | imu_drops | u32 |
| 17 | can_rx | u32, accepted chassis frames |
| 21 | can_rx_drops | u32, ISR ring overflows + counter-mismatched stamps |
| 25 | odom_tx_drops | u32, ODOM packets dropped to USB backpressure |
| 29 | pose_tx | u32, pose frames written to CAN |
| 33 | can_mode | u8: 0/1/2 as above |

A fw=2 heartbeat is `len == 17`; the host parses the extension only when
`len >= 34`.

## Time synchronisation (RIO clock ↔ Teensy clock)

The Teensy does no clock math — it forwards both the controller's
`rio_time_us` and its own arrival stamp. The host's `RioClockSync`
(`src/core/rio_clock_sync.hpp`) estimates
`teensy_us ≈ rio_us + offset(t)`:

- each pair gives `arrival − rio = offset + transit` where transit jitter is
  0.1–5 ms (controller scheduling + bus + ISR);
- per-250 ms bucket minima reject the jitter; a least-squares fit over a 6 s
  window of bucket minima tracks offset + crystal drift (≤ ~100 ppm);
- a constant floor bias of roughly one CAN frame + ISR latency (~150 µs)
  remains in the offset. Accepted: it is far below the usefulness threshold
  for 5.2 m/s odometry, and it cancels on the pose-downlink round trip
  (the inverse mapping carries the same bias);
- sync resets automatically when `rio_time_us` jumps backward (> 10 ms) or the
  offset steps by > 50 ms — controller reboot / FPGA clock set. During the
  ~1 s re-warm-up, odometry samples are stamped with the Teensy arrival time
  instead (`ChassisSpeeds.t_ns` falls back to `t_arrival_ns`).

## Controller-side sketch (out of repo scope)

WPILib Java, classic mode (RoboRIO):

```java
CAN can = new CAN(33, 8 /* kTeamUse */, 10 /* kMiscellaneous */);
// 100 Hz loop:
ChassisSpeeds sp = drivetrain.getChassisSpeeds();
long fpgaUs = RobotController.getFPGATime();
counter = (counter + 1) & 0xFF;

ByteBuffer stamp = ByteBuffer.allocate(8).order(ByteOrder.LITTLE_ENDIAN);
stamp.putInt((int) fpgaUs).put((byte) counter).put(new byte[3]);
can.writePacket(stamp.array(), 0x111);          // STAMP first

ByteBuffer speeds = ByteBuffer.allocate(8).order(ByteOrder.LITTLE_ENDIAN);
speeds.putShort((short) clamp(sp.vxMetersPerSecond * 1000));
speeds.putShort((short) clamp(sp.vyMetersPerSecond * 1000));
speeds.putShort((short) clamp(sp.omegaRadiansPerSecond * 1000));
speeds.put((byte) statusFlags).put((byte) counter);
can.writePacket(speeds.array(), 0x110);

// Pose RX: poll 0x121 then 0x122 with can.readPacketNew(...), latch XY,
// publish on THETA, and mark stale if the counter stops advancing for 200 ms.
```

SystemCore (FD) replaces the two 8-byte packets with the single 24-byte
layouts above, float32 throughout.
