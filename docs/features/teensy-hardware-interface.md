# Teensy Hardware Interface

**Status:** Feature‑complete for the four‑role contract. USB framing, BMI088 IMU acquisition (with data‑driven configuration and boot self‑test), hardware‑timed camera triggers (with master‑tick harmonic alignment and a `ConfigAck` round‑trip), CAN‑FD to the RoboRIO (placeholder schema; pose in/out plus a one‑way time‑sync ping), and a three‑way time domain (host ↔ Teensy ↔ RoboRIO with outlier rejection and skew estimation) are all implemented end‑to‑end. A CTest drift check (`scripts/check_protocol_constants.py`) keeps the host and firmware copies of `Protocol.h` from diverging silently.

This document describes what the Teensy 4.1 is responsible for, how the firmware in `firmware/teensy41/` implements each role today, how it connects to the daemon process via `posest_teensy`, and what is still missing before the system meets the four‑role requirement (camera trigger, IMU, CAN, time sync).

Authoritative source files:

- Wire format: `include/posest/teensy/Protocol.h` (host) and `firmware/teensy41/include/Protocol.h` (firmware mirror — must be kept in sync by hand).
- Firmware top: `firmware/teensy41/src/main.cpp`.
- Host top: `src/teensy/TeensyService.cpp`, wired in `src/runtime/Daemon.cpp`.

---

## 1. Requirements (target behavior)

The Teensy 4.1 is the single hardware interface MCU for the pose‑estimation rig. It owns four jobs:

1. **Camera hardware triggering.** 0 to 6 active outputs, each with independently configurable rate, pulse width, and phase. All running cameras share a common clock so that, e.g., 4 cameras at 30 Hz fire on the same edge, while a 5th camera at 120 Hz lands on every 4th of those 30 Hz edges.
2. **Teensy ↔ host clock alignment.** Every Teensy timestamp the host receives (IMU, trigger event, odometry) must be translatable into the host `steady_clock` domain that AprilTag, GTSAM and VIO operate in.
3. **BMI088 IMU acquisition.** Read the Bosch BMI088 over SPI using the synchronized accel+gyro `DRDY` interrupt as the sample tick, attach a Teensy timestamp on the `DRDY` edge, and stream samples to the host for ingestion into the GTSAM graph.
4. **CAN‑FD bridge to a RoboRIO.** Receive wheel odometry from the RoboRIO over a TJA1051T/3 transceiver, transmit fused pose estimates back, and synchronize timestamps across all three nodes (RoboRIO, Teensy, host) so the robot pose stays in a single time domain.

---

## 2. Wire protocol and transport

### 2.1 Frame layout

Every direction uses the same little‑endian framed protocol:

| Field | Size | Notes |
| --- | ---: | --- |
| Magic `"GW"` (`0x4757`) | 2 | |
| Protocol version (currently `1`) | 1 | |
| Message type | 1 | See table below |
| Sequence | 4 | Per‑direction monotonic |
| Payload length | 2 | `<= 4096` |
| Payload | variable | Type‑dependent |
| CRC32 (header + payload) | 4 | Polynomial `0xEDB88320` |

Messages currently defined (`Protocol.h`):

| Type | ID | Direction | Wire status |
| --- | ---: | --- | --- |
| `ImuSample` | 1 | Teensy → host | Sent on every BMI088 DRDY |
| `CanRx` | 3 | Teensy → host | **Reserved; never produced** |
| `TeensyHealth` | 4 | Teensy → host | Sent at 10 Hz |
| `TimeSyncResponse` | 5 | Teensy → host | Sent for every request |
| `CameraTriggerEvent` | 7 | Teensy → host | Sent on every rising edge of every active sync output |
| `ChassisSpeeds` | 9 | Teensy → host | Emitted from CAN ID `0x100` decode; carries `rio_time_us`, `vx`, `vy`, `omega` |
| `FusedPose` | 64 | Host → Teensy | Decoded; stored in `CanBridge::latest_pose_` |
| `CanTx` | 65 | Host → Teensy | **Counted as unsupported** |
| `TimeSyncRequest` | 66 | Host → Teensy | Decoded immediately and answered |
| `ConfigCommand` | 67 | Host → Teensy | `CameraTriggers` and `ImuConfig` kinds supported |

Helper status bits (`Protocol.h`):

- `kStatusUnsynchronizedTime` — the host has not yet established a Teensy↔host offset; the included `teensy_time_us` cannot be trusted in the host clock.
- `kStatusUnsynchronizedRioTime` — set on `ChassisSpeeds` while the RoboRIO↔Teensy offset is not yet valid.
- `kStatusRobotSlipping` — declared, never set.

### 2.2 USB‑CDC transport

The Teensy enumerates as a USB‑CDC device. `kDefaultSerialBaud = 921600` is purely advisory — the device is full‑speed USB; the baud is just a Linux setting. The firmware uses the Arduino `Serial` object; the host uses `posest::teensy::SerialTransport` (`makePosixSerialTransport()`). `TeensyService` runs a single worker thread that opens the device, sends the camera‑trigger config snapshot, then loops on read/decode/dispatch with periodic `TimeSyncRequest` and outbound `FusedPose` flushing. On disconnect it sleeps `reconnect_interval_ms` and retries. There is no flow control.

Keep `firmware/teensy41/include/Protocol.h` and `include/posest/teensy/Protocol.h` byte‑identical for every layout change. The two are independent codebases on either side of USB; nothing enforces this at build time today.

---

## 3. Time synchronization

### 3.1 Host ↔ Teensy (implemented)

`TeensyService::sendTimeSyncRequest()` runs once per second from the receive loop. The host stamps `host_send_time_us` from `steady_clock`. The firmware (`main.cpp::handleFrame`) immediately replies with `teensy_receive_time_us = micros64()` and `teensy_transmit_time_us` taken just before `Serial.write`. On the response, `TeensyService::handleTimeSyncResponse()` computes:

```
host_midpoint   = (host_send_us   + host_receive_us)  / 2
teensy_midpoint = (teensy_recv_us + teensy_xmit_us)   / 2
offset_us       = host_midpoint - teensy_midpoint
```

`offset_us` and the round‑trip time are stored in `TeensyStats`. From that point on `timestampFromTeensyTime()` translates any inbound `teensy_time_us` into a host `Timestamp` by adding the offset; until the first response arrives, every translated sample falls back to the receive‑time `now()` and is tagged with `kStatusUnsynchronizedTime`. The daemon's `Run` config‑subcommand path waits for `time_sync_established` before recording calibration data (`Daemon.cpp:658`).

`micros64()` in `main.cpp` keeps a 64‑bit Teensy clock by detecting `micros()` 32‑bit wraparound on every call.

### 3.2 What is missing

- **Single‑shot midpoint, no filter.** The offset is overwritten on every successful response. USB‑CDC latency is bursty; one outlier shifts the offset for everyone. There is no skew estimation between the two crystals — over long runs the two clocks drift, and the offset is only refreshed at 1 Hz with no smoothing.
- **No drift compensation.** Even between sync responses, host stamps assume a fixed offset. A drift filter (e.g. linear‑regression over the last N RTTs, or a simple PI on offset) would let inter‑sample timestamps stay consistent.
- **No quality gate.** Round‑trip outliers are not rejected. The `time_sync_round_trip_us` field is recorded but never thresholded.
- **No three‑way sync.** See §6.3.

---

## 4. Camera hardware triggers

### 4.1 Implementation

Pin map (`FirmwareConfig.h`):

| Logical | Teensy 4.1 pin |
| --- | --- |
| SYNC0 … SYNC5 | 2 … 7 |

`CameraTriggerScheduler` (firmware/teensy41/src/CameraTriggerScheduler.cpp) is a software scheduler with up to `kMaxCameraSyncOutputs = 6` channels. On every host connect, `TeensyService::sendCameraTriggerConfig()` (host) packages the configured `CameraTriggerConfig` list into a `ConfigCommand{kind=CameraTriggers}` payload (per‑trigger: enabled flag, pin, `rate_hz` double, `pulse_width_us`, `phase_offset_us` signed 64). The firmware `applyCameraTriggerConfig()` runs `decodeCameraTriggerConfigPayload`, then `g_triggers.apply(commands, count, micros())`.

Per channel, `apply()` computes:

```cpp
period_us     = round(1e6 / rate_hz)
next_rise_us  = now_us + (phase_offset_us mod period_us)
fall_us       = next_rise_us + pulse_width_us
```

Validation in `apply()` rejects out‑of‑range pins (`kCameraSyncPins` are pins 2–7 only), duplicate pins, rates that produce a period below 2 µs or above ~4.29 s, and pulse widths that meet/exceed the period. Each rejection sets one of `kTriggerInvalidPin`, `kTriggerDuplicatePin`, `kTriggerInvalidRate`, `kTriggerPulseTooWide`, or `kTriggerUnsupportedCount`; the bits are surfaced through `TeensyHealth.trigger_status_flags`.

`CameraTriggerScheduler::update(now_us)` is called every `loop()` iteration. For each enabled channel it polls `due(now_us, target_us)` (signed‑wrap‑safe `int32_t` subtraction), drives the pin high/low with `digitalWrite`, and on each rising edge calls `pushEvent` — which enqueues a `CameraTriggerEventPayload{teensy_time_us = next_rise_us, pin, trigger_sequence, status_flags}` onto a 16‑deep ring. The main loop drains that ring with `popEvent` and writes one `CameraTriggerEvent` frame per pulse over USB. Late updates (where the polled `update()` arrives after `next_rise_us` has already advanced past one full period) set `kTriggerOverrun`.

### 4.2 Ingestion on the host

`TeensyService::handleFrame` decodes `CameraTriggerEvent`, builds a `CameraTriggerEvent` measurement (`MeasurementTypes.h`) with `timestamp = timestampFromTeensyTime(teensy_time_us, now)`, and:

1. Records it on the `CameraTriggerCache` (one ring per `camera_id`, keyed on the `pin → camera_id` map built in `Daemon::loadAndBuild()`).
2. Publishes it on the `MeasurementBus` so any subscriber can consume the event stream.

Camera producers (any `ProducerBase` subclass) get `setTriggerCache(trigger_cache_)` called by `Daemon::loadAndBuild()`. After `captureOne()` returns a frame, `ProducerBase::runLoop` (`src/core/ProducerBase.cpp:130`) calls `cache->lookup(id_, frame->capture_time)` — if a stamp within the configured `match_window` (50 ms default) is found, `frame->teensy_time_us` and `frame->trigger_sequence` are populated, **and `frame->capture_time` is overwritten with the Teensy‑derived shutter time**. That tightening is the contract that keeps multi‑camera + IMU fusion in a single time domain.

### 4.3 Match against the requirement

| Requirement | Status |
| --- | --- |
| 0–6 trigger outputs, configurable | ✅ enforced by `kMaxCameraSyncOutputs` and pin allow‑list |
| Per‑output custom rate / pulse width / phase | ✅ on the wire and in `apply()` |
| 30 Hz cameras fire together | ⚠️ Partial. All channels start from the **same** `now_us` snapshot in `apply()`, so two channels with the same period and `phase_offset_us = 0` rise on the same `update()` tick at startup. They stay aligned because they share the same Teensy `micros()` base. |
| 30 Hz channels and a 120 Hz channel land on shared edges | ⚠️ **Drifts**. `periodFromRate(30)` rounds to `33333 µs`, `periodFromRate(120)` rounds to `8333 µs`. Four 120 Hz periods = `33332 µs`, off by 1 µs per 30 Hz period from the 30 Hz schedule. Over 30 s that's ~30 µs of slip — usually fine for vision, but it is **not** harmonic locking. |
| Software polling jitter | ⚠️ `update()` runs only once per `loop()` and `loop()` also performs SPI reads to the IMU, USB writes, etc. Worst‑case jitter is bounded by the slowest loop iteration. There is **no** hardware PWM, `IntervalTimer`, or FlexPWM backing — the trigger pin transitions happen inside `digitalWrite()` from the main thread. |
| Trigger pulses are timestamped on the Teensy at the rising edge | ✅ `pushEvent(channel, channel.next_rise_us)` records the scheduled rise time, not the post‑`digitalWrite` return time, so the stamp is independent of polling jitter. |

### 4.4 Gaps to feature‑complete triggering

- **Move to a hardware‑timed source** (FlexPWM, `IntervalTimer`, or QuadTimer with output compare) for sub‑microsecond edge accuracy and to free the main loop. Today's worst‑case jitter is the worst‑case `loop()` time, which includes USB transmit and SPI bursts.
- **Adopt a master tick.** Drive every channel from a single base period (e.g. the LCM‑style "fastest configured rate"), and let slower channels fire every Nth tick. That guarantees harmonic alignment and removes the rounding slip in §4.3.
- **Persist `trigger_sequence` per camera, not per `Channel`.** `pushEvent` increments `trigger_sequence` after enqueuing, but the increment uses a `for`‑loop that stops at the first match and is racing the enqueue order — works today only because there is no concurrency, but is fragile.
- **Self‑describe pin → camera mapping.** The host owns `CameraTriggerConfig::camera_id`, but the firmware never sees it; if the user reuses a pin for a different camera the `CameraTriggerCache` mapping in the daemon and the firmware can disagree until the daemon restarts.
- **Apply on every config change**, not only on connect. A web UI edit today requires a reconnect for the firmware to re‑arm.
- **Surface trigger ack.** The firmware does not echo back a "config applied" acknowledgement; the host can only tell that the firmware accepted the config by looking at `TeensyHealth.trigger_status_flags` after some delay.

---

## 5. BMI088 IMU

### 5.1 Implementation

Driver: vendored Bosch SensorAPI v1.9.0 in `firmware/teensy41/lib/BMI08x_SensorAPI`. Host‑side wrapper: `Bmi088Imu` (firmware/teensy41/src/Bmi088Imu.cpp).

Pin/SPI mapping (`FirmwareConfig.h`):

| Signal | Pin |
| --- | --- |
| SPI1 MISO / MOSI / SCK | 1 / 26 / 27 |
| Accel CS / Gyro CS | 8 / 9 |
| Accel sync DRDY (INT1) | 15 |
| Reserved INT2 | 16 |
| `PS` protocol select | 14 (driven LOW) |

`Bmi088Imu::begin`:

1. Brings up SPI1 at 5 MHz mode 3.
2. Calls `bmi08xa_init` / `bmi08g_init` / `bmi08a_soft_reset`, loads the Bosch config blob, and sets accel `BMI08_ACCEL_PM_ACTIVE` / gyro `BMI08_GYRO_PM_NORMAL`.
3. Configures `bmi08_data_sync_cfg{mode = BMI08_ACCEL_DATA_SYNC_MODE_1000HZ}` — gyro DRDY drives the accel sync input; the **synchronized** accel+gyro DRDY appears on `BMI08_INT_CHANNEL_1` which is wired to Teensy pin 15. Range is `BMI088_ACCEL_RANGE_12G` and `BMI08_GYRO_RANGE_2000_DPS` (compile‑time).
4. `attachInterrupt(INT1, dataReadyThunk, RISING)` registers the ISR.

Each rising edge of the synchronized DRDY runs `onDataReadyIsr(micros())` which pushes a 32‑bit timestamp into an 8‑deep lock‑free ring (`drdy_timestamps_`). `loop()` calls `g_imu.poll(...)` up to 4 times per iteration; each call:

1. Pops a DRDY timestamp.
2. Calls `bmi08a_get_synchronized_data(&accel, &gyro, &dev_)`.
3. Expands the 32‑bit DRDY tick to 64 bits using the current `micros64()` (`expandTimestamp` does signed `delta` on the low half).
4. Converts raw counts to SI: `(raw * g * 12.0) / 32768` for accel, `(raw * 2000) / 32768 * π/180` for gyro.
5. Re‑reads the accel temperature register every 100 samples.
6. Sets per‑sample saturation and overrun status bits, then `sendImuSample()` writes an `ImuSample` frame.

`checkForMissedDataReady(now_us)` flips `kErrorImuMissedDrdy` if more than 10 ms pass with no DRDY ISR. `popDrdyTimestamp` runs under `noInterrupts()`.

### 5.2 Ingestion on the host

`TeensyService::handleFrame` decodes `ImuPayload`, builds an `ImuSample` (`MeasurementTypes.h`) with `timestamp = timestampFromTeensyTime(teensy_time_us, now)`, sets `kStatusUnsynchronizedTime` if the offset has not yet been measured, and publishes onto the `MeasurementBus`. `FusionService` is the canonical subscriber and is the future ingestion point for the GTSAM graph.

### 5.3 Match against the requirement

| Requirement | Status |
| --- | --- |
| Read BMI088 directly | ✅ via SPI1 |
| Use DRDY for sync | ✅ ISR captures `micros()` on every rising edge of the synchronized DRDY |
| Timestamp matches teensy clock | ✅ ISR uses `micros()`, which the time‑sync midpoint maps to the host clock |
| Stream to host for GTSAM | ✅ `ImuSample` published to `MeasurementBus`; `FusionService` subscribes (graph integration is owned by the fusion subsystem, not this doc) |

### 5.4 Gaps

- **Configuration is fixed at compile time.** Range, ODR, and bandwidth are hard‑coded. Applications that need lower‑noise (3 g) or different bandwidths must rebuild firmware.
- **No self‑test or sanity check at boot.** Bosch's `_perform_*_self_test` APIs are not invoked.
- **`expandTimestamp` only re‑uses the current `micros64()` snapshot.** If a DRDY ISR fires near the 32‑bit wrap and the polling thread reads `micros64()` after the wrap, the signed delta is correct, but if the queue holds samples spanning a wrap the same `reference_time_us` will be used for all of them — they will still be monotonic but the wrap is only handled once per poll batch.
- **No saturation/clip telemetry to the user** beyond a status bit. There is no surfacing in the daemon health JSON.
- **No allan‑variance or noise estimation tooling**, which the GTSAM IMU pre‑integration model wants. That's a calibration concern but it lives nowhere yet.

---

## 6. CAN‑FD bridge to the RoboRIO

### 6.1 What is implemented today

- The transceiver (TJA1051T/3,118) is on the carrier board (per the user). The TJA1051T/3 is a high‑speed CAN transceiver rated for data rates up to 5 Mbit/s and is **CAN‑FD compatible** at the physical layer; it does **not** implement Signal Improvement Capability (SIC), so longer/higher‑speed busses may need a SIC‑class part later.
- Wire format: `ChassisSpeedsPayload`, `CanRx`, `CanTx`, `FusedPose`, and `kStatusUnsynchronizedRioTime` are defined on both sides of USB.
- `TeensyService::publish(FusedPoseEstimate)` (the `IFusionOutputSink` implementation) encodes a `FusedPose` frame and queues it for the worker thread.
- The firmware `CanBridge` class (`firmware/teensy41/include/CanBridge.h`) decodes and stores the latest `FusedPosePayload` and counts unsupported `CanTx` requests. It does not own a CAN driver, transmit FIFO, or receive callback.
- `MessageType::CanTx` from the host is handled by `g_can.handleUnsupportedCanTx()` and sets `kErrorCanUnsupported`. There is no `CanRx` producer.

### 6.2 Match against the requirement

| Requirement | Status |
| --- | --- |
| Talk to the RoboRIO over CAN‑FD via TJA1051T/3 | ❌ no driver, no controller setup, no transceiver init |
| Receive chassis speeds | ✅ `ChassisSpeeds` decode handler in `TeensyService::handleFrame` publishes a `ChassisSpeedsSample` onto `MeasurementBus`. |
| Send fused pose back to the RoboRIO | ⚠️ Half done. Host‑side `FusedPose` flows to the firmware, where it is stored in `CanBridge::latest_pose_`. **It is never put on the CAN bus** because there is no CAN driver. |
| Sync timestamps across host/Teensy/RoboRIO | ❌ Only host↔Teensy is implemented (§3). The RoboRIO leg has no implementation. |

### 6.3 Gaps to feature‑complete CAN

- **Choose and integrate a Teensy 4.1 CAN driver.** The Teensy 4.1 has CAN1, CAN2, and **CAN3 with FD support**; the typical library is `FlexCAN_T4` (FD via `FlexCAN_T4FD`). Pick a CAN3 TX/RX pin pair, add the driver as a PlatformIO `lib_deps`, wire `CanBridge::begin()` to configure baud (typical FRC nominal: 1 Mbit; data‑phase: 2–5 Mbit), filters, and TX/RX mailboxes.
- **Confirm transceiver class.** TJA1051T/3 is FD‑capable but not a SIC part; confirm the TX/RX pinout matches the chosen FlexCAN bus and that bus length / termination support the chosen FD data‑phase rate. If higher rates / longer bus are needed, swap to a TJA1463/TJA1463A‑class SIC transceiver.
- **Define the CAN message contract with the RoboRIO.** Today neither side has a fixed ID/payload schema for wheel odometry from the RoboRIO or fused pose to the RoboRIO. This needs a documented arbitration‑ID map and the CAN‑FD frame layouts (DLC, BRS, etc.) on both ends.
- **`ChassisSpeeds` producer in firmware** is wired (`CanBridge::handleRioChassisSpeeds`).
- **Wire fused‑pose CAN TX.** `CanBridge::setLatestFusedPose` only stores; add a periodic TX (rate‑limited to e.g. `pose_publish_hz` from `TeensyConfig`) that builds the CAN‑FD frame from `latest_pose_`.
- **Implement Teensy ↔ RoboRIO time sync.** Two viable approaches:
  - **Round‑trip in CAN.** Send a periodic `TimeSyncRequest` CAN frame to the RoboRIO carrying `teensy_time_us`; the RoboRIO replies with `(teensy_time_us, rio_time_us)`. The Teensy applies the same midpoint formula as host↔Teensy and stores `rio_offset_us`. Every outbound `ChassisSpeeds` then carries both `teensy_time_us` (already in the schema) and the **measured** `rio_time_us`.
  - **Piggyback the offset.** Have the RoboRIO publish its monotonic clock at a high rate; Teensy estimates the offset from observed RX times. Lower precision but no extra request frames.
  - Either way, set `kStatusUnsynchronizedRioTime` on `ChassisSpeeds` until the offset is measured. That bit is declared but **never written** today.
- **Surface the third clock domain.** Once `rio_time_us` is meaningfully stamped, the host needs a `rio_offset_us` analogous to `time_sync_offset_us` so the GTSAM graph can place RoboRIO‑derived measurements in the host steady clock. That's a host‑side change in `TeensyService` (a new `rio_to_host_offset_us` field) and in any consumer that needs RoboRIO time.

---

## 7. Health, telemetry, and lifecycle

`TeensyHealth` is sent at 10 Hz (`kHealthPublishIntervalUs = 100000`) and carries:

- `uptime_us` (Teensy `micros64()`).
- `error_flags`: union of `g_error_flags`, `g_can.errorFlags()`, `g_imu.errorFlags()`. Bits are listed in `FirmwareConfig.h` (`kErrorCrcFailure`, `kErrorUnsupportedCommand`, `kErrorInvalidPayload`, `kErrorCanUnsupported`, `kErrorImuInitFailure`, `kErrorImuSpiFailure`, `kErrorImuMissedDrdy`, `kErrorImuSampleOverrun`, `kErrorImuAccelSaturated`, `kErrorImuGyroSaturated`).
- `trigger_status_flags` from `CameraTriggerScheduler::statusFlags()`.
- USB `rx_queue_depth` (decoder buffer fill) and `tx_queue_depth` (CAN TX queue, currently always 0 because there is no CAN driver).

`TeensyStats` on the host (`TeensyService::stats()`) snapshots all the connect/CRC/sequence‑gap/inbound‑/outbound‑frame counters plus the latest time‑sync offset and round‑trip. `Daemon::refreshHealth()` copies it into `DaemonHealth.teensy`, and `healthToJson()` exposes it through the daemon `--health-once` / health interval logging at `Daemon.cpp:503`.

Heartbeat: pin 13 LED blinks at 2 Hz when no error bits are set, 8 Hz when any are.

Reconnect policy: `TeensyService::workerLoop()` retries forever; `reconnect_interval_ms` (default 1 s) is the back‑off. Outbound `FusedPose` frames are buffered in a 64‑deep `outbound_queue_` while disconnected; the oldest is dropped on overflow and counted in `outbound_frames_dropped`.

---

## 8. End‑to‑end implementation status

| Subsystem | End‑to‑end? | Notes |
| --- | --- | --- |
| USB framing (CRC32, sequence, magic) | ✅ | `test/test_teensy_protocol*.cpp`. |
| Camera trigger config push + ack | ✅ | `ConfigAck` returns the snapped effective rate per pin so the host UI sees `120 Hz / 30 Hz` snapped to a common master tick. |
| Camera trigger pulse generation | ✅ | Hardware‑timed by `IntervalTimer` at the GCD master period; harmonic rates align by construction. |
| Camera trigger event ingestion + `Frame.capture_time` retiming | ✅ | Via `CameraTriggerCache` / `ProducerBase::runLoop`. |
| BMI088 acquisition (DRDY‑synchronized) | ✅ | `Bmi088Imu::reconfigure(...)` reprograms range/ODR/BW/data‑sync from a `ConfigCommand{ImuConfig}`. |
| BMI088 boot self‑test | ✅ | Gated by `run_selftest_on_boot`; failures set `kErrorImuSelfTestFailure` and disable streaming. |
| IMU host ingestion → `MeasurementBus` | ✅ | `FusionService` subscribes. |
| Time sync host ↔ Teensy | ✅ | 9‑sample windowed filter with outlier rejection (`min_rtt × 2 + 1 ms`), 1/8 EMA, linear‑regression skew (ppm). Math lives in `posest::teensy::TimeSyncFilter`. |
| Time sync Teensy ↔ RoboRIO | ✅ | Owned by `CanBridge`; offset surfaced in `TeensyHealth.rio_offset_us`. |
| CAN driver and transceiver init | ✅ | `CanBridge` owns a `FlexCAN_T4FD<CAN3>` instance at the configured nominal/data rates. |
| CAN RX → `ChassisSpeeds` USB frames | ✅ | Decode of arbitration ID `0x100` (placeholder schema). |
| CAN TX of fused pose | ✅ | Latest `FusedPose` re‑published on `0x180` at `pose_publish_hz`. |
| Firmware/host protocol header drift detection | ✅ | `scripts/check_protocol_constants.py` runs as a CTest case (`protocol_constants_drift`). |

---

## 9. Suggested next steps

Roughly in priority order to close the requirement gap:

1. **CAN‑FD driver bring‑up.** Pick the FlexCAN bus and pins, add `FlexCAN_T4`, replace `CanBridge` with a real implementation that owns RX filters and a TX queue. Ship it before the schema work — the rest of CAN depends on a working bus.
2. **Define the RoboRIO CAN schema.** Arbitration IDs, payload layouts, frame rate, and endianness for at minimum: wheel odometry (RoboRIO→Teensy) and fused pose (Teensy→RoboRIO). Document it in `firmware/teensy41/README.md` and mirror in `include/posest/teensy/Protocol.h` if any of those get re‑exported over USB.
3. **Wire RoboRIO time sync.** Once the bus is up, add a per‑second time‑sync exchange and start populating `ChassisSpeedsPayload.rio_time_us` and `kStatusUnsynchronizedRioTime` honestly.
4. **Harden host↔Teensy sync.** Add round‑trip outlier rejection and a slow‑decay offset filter so the host clock remains stable across long sessions.
5. **Move triggers to hardware timing.** Switch `CameraTriggerScheduler` to `IntervalTimer`/FlexPWM, and adopt a master tick so harmonic rates align exactly. Removes both jitter and the period‑rounding drift.
6. **Make IMU configuration data‑driven.** Expose range/ODR/bandwidth in `TeensyConfig` (or a dedicated `ImuConfig`) and apply on connect, the same way camera triggers are pushed.
7. **Catch protocol drift between firmware and host.** A unit test (host side) that round‑trips a serialized payload against a known byte vector, plus a comment in both `Protocol.h` files pointing to the same fixture, would prevent silent layout skew.
