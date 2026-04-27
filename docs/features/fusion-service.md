# Fusion Service (GTSAM)

**Status:** Functionally complete for the in-scope requirements (§1). `FusionService` consumes `ChassisSpeedsSample`, `AprilTagObservation`, `VioMeasurement` (gated, placeholder-aware), and `ImuSample` from the `MeasurementBus`. The iSAM2 graph carries `Pose3` plus optional `Vector3` velocity keys (when `enable_imu_preintegration` is on); `BetweenFactor<Pose3>` for chassis odometry is wrapped in a Huber kernel for slip robustness; vision arrives as `PriorFactor<Pose3>` with the AprilTag pipeline's covariance; a soft floor-constraint `PriorFactor<Pose3>` pins z/roll/pitch toward zero on every new pose key (the always-grounded platform invariant); `gtsam::ImuFactor` carries body-frame IMU between keyframes once a boot stationary calibration has anchored bias and the first vision observation has fixed the field frame. Wheel slip is handled four ways: Huber on every chassis step, RIO-driven `kFusionStatusSlipReported`, IMU shock/free-fall sigma inflation, and (Phase C only) chassis-vs-IMU velocity disagreement → `kFusionStatusSlipDetected` with 2-step hysteresis. A planar-speed gate drops chassis samples whose `√(vx² + vy²)` exceeds `max_chassis_speed_mps` (default 6.5 m/s ≈ platform 5.2 m/s × 1.25 margin) and surfaces them as `kFusionStatusDegradedInput`. The published `FusedPoseEstimate` flows through `IFusionOutputSink → TeensyService` over a v4 USB protocol (`host_send_time_us` prefix + Pose3 + velocity + 6×6 covariance + status, 376 B); the Teensy `CanBridge` repacks that into a 64-byte CAN-FD frame on `teensy_pose_can_id` at the configured rate and reports a rolling decode-to-CAN-TX latency window in `TeensyHealthPayload`. Every fusion tunable persists through `runtime::FusionConfig` and the `fusion_config` SQLite table (migration 10), validated by `ConfigValidator`. Live config reload is wired: `WebService::stageConfig` invokes a daemon-installed callback that hands the new config to `FusionService::applyConfig`; the worker swaps it at the top of the next `process()` iteration. Live-reloadable fields update in place; structural fields (Phase C toggles, IMU noise sigmas, IMU extrinsic, persisted bias, floor-constraint enable) are rejected on the live path and counted in `config_reloads_structural_skipped`. On clean daemon shutdown, a freshly-calibrated IMU bias is written back to `RuntimeConfig.fusion.persisted_bias` so the next boot starts from the last good seed. Per-stage latency histograms and graph-size counters surface through `DaemonHealth` and `posest_daemon --health-once` JSON, alongside the new Phase C / F-1 / F-3 / F-4 / F-6 counters. **Three items remain deferred:** the field-tag layout is not yet used as landmark factors (gap 8.4 — defer until single-camera flips become a measured problem), `VioMeasurement` flows through a placeholder-aware path that becomes load-bearing only when the real VIO frontend lands (gap 8.9), and Phase C IMU preintegration ships behind `enable_imu_preintegration = false` by default — flip it on after the on-robot validation described in §8.

This document covers the GTSAM-based sensor fusion service end-to-end: what it is, how it integrates with the rest of the runtime, how the graph is constructed, and what's missing before it meets the high-performance-FRC bar laid out in §1. Generic plumbing (`MeasurementBus`, the non-blocking-publisher rule) lives in [`producer-consumer-architecture.md`](producer-consumer-architecture.md); the AprilTag-side covariance and aggregation that feed this service live in [`apriltag-pipeline.md`](apriltag-pipeline.md); the Teensy USB framing and CAN bridge live in [`teensy-hardware-interface.md`](teensy-hardware-interface.md).

---

## 1. Requirements (target behavior)

The bar this document is reviewing against:

1. **Inputs:** all AprilTag-pipeline outputs (one `AprilTagObservation` per camera per frame), Teensy-mediated IMU samples (`ImuSample`, ~1 kHz), and Teensy-mediated chassis kinematics from the RoboRIO (`ChassisSpeedsSample`, ~100 Hz).
2. **VIO is deferred.** A `VioMeasurement` type already exists; the fusion service must remain easy to extend when VIO lands, but is not required to consume it today.
3. **Platform reality:** the robot is high-performance FRC, max ~5.2 m/s, always grounded (floor-constrained), and routinely takes hard impacts (defense, ramps, bumpers) and experiences wheel slip on aggressive direction changes.
4. **Configurable from the website.** Every fusion knob (process noise, prior sigmas, shock thresholds, IMU window, chassis-gap timeout, floor sigmas, max chassis speed, slip-disagreement threshold) must be data — JSON the future HTTP server can read and write — persisted in SQLite, and reloaded into the running graph when changed. The web server is not yet built, but the live-reload path is wired end-to-end: `WebService::stageConfig` invokes a daemon-installed callback that calls `FusionService::applyConfig`. Live-reloadable knobs take effect on the worker's next iteration; structural knobs (Phase C toggles, IMU noise sigmas, IMU extrinsic, persisted bias, floor-constraint enable) are silently rejected on the live path and require a daemon restart — see §6.1.
5. **Use the AprilTag field layout** if it tightens the graph. The layout is already loaded into `RuntimeConfig::field_layouts` for the vision pipeline; if landmark factors or per-tag constraints would help fusion, the same data should flow into the graph.
6. **Output goes to the RoboRIO over CAN, via the Teensy.** Fusion publishes a `FusedPoseEstimate`; `TeensyService` encodes it into a USB `FusedPose` frame; the Teensy's `CanBridge` re-broadcasts it on the configured CAN ID at the configured rate. No path to the RIO except through the Teensy.
7. **Latency.** This is a real-time pose source for a 5.2 m/s robot. End-to-end latency from `ChassisSpeedsSample` arrival → CAN frame on the bus must stay tight. The non-blocking-publisher rule from [`producer-consumer-architecture.md`](producer-consumer-architecture.md#23-the-latency-contract) applies: nothing on the publish path may block another thread. Host-side stages (`graph_update_us`, `publish_us`) and firmware-side decode-to-CAN-TX latency (rolling min/avg/max in `TeensyHealthPayload`, surfaced through `DaemonHealth.teensy.fused_pose_*`) are measured and surfaced through `posest_daemon --health-once`.

---

## 2. Architecture

### 2.1 Where it sits in the runtime graph

```
┌─ AprilTag pipelines (N) ─┐
│ AprilTagObservation       │──┐
└───────────────────────────┘  │
                               │
┌─ TeensyService (RX) ──────┐  │            ┌─ FusionService (worker thread) ───┐
│ ImuSample (1 kHz)          │──┼──────────▶│ MeasurementBus.take()              │
│ ChassisSpeedsSample (100Hz)│  │           │  ├─ IMU      → shock window only   │
└───────────────────────────┘  │           │  ├─ Chassis  → BetweenFactor (Δt)  │
                               │           │  └─ AprilTag → PriorFactor          │
                               │           │ iSAM2.update()                       │
                               │           │ marginalCovariance(current_key)      │
                               │           │ publishEstimate(FusedPoseEstimate)   │
                               │           └────────────┬─────────────────────────┘
                               │                        │ IFusionOutputSink::publish
                               │                        ▼
                               │           ┌─ TeensyService (TX, sink) ──────────┐
                               │           │ encodeFusedPosePayload → USB Frame  │
                               │           │ MessageType::FusedPose (id 64)      │
                               │           └────────────┬─────────────────────────┘
                               │                        │ USB serial
                               │                        ▼
                               │           ┌─ Teensy 4.1 firmware ───────────────┐
                               │           │ decodeFusedPosePayload              │
                               │           │ CanBridge.setLatestFusedPose        │
                               │           │ maybeSendFusedPose @ pose_publish_hz│
                               │           │   → CAN-FD on teensy_pose_can_id    │
                               │           └────────────┬─────────────────────────┘
                               │                        │ CAN-FD
                               │                        ▼
                               │                    RoboRIO
                               │
(MeasurementBus is the queue both sides share; FusionService is the only consumer.)
```

| Component | File | Role |
|-----------|------|------|
| `FusionService` | `include/posest/fusion/FusionService.h`, `src/fusion/FusionService.cpp` | The service. Owns the worker thread, the `FusionBackend`, the sink list, and the stats. Pulls measurements off the bus, dispatches by type, publishes estimates to all sinks. |
| `FusionBackend` (private) | `src/fusion/FusionService.cpp:77` | The graph itself. Holds the `gtsam::ISAM2`, the running `gtsam::Values`, the IMU shock window, and the bootstrap state machine. |
| `fusion::FusionConfig` | `include/posest/fusion/FusionService.h` | In-process tuning struct. Built from the SQLite-loaded `runtime::FusionConfig` by `buildFusionConfig`. |
| `runtime::FusionConfig` | `include/posest/runtime/RuntimeConfig.h` | Persisted singleton; mirrors the in-process struct field-for-field. See §6. |
| `IFusionOutputSink` | `include/posest/fusion/IFusionOutputSink.h` | One-method abstract sink. Implemented by `TeensyService`. Multiple sinks supported (e.g. for future telemetry/logging). |
| `FusedPoseEstimate` | `include/posest/MeasurementTypes.h` | The published payload: `timestamp`, `Pose2d field_to_robot`, optional `velocity`, 6×6 `covariance` (gtsam tangent order), `status_flags`. The wire encoder builds a wider Pose3 + velocity + covariance payload from this — see §7. |
| `MeasurementBus` | `include/posest/MeasurementBus.h` | Bounded MPSC queue (capacity 4096 in `Daemon.cpp`) of `Measurement = std::variant<...>`. Drop-newest on overflow. |
| `TeensyService` | `src/teensy/TeensyService.cpp` | Implements `IFusionOutputSink::publish`: encodes the 376-byte v4 USB payload (with `host_send_time_us` prefix) and enqueues a `MessageType::FusedPose` frame. |
| `CanBridge` | `firmware/teensy41/src/CanBridge.cpp` | Periodic CAN-FD TX of the most recent host-supplied pose at `pose_publish_hz`. Repacks Pose3 + velocity + sigma triple into the 64-byte frame in §7. Maintains a rolling decode-to-CAN-TX latency window (F-6) reset on every `TeensyHealth` emit. |
| `LatencyHistogram` | `include/posest/util/LatencyHistogram.h` | Rolling per-stage timing buffer. Two instances live in the fusion service (graph-update and publish stages); snapshots flow into `DaemonHealth` and the `posest_daemon --health-once` JSON. |

### 2.2 The latency contract for fusion specifically

The fusion service inherits the [non-blocking publisher rule](producer-consumer-architecture.md#23-the-latency-contract) from the bus side and re-applies it on the sink side:

- **Publishers** (`AprilTagPipeline`, `TeensyService` RX path) call `MeasurementBus::publish()`, which is non-blocking and drops the newest sample on overflow. Drops are counted via `MeasurementBus::droppedNewestCount()` and surfaced through `DaemonHealth`.
- **The fusion worker** is single-threaded by design: GTSAM's `ISAM2` is not safe to call from multiple threads, so all graph mutation happens on one consumer pulling off the bus.
- **Sink invocation** is direct, synchronous, and outside the service mutex (`FusionService.cpp:404-413`). `TeensyService::publish` only enqueues for the serial worker thread, so it returns in microseconds — safe to call inline.

The end-to-end critical path for a chassis-speed update is:

```
RoboRIO CAN frame
  → Teensy CanBridge RX
  → Teensy USB TX
  → host TeensyService decode
  → MeasurementBus::publish              (non-blocking, ~µs)
  → FusionService::process               (worker thread)
  → ISAM2::update + calculateEstimate    (the budget item)
  → marginalCovariance                   (the other budget item)
  → TeensyService::publish (sink)        (enqueue only)
  → host USB TX
  → Teensy CanBridge::maybeSendFusedPose (rate-limited at pose_publish_hz)
  → CAN-FD frame on teensy_pose_can_id
```

The two host-side stages of this path (`bus-pop → ISAM2.update return` and `update done → all sinks returned`) are sampled into rolling `LatencyHistogram` buffers and surfaced through `FusionStats.graph_update_us` / `publish_us`. The firmware-side decode-to-CAN-TX slice is also measured (F-6, v4): `decodeFusedPosePayload` stamps `decode_time_us = micros()` into `CanBridge::setLatestFusedPose`, and `transmitTeensyPose` records `now_us − decode_time_us` into a rolling window reset on every `TeensyHealth` emit. The window's `min_us` / `avg_us` / `max_us` / `samples` flow up via `TeensyHealthPayload` → `TeensyStats` → `DaemonHealth.teensy.fused_pose_*`. The host→Teensy USB transport slice (using the v4 `host_send_time_us` prefix and the time-sync offset) is the one piece still uninstrumented — see §8.

### 2.3 Threading and lifecycle

- `FusionService::start()` (`FusionService.cpp:316`) atomically transitions `running_ = true` and spawns one `worker_` thread running `runLoop()`.
- `runLoop()` (`:345`) loops on `measurement_bus_.take()` (a blocking pop with shutdown semantics) and dispatches each measurement through `process()`.
- `FusionService::stop()` (`:324`) flips `running_` to false, calls `measurement_bus_.shutdown()` to unblock the `take()`, and joins the worker.
- The destructor calls `stop()`, so a leaked instance still exits cleanly — but the daemon stops it explicitly in the documented order.

The Daemon's lifecycle (`src/runtime/Daemon.cpp`) wires this in:

1. **Build** (Daemon.cpp:818-826): construct `fusion_` with the bus + `buildFusionConfig(config_)`, construct `teensy_`, then `fusion_->addOutputSink(teensy_)`. The sink list is captured before `start()` so the very first published estimate already reaches the Teensy.
2. **Start** (Daemon.cpp:861-863): start order is `teensy_ → fusion_ → graph` (vision pipelines). Teensy first so the sink can drain; fusion second so the graph has a consumer ready before vision/odometry start emitting.
3. **Stop** (Daemon.cpp:904): reverse order — `graph → teensy → fusion` — so vision stops publishing before the bus is shut down by `fusion_->stop()`.

### 2.4 Sink fan-out

`IFusionOutputSink` is a single-method virtual:

```cpp
virtual void publish(FusedPoseEstimate estimate) = 0;
```

Sinks are stored in `std::vector<std::shared_ptr<IFusionOutputSink>>` and copied under the mutex before iteration (`FusionService.cpp:404-413`) so a slow sink can't block list mutation. Today there is exactly one sink registered (`TeensyService`); future additions (web telemetry, log file, replay) plug in with no graph changes.

---

## 3. The graph

### 3.1 Variables

The graph carries `gtsam::Pose3` keyed by `gtsam::Symbol('x', index)`, plus — when `enable_imu_preintegration` is on and the init state machine has reached `kRunning` — a parallel `Vector3` velocity series keyed by `gtsam::Symbol('v', index)`. A vision update reuses the current pose key (it tightens the existing pose rather than introducing a new variable); chassis updates advance the index per accepted sample (legacy mode, flag off) or per keyframe (Phase C, flag on). There is **no** bias variable in the graph: bias is a constant injected into each `gtsam::ImuFactor` and re-estimated only at boot during the stationary calibration window. The design memo behind this choice is the 8.2 entry in the implementation plan.

### 3.2 Factors

**`PriorFactor<Pose3>` for bootstrap.** On the first chassis sample, the backend anchors `x_0` at identity with very loose sigmas (`origin_prior_sigmas`, default `{10 rad, 10 rad, π rad, 10 m, 10 m, 10 m}`). The first `AprilTagObservation` then snaps the pose into the field frame.

**`PriorFactor<Pose3>` for vision.** Each `AprilTagObservation` with a non-empty `field_to_robot` becomes a prior on the **current** key. The 6×6 covariance comes directly from the AprilTag pipeline (see [`apriltag-pipeline.md`](apriltag-pipeline.md)) and is converted with `gtsam::noiseModel::Gaussian::Covariance`. If GTSAM rejects the matrix as not positive-definite, the factor is dropped and the estimate is published with `kFusionStatusOptimizerError` set.

**`BetweenFactor<Pose3>` for chassis odometry, Huber-wrapped.** Each accepted `ChassisSpeedsSample` produces a relative-motion factor `Pose3(Rot3::Rz(ω·Δt), Point3(vx·Δt, vy·Δt, 0))` — yaw-only rotation, planar translation. The diagonal `chassis_sigmas` (default `{0.05, 0.05, 0.05, 0.02, 0.02, 0.02}`) are wrapped in `gtsam::noiseModel::Robust::Create(Huber::Create(huber_k), …)` (default `huber_k = 1.5`) so short-duration outliers from wheel slip get downweighted without rejecting legitimate hard-acceleration steps. Inflation by `shock_inflation_factor` fires on any of: an IMU shock/free-fall trigger (`kFusionStatusShockInflated`); a RIO-reported `kChassisStatusSlip` (`kFusionStatusSlipReported`); or, in Phase C, a sustained chassis-vs-IMU velocity disagreement (`kFusionStatusSlipDetected`, see §4). Multiple bits can fire on the same step; each rebuilds the noise model with multiplied sigmas before re-wrapping in Huber.

Before any chassis factor is built, `FusionService::process()` checks `√(vx² + vy²)` against `max_chassis_speed_mps` (default 6.5 m/s) and drops the sample if it exceeds the bound. The drop is counted as `chassis_speed_gated`, the per-type cursor is advanced (so subsequent good samples aren't rejected as stale), and the next emitted estimate carries `kFusionStatusDegradedInput`. This is data hygiene: a CAN glitch reporting 50 m/s never reaches the BetweenFactor where Huber would only partially blunt it.

**`PriorFactor<Pose3>` for the floor constraint (F-2).** When `enable_floor_constraint` is true (default), every new pose key gets a soft prior at `Pose3::identity()` with sigmas `[σ_roll, σ_pitch, 1e6, 1e6, 1e6, σ_z]` in tangent order — i.e. very loose on x/y/yaw, tight on z/roll/pitch. Defaults are σ_z = 0.01 m and σ_roll = σ_pitch = 0.0087 rad (≈ 0.5°). The factor is built per-call-site (no cached noise model) so live-reloading the sigmas takes effect on the next keyframe without a graph rebuild. Toggling `enable_floor_constraint` itself is structural — flipping on creates an asymmetry between old and new keys (old keys keep their lack-of-prior; new keys gain one), and flipping off cannot remove priors already in the graph; both require a daemon restart to take effect cleanly.

**`PriorFactor<Pose3>` for vision-anchored field fix.** Same construction as above; this is what transitions `kAwaitingFieldFix → kRunning` when `enable_imu_preintegration` is on, by also inserting an initial velocity variable + a 5 m/s prior on it so subsequent ImuFactors have a tail key to link against.

**`gtsam::ImuFactor` for IMU preintegration (Phase C, flag-gated).** When `enable_imu_preintegration` is on and `state_ == kRunning`, every keyframe commits one `ImuFactor` linking the previous pose+velocity keys, the new pose+velocity keys, and a constant `gtsam::imuBias::ConstantBias` symbol. Preintegration is built off `gtsam::PreintegrationParams::MakeSharedU(g = 9.80665)` with body-to-IMU extrinsic from `imu_extrinsic_body_to_imu`. Out-of-order IMU samples are dropped with `imu_out_of_order` incremented; a forward gap > `max_imu_gap_seconds` resets the active preintegrator and skips the IMU factor for the in-progress interval (chassis-only fallback).

**`PriorFactor<Vector3>` for velocity bootstrap.** Inserted once on the field-fix transition with σ = 5 m/s; ImuFactors take over from there.

### 3.3 Update strategy

`FusionBackend::update()` drives `gtsam::ISAM2` with `relinearizeThreshold = 0.01`, `relinearizeSkip = 1` — relinearize whenever the relative error change exceeds 1%, every iteration. Two `update()` calls per measurement (one with the new factors, one empty to push relinearization), then `calculateEstimate()` to refresh `current_pose_` and `current_velocity_`.

When IMU preintegration is on, every successful keyframe commit also calls `marginalizeOldKeys()` once `live_keys_.size()` exceeds `marginalize_keyframe_window` (default 500). Older leaf keys are handed to `ISAM2::marginalizeLeaves`, bounding the working-set size for long matches. Marginalization failures are caught and benignly retried on the next commit.

### 3.4 Output construction

`makeEstimate()` builds the `FusedPoseEstimate`:

- `timestamp` = the triggering measurement's timestamp.
- `field_to_robot` = `Pose2d{x, y, yaw}` extracted from the current `Pose3` (z, roll, pitch are not in the planar host-side `FusedPoseEstimate` struct; they live in the wider USB payload built by the encoder — see §7).
- `velocity` = body-frame `Vec3` from `current_velocity_` once `state_ == kRunning`; `nullopt` otherwise so the wire format signals "unavailable" rather than emitting a stale zero.
- `covariance` = `isam_.marginalCovariance(current_key)` flattened row-major (gtsam tangent order `[rx, ry, rz, tx, ty, tz]`). On exception, zeroed and `kFusionStatusMarginalUnavailable` is OR'd in.
- `status_flags` = the full bitfield laid out in §7. Init-state bits (`kFusionStatusAwaitingFieldFix`, `kFusionStatusBiasUnverified`) are surfaced through `initStateStatusBits()` so consumers can see why the graph is in degraded mode.

### 3.5 Out-of-order and gap handling

`acceptTimestamp()` rejects any measurement older than the last accepted one **of the same `Measurement` variant alternative** (counted into `stats_.stale_measurements`). IMU samples bypass the cursor entirely so they can interleave with chassis at 1 kHz; their own monotonicity is enforced by the backend's `last_imu_time_`. AprilTag and chassis cursors are independent — a chassis sample at t=10 ms does not silently drop an AprilTag observation at t=9 ms.

`addChassisSpeeds` skips the `BetweenFactor` if `Δt <= 0` or `Δt > max_chassis_dt_seconds` and emits the existing pose with `kFusionStatusChassisGap` set. Without this guard a 5 s gap would compound a 5 m/s × 5 s = 25 m relative-motion factor with the wrong noise model and likely diverge the graph. Phase C's keyframe accumulator carries the same guard via `chassis_gap_seen_`.

---

## 4. IMU integration

The IMU enters the system three ways:

**Shock / free-fall detection (always on).** Every `ImuSample` populates a sliding window of `ImuShockSample` (timestamp + `|a|` + `|a − g_local|`) for the most recent `imu_window_seconds` (default 50 ms). On every chassis update, `detectShockOrFreefall()` scans the window:

- If any sample has `|a − g_local| > shock_threshold_mps2` (default 50 m/s² ≈ 5g), set the shock flag.
- If any sample has `|a| < freefall_threshold_mps2` (default 3 m/s²), set the shock flag.

If either branch fires, every diagonal sigma on the chassis `BetweenFactor` is multiplied by `shock_inflation_factor` (default 100×) for that one step, the noise is re-wrapped in Huber, and the estimate is flagged with `kFusionStatusShockInflated`. This catches collisions and the "wheels off the ground over a bump" case.

**Wheel slip (always on, RIO-driven).** When the chassis sample's `status_flags` has `kChassisStatusSlip` set, the same sigma-inflation path runs and `kFusionStatusSlipReported` is OR'd into the output. This is independent of IMU shock detection — slip can happen with the IMU reading nominal accelerations (e.g. wheels spinning on a polished floor).

**Wheel slip (Phase C, chassis-vs-IMU disagreement).** When `enable_imu_preintegration` is on and the state machine has reached `kRunning`, `commitKeyframe` compares chassis-mean body velocity against the IMU-predicted body velocity from `active_pim_->predict(...)` (rotated nav→body via `current_pose_.rotation().unrotate(...)`). If `||chassis_v − imu_v_body|| > slip_disagreement_mps` (default 1.0) for two consecutive keyframes (`kSlipDisagreementHysteresis = 2`), the chassis BetweenFactor for the inflating keyframe is built with `shock_inflation_factor`-multiplied sigmas, `kFusionStatusSlipDetected` is OR'd into the estimate, and the streak resets. The check is skipped when `pending_.chassis_dt_accum < 0.5 × max_keyframe_dt_seconds` (avoids noisy short windows) or when the RIO already flagged slip on this keyframe (avoids double-inflation). `slip_disagreement_events` and `slip_disagreement_inflations` counters surface through `FusionStats` and the daemon health JSON. Inert until Phase C is on.

**Preintegration (Phase C, gated by `enable_imu_preintegration`).** When the flag is on and the state machine has reached `kRunning`:

1. Every IMU sample calls `gtsam::PreintegratedImuMeasurements::integrateMeasurement(accel, gyro, dt)` against the active preintegrator, gated on monotonic timestamps and `max_imu_gap_seconds` (100 ms default).
2. Chassis samples are accumulated into a `PendingKeyframe` (composed Δ-twist + total dt).
3. Keyframe commit fires when either a vision observation arrives or `max_keyframe_dt_seconds` (20 ms default) has elapsed since the last commit. The commit builds the Huber-wrapped chassis `BetweenFactor`, the `ImuFactor` (if `has_imu`), inserts the next pose + velocity keys, calls `ISAM2::update`, and resets the preintegrator with `current_bias_`.
4. The preintegrator is rebuilt against `current_bias_` at every commit, every long-gap reset, and at the field-fix transition.

The init state machine — `kBoot → kCalibratingBias → kAwaitingFieldFix → kRunning` — sequences this:

- `kBoot`: load persisted bias (`config_.persisted_bias`).
- `kCalibratingBias`: every IMU sample contributes to the running mean while the chassis cursor reads quiet (`|v| < bias_calibration_chassis_threshold` AND `|ω| < bias_calibration_chassis_threshold`); after `bias_calibration_seconds` elapses, the mean (with gravity subtracted from accel) becomes the new bias and the state advances. If the window saw any non-quiet chassis, the persisted bias stays in place and `kFusionStatusBiasUnverified` is set.
- `kAwaitingFieldFix`: chassis BetweenFactors keep flowing in body frame (legacy path). IMU factors are suppressed — `gtsam::PreintegrationParams::MakeSharedU` requires a defined nav frame, which the first AprilTag observation supplies.
- `kRunning`: keyframe-driven commits with ImuFactors active. `velocity` populated on the wire.

When `enable_imu_preintegration` is off (default), the legacy chassis path runs identically to before — the new state machine stays in `kBoot`, the preintegrator stays null, and no ImuFactor or velocity key is ever inserted.

---

## 5. Field tag layout: still pipeline-only (deferred per §8.4)

The field layout (`AprilTagFieldLayout`) is loaded into `RuntimeConfig::field_layouts` and passed into each `AprilTagPipeline` so the pipeline can build the world-corner array for the multi-tag SQPNP solve (see [`apriltag-pipeline.md`](apriltag-pipeline.md)). By the time the observation reaches fusion, it has already been collapsed to a 6-DOF `field_to_robot` pose plus a 6×6 covariance; the per-tag detections and the field map itself are not visible to the fusion graph.

This is a deliberate split (it keeps the heavy PnP solve in the camera-bound pipeline thread and away from the graph), but it forecloses a class of refinements: the graph can never decide "tag 7 looks like a flip; downweight it" or "tag 12's reprojection is suspiciously high; gate it out". Whatever the AprilTag pipeline outputs is what fusion eats. The implementation plan defers the landmark-factor option until single-camera flips become a measured problem (see §8.4); for the in-scope work the aggregated `PriorFactor<Pose3>` plus the per-camera dynamic covariance from the AprilTag side are sufficient.

---

## 6. Configuration: SQLite-backed, with live reload

Every parameter named in §3 lives on the `fusion::FusionConfig` struct in `include/posest/fusion/FusionService.h`. The runtime-config mirror in `runtime::FusionConfig` (`include/posest/runtime/RuntimeConfig.h`) is what the SQLite store reads and writes; the factory function:

```cpp
FusionConfig buildFusionConfig(const runtime::RuntimeConfig& runtime_config);
```

— `FusionService.cpp` — copies every field from `runtime_config.fusion` into the active `fusion::FusionConfig`. The persistence chain is:

1. **`runtime::FusionConfig`** (`include/posest/runtime/RuntimeConfig.h`) — the singleton tunable struct. Mirrors the in-process `fusion::FusionConfig` field-for-field but lives in the runtime-config namespace to keep the SQLite layer free of GTSAM/Eigen includes.
2. **`fusion_config` SQLite table** (latest migration 10, `src/config/SqliteSchema.cpp`) — singleton row, `id INTEGER PRIMARY KEY CHECK (id = 1)`, with one column per scalar field. Migration 10 added `enable_floor_constraint`, `floor_constraint_sigma_{z,roll,pitch}`, and `max_chassis_speed_mps`. Loaded and saved by `SqliteConfigStore` atomically with the rest of `RuntimeConfig`.
3. **`ConfigValidator::validateRuntimeConfig`** (`src/config/ConfigValidator.cpp`) — enforces sigma > 0, `shock_threshold > freefall_threshold`, `shock_inflation_factor >= 1`, `bias_calibration_seconds >= 0.5`, `max_keyframe_dt_seconds ∈ [0.005, 0.1]`, `max_imu_gap_seconds > max_keyframe_dt_seconds`, `max_chassis_speed_mps >= 1.0`, `floor_constraint_sigmas[i] > 0` when the floor is enabled (any finite value otherwise), and so on.

### 6.1 Live reload (F-1)

`FusionService::applyConfig(FusionConfig)` is the runtime entry point. It writes the new config into a `pending_config_` slot under the worker mutex; the worker observes the slot at the top of every `process()` iteration, swaps it into a local, and calls `FusionBackend::applyConfig`. ISAM2 stays single-threaded — no in-flight factor build is interrupted.

`FusionBackend::applyConfig` rebuilds the cached chassis / robust-chassis / origin-prior noise models from the new sigmas. It then classifies the change:

- **Live-reloadable**: chassis & origin sigmas, shock thresholds, `shock_inflation_factor`, `imu_window_seconds`, `max_chassis_dt_seconds`, `gravity_local_mps2`, `huber_k`, `vio_default_sigmas`, `enable_vio` is *not* live (see below), `slip_disagreement_mps`, `marginalize_keyframe_window`, `max_keyframe_dt_seconds`, `max_imu_gap_seconds`, `max_chassis_speed_mps`, `floor_constraint_sigmas`. These take effect on the next factor built.
- **Structural (rejected, counted in `config_reloads_structural_skipped`)**: `enable_imu_preintegration`, `enable_vio`, `enable_floor_constraint`, IMU noise sigmas (`accel_noise_sigma`, `gyro_noise_sigma`, `accel_bias_rw_sigma`, `gyro_bias_rw_sigma`, `integration_cov_sigma`), `imu_extrinsic_body_to_imu`, `persisted_bias`, `bias_calibration_seconds`, `bias_calibration_chassis_threshold`. These are baked into long-lived state (preintegration params, bias seed, graph topology) at construction time and need a daemon restart to apply cleanly.

Wire-up: `Daemon::loadAndBuild` constructs `WebService` with an `on_saved` callback that runs after a successful `saveRuntimeConfig`. The callback rebuilds `fusion::FusionConfig` from the new `RuntimeConfig` and hands it to `FusionService::applyConfig`. The web request thread holds the `FusionService::mu_` only briefly to write `pending_config_`; no graph work happens on that thread.

### 6.2 Bias persistence on shutdown (F-7)

When the daemon stops cleanly, it calls `fusion_->currentBiasIfTrusted()` *before* `fusion_->stop()`. The accessor returns `nullopt` until the boot stationary calibration window has produced a fresh bias mean — this means the seeded persisted-bias is never overwritten by the in-memory placeholder if calibration didn't complete. When trusted, the bias is converted to `[ax, ay, az, gx, gy, gz]`, written to `RuntimeConfig.fusion.persisted_bias`, and saved via `IConfigStore::saveRuntimeConfig`. Save failures are swallowed: shutdown must always make forward progress, and the next boot's stationary window will recalibrate anyway. `persisted_bias` is loaded into the backend constructor — the next boot starts from the previous match's bias rather than identity.

---

## 7. Output to the RoboRIO over CAN

The output path is fully wired and working at protocol v4 (376-byte USB payload, 64-byte CAN-FD frame):

1. **`FusionService::publishEstimate`** hands the `FusedPoseEstimate` to every registered sink outside the service mutex; per-stage publish latency is recorded into the `publish_us` histogram surfaced through `DaemonHealth`.
2. **`TeensyService::publish`** (`src/teensy/TeensyService.cpp`) captures `host_send_time_us = steady_clock::now()` and encodes a 376-byte USB payload carrying that timestamp, full Pose3 (`x`, `y`, `z`, `roll`, `pitch`, `yaw` as doubles), velocity (`vx`, `vy`, `vz` doubles), a `has_velocity` flag, the 32-bit `status_flags`, and the full row-major 6×6 covariance in gtsam tangent order. The protocol version on both sides is `kProtocolVersion = 4` (`include/posest/teensy/Protocol.h`, `firmware/teensy41/include/Protocol.h`); v3 → v4 was a clean break, not negotiated.
3. **Teensy firmware** (`firmware/teensy41/src/main.cpp`) decodes the v4 payload via `decodeFusedPosePayload` (`firmware/teensy41/src/Protocol.cpp`) and caches it as the latest `FusedPose3Payload`. The decode-handler also stamps `decode_time_us = micros()` and feeds it to `CanBridge::setLatestFusedPose` so the bridge can measure firmware-side latency (F-6).
4. **`CanBridge::maybeSendFusedPose`** transmits at `1'000'000 / pose_publish_hz` µs (default 10 ms / 100 Hz) on CAN-FD ID `teensy_pose_can_id`. Each successful TX records `now_us − decode_time_us` into a rolling window (`min_us`, `avg_us`, `max_us`, `samples`), reset by `sendHealth` on every health emit so each `TeensyHealthPayload` carries the latency stats for the most recent ~100 ms window.
5. **CAN frame layout** (`writeFusedPoseFrame` in `firmware/teensy41/src/CanBridge.cpp`, schema in `firmware/teensy41/include/CanSchema.h`), 64 bytes exact, zero padding-free:

   | offset | size | field |
   | -----: | ---: | ----- |
   |  0 |  8 | `teensy_time_us` (u64) |
   |  8 |  8 | `x_m` (double) |
   | 16 |  8 | `y_m` (double) |
   | 24 |  4 | `z_m` (float) |
   | 28 |  4 | `roll_rad` (float) |
   | 32 |  4 | `pitch_rad` (float) |
   | 36 |  4 | `yaw_rad` (float) |
   | 40 |  4 | `vx_mps` (float, NaN if velocity unavailable) |
   | 44 |  4 | `vy_mps` (float) |
   | 48 |  4 | `vz_mps` (float) |
   | 52 |  4 | `sigma_x_m` (float = sqrt(cov(tx, tx))) |
   | 56 |  4 | `sigma_y_m` (float = sqrt(cov(ty, ty))) |
   | 60 |  2 | `sigma_yaw_mrad` (uint16; sentinel 0xFFFF = NaN) |
   | 62 |  2 | `status_flags` (uint16, low 16 bits of full 32-bit field) |

   Velocity fields encode `NaN` when `FusedPoseEstimate::velocity` is unavailable (pre-`kRunning` or pre-bootstrap); sigma fields encode `NaN`/`0xFFFF` when the marginal covariance was unavailable. The RIO must check `kFusionStatusMarginalUnavailable` before consuming.

6. **Firmware-side latency window in `TeensyHealthPayload` (F-6, v4).** The 76-byte v4 health payload appends four `uint32_t` fields at offset 60: `fused_pose_decode_to_tx_min_us`, `fused_pose_decode_to_tx_avg_us`, `fused_pose_decode_to_tx_max_us`, `fused_pose_latency_samples`. The window resets every health emit (~10 Hz). `samples == 0` ⇒ no FusedPose was transmitted in the most recent window. The host's `TeensyService` mirrors the four fields onto `TeensyStats`; `Daemon::healthToJson` emits them under `teensy.fused_pose_*`, surfaced by `posest_daemon --health-once`. `kFusionStatusDegradedInput` covers the F-3 chassis-speed gate (no separate bit).

   Implication for v4 deployment: a host running v4 against a v3 firmware (or vice versa) fails fast at the protocol-version check — no silent corruption. Reflash the firmware whenever the host is upgraded across this boundary.

The status-flag bit layout (low 16 bits, full 32-bit set on USB; `kFusionStatus*` constants in `include/posest/fusion/FusionService.h`):

| Bit | Constant | Meaning |
| --: | -------- | ------- |
| 0 | `kFusionStatusInitializing` | Pre-bootstrap; pose not yet anchored. |
| 1 | `kFusionStatusVisionUnavailable` | No vision observation in the most recent estimate. |
| 2 | `kFusionStatusMarginalUnavailable` | `ISAM2::marginalCovariance` threw; covariance fields are zero. |
| 3 | `kFusionStatusOptimizerError` | `ISAM2::update` rejected the most recent factor; pose was held over. |
| 4 | `kFusionStatusDegradedInput` | RIO `ChassisSpeedsSample::status_flags` was non-zero. |
| 5 | `kFusionStatusShockInflated` | IMU shock or free-fall fired; chassis sigmas multiplied by `shock_inflation_factor`. |
| 6 | `kFusionStatusChassisGap` | `Δt > max_chassis_dt_seconds`; `BetweenFactor` was skipped. |
| 7 | `kFusionStatusSlipReported` | `kChassisStatusSlip` was set on the chassis sample; chassis sigmas inflated independently of IMU shock. |
| 8 | `kFusionStatusImuGap` | IMU gap > `max_imu_gap_seconds`; preintegrator was reset. |
| 9 | `kFusionStatusBiasUnverified` | Boot stationary calibration timed out without enough quiet samples. |
| 10 | `kFusionStatusAwaitingFieldFix` | Graph initialized in body frame; vision has not yet anchored to field — IMU factors suppressed. |
| 11 | `kFusionStatusSlipDetected` | Phase C: chassis-vs-IMU velocity disagreement crossed `slip_disagreement_mps` for `kSlipDisagreementHysteresis = 2` consecutive keyframes. Inert until Phase C is on. |

---

## 8. What's missing (deferred items only)

The list below carries only items still open after the F-1/F-2/F-3/F-4/F-6/F-7 work landed. Everything else from the previous gap inventory has been resolved; the §1 requirements are satisfied for the in-scope work.

### 8.4 Field tag layout not used as landmarks (deferred)

If the graph held landmark variables for each known tag and consumed individual `AprilTagDetection` entries (currently nested in `AprilTagObservation::detections` but unused by fusion), it could:
- Detect per-tag flip ambiguities the pipeline missed.
- Reject single-tag flips by comparing against the known tag pose.
- Improve the rotation observability when the pipeline-aggregated pose has an inflated rotational covariance.

This is a larger change — it splits the PnP work between the pipeline and the graph — and may not pay off until single-camera flips become a measured problem. Defer until the simpler items above are validated on hardware; documented here so it's not forgotten.

### 8.9 VioMeasurement is gated, not silently dropped (revisit when VIO ships)

The fusion service now consumes `VioMeasurement` through a dedicated path (`FusionBackend::addVio`) gated by `FusionConfig::enable_vio` and the per-sample `tracking_ok` flag. The placeholder VIO pipeline still publishes `tracking_ok = false` on every sample, so in practice nothing reaches the graph today; counter `measurements_vio_skipped_no_tracking` exposes that. When the real VIO frontend lands the gate stays in place — flip `enable_vio` via config (currently a structural change, requires restart) and verify the existing tests in `test/test_fusion_service.cpp` (`IngestsVioMeasurementWhenTrackingOk`, `SkipsVioMeasurementWhenTrackingNotOk`, `SkipsVioMeasurementWhenDisabledByConfig`) still represent the contract.

### Phase C IMU preintegration: validation pending

`enable_imu_preintegration` defaults to `false` in both the runtime config and the SQLite default row. Toggle to `true` only after the on-robot validation steps:

- Stationary boot: `bias_calibrations_completed` reaches 1 within ~2 s.
- First AprilTag: `state_` reaches `kRunning`, `velocity` populates on the wire.
- Closed-loop drive: pose closure error < ~5 cm at end-of-loop.
- Long marginalization soak: > 600 keyframes with the floor on; `graph_factor_count` plateaus at ~marginalize-window × per-keyframe-factor-count; no z drift.

F-4's slip-disagreement detector and the velocity field on `FusedPoseEstimate` are inert until Phase C is on but otherwise ready. Once validated, the SQLite default row's `enable_imu_preintegration` flag flips to 1 in a follow-up migration.

### Optional follow-ups (not in any phase)

- **End-to-end (host-to-CAN) latency, not just firmware-side.** F-6 measures decode-to-CAN-TX on the firmware. The `host_send_time_us` prefix on the v4 FusedPose payload is the other half of an end-to-end measurement, but the bridge currently only diffs against its own `decode_time_us`. Adding a second tracker that diffs CAN-TX against the time-sync-translated `host_send_time_us` would expose USB transport latency. Cheap (~10 LOC) once we want it.
- **Live reload of structural fields.** Today, toggling `enable_imu_preintegration`, `enable_vio`, `enable_floor_constraint`, IMU noise sigmas, IMU extrinsic, or `persisted_bias` via the website is silently rejected on the live path (counted in `config_reloads_structural_skipped`) and requires a daemon restart. Building a "stop, reconfigure, restart fusion" path inside the daemon is straightforward but invites a graph-history-loss footgun; defer until users actually need it.
- **Migrate to `CombinedImuFactor`.** `ImuFactor` injects bias as a constant per factor; `CombinedImuFactor` adds bias-random-walk between successive bias keys. Worth doing when matches get long enough that stationary-only bias estimation starts to drift. Persisted bias (F-7) covers the simpler case of "use last match's bias as the seed."
- **Configurable slip-disagreement hysteresis (F-4 follow-up).** `kSlipDisagreementHysteresis` is a `static constexpr int = 2`. Move to `FusionConfig` once on-robot data tells us 2 isn't the right value.

---

## 9. Tests

`test/test_fusion_service.cpp` covers the running graph:

**Phase A/B (always-on path):**
- Chassis integration produces the expected pose displacement.
- An `AprilTagObservation` becomes the bootstrap prior.
- Non-positive-definite vision covariance is rejected and flagged.
- A subsequent empty-tag observation flags `VisionUnavailable` without resetting the pose.
- Out-of-order chassis samples are counted as stale.
- IMU shock (>50 m/s² departure from gravity) inflates step noise and flags it.
- IMU free-fall (<3 m/s² absolute) inflates step noise and flags it.
- Normal driving (~1.2 m/s² lateral) does not trigger the shock branch.
- A `Δt > max_chassis_dt_seconds` gap skips the BetweenFactor and flags `ChassisGap`.
- A chassis sample with `kChassisStatusSlip` set inflates noise even when the IMU is quiet (Phase B).
- Per-stage latency histograms accumulate samples on every chassis update (Phase E).
- `VioMeasurement` ingestion respects both the `enable_vio` config gate and the per-sample `tracking_ok` flag.
- `buildFusionConfig` round-trips every tunable field from `runtime::FusionConfig` to `fusion::FusionConfig` (Phase A).

**F-2 floor constraint:**
- `FloorConstraintAcceptsAprilTagWithExactZ` — with the floor on, a vision prior with very loose z/roll/pitch sigmas still produces a marginal covariance whose z/roll/pitch diagonals are tightened by the floor factor.
- `FloorConstraintDisabledLeavesUnconstrainedDOFsLoose` — with the floor off, the loose vision sigmas survive into the marginal covariance unchanged.

**F-3 max chassis speed gate:**
- `GatesChassisAboveMaxSpeed` — a 50 m/s sample produces `chassis_speed_gated == 1` and `kFusionStatusDegradedInput` on the next estimate; the BetweenFactor is never built.
- `AcceptsChassisAtMaxSpeedBoundary` — strict-greater-than: a sample at exactly the configured ceiling is accepted.

**F-1 live config reload:**
- `ApplyConfigUpdatesLiveCounter` — a non-structural change increments `config_reloads_applied` and not `config_reloads_structural_skipped`.
- `ApplyConfigStructuralChangeIsSkipped` — toggling `enable_imu_preintegration` increments the structural counter and is otherwise inert (Phase C bookkeeping doesn't advance).

**F-4 slip-disagreement (Phase C):**
- `SlipDisagreementInflatesOnSustainedDelta` — chassis at 2 m/s with stationary IMU produces ≥1 inflation event after the 2-step hysteresis; at least one estimate in the sink history carries `kFusionStatusSlipDetected`.
- `SlipDisagreementSkippedWhenRioFlagAlreadySet` — when the chassis sample carries `kChassisStatusSlip`, F-4 yields to the existing RIO-driven path; `kFusionStatusSlipReported` set, `kFusionStatusSlipDetected` not set, F-4 counters stay zero.

**F-7 bias persistence accessor:**
- `CurrentBiasReturnsNulloptBeforeCalibration` — the accessor is `nullopt` until the boot stationary calibration window closes.
- `CurrentBiasReflectsCalibrationResult` — feeding a stationary IMU window with a known accel/gyro offset produces the expected bias vector after calibration.

**FusionServiceImu (Phase C):**
- Bias calibration completes during a stationary boot and advances to `kAwaitingFieldFix`.
- The first vision observation transitions to `kRunning` and starts populating `velocity`.
- Out-of-order IMU samples are counted as stale or `imu_out_of_order`.

**Config schema + validator (`test/test_config_schema.cpp`):**
- `FusionConfigRoundTripsAndReopens` extended for `enable_floor_constraint`, `floor_constraint_sigmas`, `max_chassis_speed_mps`.
- `FusionConfigDefaultsForFreshDatabase` extended for migration-10 default-on floor + 6.5 m/s gate.
- `RejectsFloorConstraintNonPositiveWhenEnabled`, `AllowsFloorConstraintZeroSigmaWhenDisabled`, `RejectsMaxChassisSpeedBelowOne` for the new validator bounds.

**Wire-format (`test/test_teensy_protocol.cpp` + `test/test_teensy_protocol_golden.cpp`):**
- `kProtocolVersion = 4` golden assertion.
- `FusedPosePayloadV4LayoutAndSize` (376 B) + `FusedPosePayloadV4OmitsVelocityWhenAbsent`.
- `TeensyHealthPayloadRoundTripsExtended` (76 B) round-trips the four new latency fields.
- Golden size assertions for `TeensyHealthPayload` (76 B) and `FusedPose3Payload` (376 B).

`test/test_latency_histogram.cpp` covers the rolling-window utility on its own.

Notably absent: tests for the sink fan-out path with multiple sinks, marginal-covariance failure recovery, and a long-running marginalization soak (>500 keyframes).

---

## 10. Quick-reference: files

- **Service:** `include/posest/fusion/FusionService.h` (`applyConfig`, `currentBiasIfTrusted`), `src/fusion/FusionService.cpp` (`FusionBackend::applyConfig`, floor factor at all 5 keypoints, F-3 gate in `process()`, F-4 in `commitKeyframe`)
- **Sink interface:** `include/posest/fusion/IFusionOutputSink.h`
- **Output type:** `include/posest/MeasurementTypes.h` (`FusedPoseEstimate`, `kChassisStatusSlip`)
- **Config struct:** `include/posest/runtime/RuntimeConfig.h` (`runtime::FusionConfig`)
- **SQLite schema (latest migration 10):** `src/config/SqliteSchema.cpp`, load/save in `src/config/SqliteConfigStore.cpp`, validation in `src/config/ConfigValidator.cpp`
- **Web → fusion reload wiring:** `include/posest/runtime/WebService.h` (`ConfigSavedCallback`), `src/runtime/WebService.cpp` (invokes the callback after `saveRuntimeConfig`), `src/runtime/Daemon.cpp` (installs the callback wrapping `applyConfig`)
- **Latency histogram:** `include/posest/util/LatencyHistogram.h`
- **Wire format (host side):** `src/teensy/TeensyService.cpp` (`encodeFusedPosePayload`), `include/posest/teensy/Protocol.h` (`kProtocolVersion = 4`)
- **Wire format (firmware side):** `firmware/teensy41/include/Protocol.h` (`FusedPose3Payload` with `host_send_time_us`), `firmware/teensy41/src/Protocol.cpp` (`decodeFusedPosePayload`, expected size 376 B)
- **CAN bridge:** `firmware/teensy41/src/CanBridge.cpp` (`writeFusedPoseFrame`, F-6 latency tracker in `transmitTeensyPose`), `firmware/teensy41/include/CanBridge.h` (`FusedPoseLatencyWindow` + `setLatestFusedPose(pose, decode_time_us)`), `firmware/teensy41/include/CanSchema.h` (offsets, `kTeensyPosePayloadBytes = 64`)
- **Daemon wiring + telemetry:** `src/runtime/Daemon.cpp` (`refreshHealth`, `healthToJson`, F-7 bias-on-shutdown in `stop()`); `DaemonHealth.fusion` is a `fusion::FusionStats` carrying Phase C, F-1, F-3, F-4 counters; `DaemonHealth.teensy.fused_pose_*` exposes the F-6 latency window.
- **Tests:** `test/test_fusion_service.cpp`, `test/test_config_schema.cpp`, `test/test_latency_histogram.cpp`, `test/test_teensy_protocol.cpp`, `test/test_teensy_protocol_golden.cpp`
