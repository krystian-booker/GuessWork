# Fusion Service (GTSAM)

**Status:** Partially complete. Core graph runs end-to-end: `FusionService` consumes `ChassisSpeedsSample` and `AprilTagObservation` from the `MeasurementBus`, runs an incremental iSAM2 graph (Pose3-only, BetweenFactor for odometry, PriorFactor for vision), publishes a `FusedPoseEstimate` through `IFusionOutputSink` into `TeensyService`, and the Teensy firmware re-broadcasts the pose on CAN-FD at a configurable rate. IMU shock/free-fall detection inflates step noise. **Significant gaps remain before this is feature-complete:** no IMU preintegration (IMU is shock-only, not state-propagating), no wheel-slip detection independent of IMU, no robust noise on the odometry factor, no use of the field tag layout as landmarks, fusion parameters are hard-coded (the SQLite store has no `fusion_*` tables yet, and `buildFusionConfig(RuntimeConfig)` ignores its argument), and the published estimate collapses to `Pose2d` even though the graph carries `Pose3`.

This document covers the GTSAM-based sensor fusion service end-to-end: what it is, how it integrates with the rest of the runtime, how the graph is constructed, and what's missing before it meets the high-performance-FRC bar laid out in §1. Generic plumbing (`MeasurementBus`, the non-blocking-publisher rule) lives in [`producer-consumer-architecture.md`](producer-consumer-architecture.md); the AprilTag-side covariance and aggregation that feed this service live in [`apriltag-pipeline.md`](apriltag-pipeline.md); the Teensy USB framing and CAN bridge live in [`teensy-hardware-interface.md`](teensy-hardware-interface.md).

---

## 1. Requirements (target behavior)

The bar this document is reviewing against:

1. **Inputs:** all AprilTag-pipeline outputs (one `AprilTagObservation` per camera per frame), Teensy-mediated IMU samples (`ImuSample`, ~1 kHz), and Teensy-mediated chassis kinematics from the RoboRIO (`ChassisSpeedsSample`, ~100 Hz).
2. **VIO is deferred.** A `VioMeasurement` type already exists; the fusion service must remain easy to extend when VIO lands, but is not required to consume it today.
3. **Platform reality:** the robot is high-performance FRC, max ~5.2 m/s, always grounded (floor-constrained), and routinely takes hard impacts (defense, ramps, bumpers) and experiences wheel slip on aggressive direction changes.
4. **Configurable from the website.** Every fusion knob (process noise, prior sigmas, shock thresholds, IMU window, chassis-gap timeout) must be data — JSON the future HTTP server can read and write — persisted in SQLite, and reloaded into the running graph when changed. The web server is not yet built; the parameters must already be addressable via `RuntimeConfig` and `IConfigStore` so the wiring is complete the moment the HTTP layer lands.
5. **Use the AprilTag field layout** if it tightens the graph. The layout is already loaded into `RuntimeConfig::field_layouts` for the vision pipeline; if landmark factors or per-tag constraints would help fusion, the same data should flow into the graph.
6. **Output goes to the RoboRIO over CAN, via the Teensy.** Fusion publishes a `FusedPoseEstimate`; `TeensyService` encodes it into a USB `FusedPose` frame; the Teensy's `CanBridge` re-broadcasts it on the configured CAN ID at the configured rate. No path to the RIO except through the Teensy.
7. **Latency.** This is a real-time pose source for a 5.2 m/s robot. End-to-end latency from `ChassisSpeedsSample` arrival → CAN frame on the bus must stay tight. The non-blocking-publisher rule from [`producer-consumer-architecture.md`](producer-consumer-architecture.md#23-the-latency-contract) applies: nothing on the publish path may block another thread.

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
| `FusionConfig` | `include/posest/fusion/FusionService.h:32` | Compile-time-default tuning struct. **Currently ignored by the config store**; see §6. |
| `IFusionOutputSink` | `include/posest/fusion/IFusionOutputSink.h` | One-method abstract sink. Implemented by `TeensyService`. Multiple sinks supported (e.g. for future telemetry/logging). |
| `FusedPoseEstimate` | `include/posest/MeasurementTypes.h:117` | The published payload: `timestamp`, `Pose2d field_to_robot`, 6×6 `covariance` (gtsam tangent order), `status_flags`, optional `velocity` (currently never populated). |
| `MeasurementBus` | `include/posest/MeasurementBus.h` | Bounded MPSC queue (capacity 4096 in `Daemon.cpp`) of `Measurement = std::variant<...>`. Drop-newest on overflow. |
| `TeensyService` | `src/teensy/TeensyService.cpp:100,668` | Implements `IFusionOutputSink::publish`: encodes 28-byte payload (`x`, `y`, `θ`, `status_flags`) into a `MessageType::FusedPose` USB frame and enqueues it. |
| `CanBridge` | `firmware/teensy41/src/CanBridge.cpp:248` | Periodic CAN-FD TX of the most recent host-supplied pose at `pose_publish_hz`. |

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

There is no measurement of this path in code today; latency budgeting and instrumentation are §7 work.

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

Only one variable type is in the graph: `gtsam::Pose3`, keyed by `gtsam::Symbol('x', index)` with `index` advancing on every accepted chassis update. A vision update reuses the current key — it tightens the existing pose rather than introducing a new variable. There is **no** velocity, gyro-bias, accel-bias, or IMU navstate variable. This is a deliberate simplification documented in `process()` (`FusionService.cpp:357-359`): the 1 kHz IMU and ~100 Hz chassis streams interleave naturally; the graph models the chassis cursor only.

### 3.2 Factors

**`PriorFactor<Pose3>` for bootstrap.** On the first chassis sample, the backend anchors `x_0` at identity with very loose sigmas (`origin_prior_sigmas = {10 rad, 10 rad, π rad, 10 m, 10 m, 10 m}` — `FusionService.h:36`). The first `AprilTagObservation` then snaps the pose into the field frame.

**`PriorFactor<Pose3>` for vision (`FusionService.cpp:201`).** Each `AprilTagObservation` with a non-empty `field_to_robot` becomes a prior on the **current** key. The 6×6 covariance comes directly from the AprilTag pipeline (see [`apriltag-pipeline.md`](apriltag-pipeline.md)) and is converted with `gtsam::noiseModel::Gaussian::Covariance`. If GTSAM rejects the matrix as not positive-definite, the factor is dropped and the estimate is published with `kFusionStatusOptimizerError` set (no robust fallback).

**`BetweenFactor<Pose3>` for chassis odometry (`FusionService.cpp:155`).** Each accepted `ChassisSpeedsSample` produces a relative-motion factor between the previous and next pose key. The delta is `Pose3(Rot3::Rz(ω·Δt), Point3(vx·Δt, vy·Δt, 0))` — yaw-only rotation, planar translation. Process noise is the diagonal `chassis_sigmas` (default `{0.05, 0.05, 0.05, 0.02, 0.02, 0.02}`).

**No other factors.** No IMU preintegration factor, no per-tag projection factor, no field-landmark factor, no robust kernel (Cauchy/Huber) anywhere. Every noise model is plain Gaussian.

### 3.3 Update strategy

`FusionBackend::update()` (`FusionService.cpp:210`) drives `gtsam::ISAM2` with `relinearizeThreshold = 0.01`, `relinearizeSkip = 1` (`:82-84`) — relinearize whenever the relative error change exceeds 1%, every iteration. Two `update()` calls per measurement (one with the new factors, one empty to push relinearization), then `calculateEstimate()` to refresh `current_pose_`.

This is fine for the current graph size but makes the cost of every update grow with graph length; there is no fixed-lag smoother windowing today, so the graph grows for the entire match. See §7.

### 3.4 Output construction

`makeEstimate()` (`FusionService.cpp:238`) builds the `FusedPoseEstimate`:

- `timestamp` = the triggering measurement's timestamp.
- `field_to_robot` = `Pose2d{x, y, yaw}` extracted from the current `Pose3` via `toPose2d()` (`:49`). **Pitch, roll, and Z are dropped on the way out** even though they live in the graph.
- `covariance` = `isam_.marginalCovariance(current_key)` flattened row-major (gtsam tangent order `[rx, ry, rz, tx, ty, tz]`). On exception, set to zeros and `kFusionStatusMarginalUnavailable` is OR'd in.
- `status_flags` = a bitfield (`FusionService.h:24-30`): `Initializing`, `VisionUnavailable`, `MarginalUnavailable`, `OptimizerError`, `DegradedInput`, `ShockInflated`, `ChassisGap`.
- `velocity` is **never populated**. The struct field exists in `MeasurementTypes.h:117` but no code path writes it.

### 3.5 Out-of-order and gap handling

`acceptTimestamp()` (`FusionService.cpp:392`) rejects any measurement older than the last accepted one (counted into `stats_.stale_measurements`) — except IMU samples, which bypass the check entirely (`:356-363`). This is intentional: 1 kHz IMU is allowed to interleave with 100 Hz chassis; it never advances the pose cursor.

`addChassisSpeeds()` (`FusionService.cpp:127-133`) skips the `BetweenFactor` if `Δt <= 0` or `Δt > max_chassis_dt_seconds` (default 0.5 s) and emits the existing pose with `kFusionStatusChassisGap` set. Without this guard, a 5 s gap would compound a 5 m/s × 5 s = 25 m relative-motion factor with the wrong noise model and likely diverge the graph.

---

## 4. IMU integration: shock detection only

This is the biggest gap between the requirements and the implementation, so it gets its own section.

The IMU does not contribute to the state estimate. It populates a sliding window of `ImuShockSample` (timestamp + `|a|` + `|a − g_local|`) for the most recent `imu_window_seconds` (default 50 ms). On every chassis update, `detectShockOrFreefall()` (`FusionService.cpp:275`) scans the window:

- If any sample has `|a − g_local| > shock_threshold_mps2` (default 50 m/s² ≈ 5g), set the shock flag.
- If any sample has `|a| < freefall_threshold_mps2` (default 3 m/s²), set the shock flag.

If either branch fires, every diagonal sigma on the chassis `BetweenFactor` is multiplied by `shock_inflation_factor` (default 100×) for that one step, and the estimate is flagged with `kFusionStatusShockInflated`.

This catches collisions and the "wheels off the ground over a bump" case. It does **not** catch wheel slip — slip can happen with the chassis on the ground and the IMU reading nominal accelerations. See §7.

---

## 5. Field tag layout: not used in fusion

The field layout (`AprilTagFieldLayout`) is loaded into `RuntimeConfig::field_layouts` and passed into each `AprilTagPipeline` so the pipeline can build the world-corner array for the multi-tag SQPNP solve (see [`apriltag-pipeline.md`](apriltag-pipeline.md)). By the time the observation reaches fusion, it has already been collapsed to a 6-DOF `field_to_robot` pose plus a 6×6 covariance; the per-tag detections and the field map itself are not visible to the fusion graph.

This is a deliberate split (it keeps the heavy PnP solve in the camera-bound pipeline thread and away from the graph), but it forecloses a class of refinements: the graph can never decide "tag 7 looks like a flip; downweight it" or "tag 12's reprojection is suspiciously high; gate it out". Whatever the AprilTag pipeline outputs is what fusion eats.

This is fine for now but should be revisited if the per-tag detail in `AprilTagObservation::detections` would let fusion catch flip ambiguities the pipeline missed.

---

## 6. Configuration: hard-coded today

Every parameter named in §3 lives on the `FusionConfig` struct (`FusionService.h:32-56`) with compile-time defaults. The factory function:

```cpp
FusionConfig buildFusionConfig(const runtime::RuntimeConfig& /*runtime_config*/) {
    return FusionConfig{};
}
```

— `FusionService.cpp:299` — explicitly ignores its input. There is **no** `FusionConfig` struct on `RuntimeConfig`, **no** `fusion_*` table or column in the SQLite schema (`include/posest/config/SqliteSchema.h`), and **no** validator entries in `posest_config`. To change a fusion parameter today, edit the C++ defaults and recompile.

This is the largest blocker for the website-driven configurability required by the project bar. The full chain — JSON shape → `RuntimeConfig::FusionConfig` → SQLite migration + columns → `SqliteConfigStore` load/save → `ConfigValidator` bounds → `buildFusionConfig` mapping → live reload via the existing config-reload path used elsewhere in the daemon — is unimplemented end-to-end. The HTTP server doesn't exist either, but the data layer should land first so it's ready when the web layer arrives.

---

## 7. Output to the RoboRIO over CAN

The output path is fully wired and working:

1. **`FusionService::publishEstimate`** (`FusionService.cpp:403`) hands the `FusedPoseEstimate` to every registered sink under no lock.
2. **`TeensyService::publish`** (`TeensyService.cpp:100`) builds a `MessageType::FusedPose` USB frame. The payload is 28 bytes: `x` (double), `y` (double), `θ` (double), `status_flags` (u32) — see `encodeFusedPosePayload` (`:668`).
3. **Teensy firmware** (`firmware/teensy41/src/main.cpp:326`) decodes the frame and calls `CanBridge::setLatestFusedPose`, which caches the latest `FusedPosePayload`.
4. **`CanBridge::maybeSendFusedPose`** (`firmware/teensy41/src/CanBridge.cpp:248`) is called on every poll; it transmits at `1'000'000 / pose_publish_hz` µs (default 10 ms / 100 Hz) on CAN-FD ID `teensy_pose_can_id`.
5. **CAN frame layout** (`writeFusedPoseFrame`, `:28`): `teensy_time_us` (u64), `x` (double), `y` (double), `θ` (double), `status_flags` (u32), zero-padded to `kTeensyPosePayloadBytes`.

Two limitations of the wire format:

- **Pose2d only.** Z, pitch, roll never leave the host, even though they are tracked in the graph and present in the local `gtsam::Pose3`. For a grounded robot this is mostly fine, but it makes pitch/roll unobservable to the RIO if it ever wants them (e.g. for shooter aim corrections on a tilted bumper).
- **No covariance on the CAN frame.** The 6×6 covariance is computed on every publish but not transmitted. The RIO sees only `status_flags` to know "something is degraded" but cannot tell how much.

---

## 8. What's missing (gap list)

Ranked roughly by impact on the §1 requirements, not by implementation effort:

### 8.1 IMU is shock-only, not state-propagating

The IMU contributes nothing to between-vision-update propagation. Between two chassis samples (≈10 ms), the graph relies entirely on `vx·Δt, vy·Δt, ω·Δt` from the RIO — which is the same wheel odometry that's wrong during slip.

**What's needed:** an `ImuFactor` (GTSAM preintegration) integrating gyro and accelerometer between every pair of pose keys, with bias states (`gtsam::imuBias::ConstantBias`). This requires adding `Vector3` velocity and bias variables to the graph and an initial bias prior, plus a calibration step to estimate the gravity vector and bias at startup. See `gtsam/navigation/ImuFactor.h`.

### 8.2 No wheel-slip detection independent of the IMU

Slip is the named requirement. Today the only mitigation is shock-driven sigma inflation, which catches "the chassis hit a wall" but not "we floored it on a polished floor and the wheels spun."

**What's needed (any one of, or in combination):**
- **Cauchy/Huber kernel on the chassis BetweenFactor.** This is one line — wrap the noise model in `gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(k), chassis_noise_)`. Cheap, low risk, mostly defensive against unmodeled outliers.
- **Cross-check chassis vs. IMU.** With preintegration in place (§8.1), the IMU-implied velocity change vs. the wheel-implied velocity change is observable. If they disagree by more than a slip threshold over a short window, inflate the chassis sigmas (or drop the factor) for that step.
- **`ChassisSpeedsSample::status_flags`.** The struct already carries flags from the RIO; if the RIO surfaces a slip indicator, fusion should already be inflating on it. Today only "non-zero" maps to `kFusionStatusDegradedInput` without changing the noise model.

### 8.3 Configuration not persisted

The full chain described in §6 needs to exist:
1. Add `FusionConfig` (or a serializable mirror) to `RuntimeConfig`.
2. Add a `fusion_config` table or `fusion_*` columns plus a migration in `SqliteSchema.cpp`.
3. Implement load/save in `SqliteConfigStore.cpp` matching the existing pattern.
4. Add bounds checks to `ConfigValidator.cpp`.
5. Make `buildFusionConfig(RuntimeConfig)` actually map fields.
6. Decide on hot-reload semantics: rebuild the graph on change, or only apply to new factors? (The latter is safer; the former is simpler.)

The HTTP layer can land separately; once SQLite + `RuntimeConfig` know about it, the web layer is a thin REST/WebSocket on top.

### 8.4 Field tag layout not used as landmarks

If the graph held landmark variables for each known tag and consumed individual `AprilTagDetection` entries (currently nested in `AprilTagObservation::detections` but unused by fusion), it could:
- Detect per-tag flip ambiguities the pipeline missed.
- Reject single-tag flips by comparing against the known tag pose.
- Improve the rotation observability when the pipeline-aggregated pose has an inflated rotational covariance.

This is a larger change — it splits the PnP work between the pipeline and the graph — and may not pay off until single-camera flips become a measured problem. Defer until the simpler items above are done, but document the option here so it's not forgotten.

### 8.5 Output collapses to Pose2d; no covariance on CAN

For a grounded robot the 2D collapse is defensible, but it should be a deliberate decision tracked in the wire format, not an accident of `toPose2d()`. Two options:
- Keep `Pose2d` on CAN but expose the full `Pose3` + 6×6 covariance via a future telemetry sink for diagnostics.
- Extend the `kTeensyPosePayloadBytes` format to carry pitch/roll/Z plus a compact covariance (e.g. 3-DOF diag + correlation, fitting in a single CAN-FD frame).

Either way, the covariance on the CAN frame is the higher-value addition: today the RIO sees only a status bitfield and has no quantitative trust signal.

### 8.6 Velocity field never populated

`FusedPoseEstimate::velocity` exists and is consumed downstream (no current consumer reads it, but the type contract is "may be populated"). Once IMU preintegration adds a velocity variable to the graph (§8.1), populate it in `makeEstimate()`.

### 8.7 Graph grows unboundedly

`ISAM2` is incremental, but every chassis sample adds a key — at 100 Hz over a 2:30 match that's ~15k poses. Marginalization or a fixed-lag smoother (`gtsam::IncrementalFixedLagSmoother`) would keep the working set bounded and make `marginalCovariance(current_key)` cheap. Not a problem today (a 2:30 run hasn't been profiled), but flag for measurement once latency budgeting (§8.8) is in place.

### 8.8 No latency instrumentation

There is no histogram, no p99, no end-to-end timestamp on the path described in §2.2. `FusionStats` (`FusionService.h:18`) has only `measurements_processed`, `stale_measurements`, and `last_measurement_time`. Add per-stage timers (bus-pop → graph-update → publish, plus the host→Teensy→CAN slice) and surface them through `DaemonHealth` so latency regressions are visible.

### 8.9 VioMeasurement is silently dropped

`isSupportedMeasurement` (`FusionService.cpp:387`) excludes `VioMeasurement` without a counter or log. When VIO is implemented, this filter changes from "drop" to a real handler — but there's no test today proving the rest of the pipeline can ferry a `VioMeasurement` through the bus and into fusion's process function. Acceptable as a future-work flag; revisit when VIO is in scope.

---

## 9. Tests

`test/test_fusion_service.cpp` (320 lines, 11 tests) covers the implemented behavior:

- Chassis integration produces the expected pose displacement.
- An `AprilTagObservation` becomes the bootstrap prior.
- Non-positive-definite vision covariance is rejected and flagged.
- A subsequent empty-tag observation flags `VisionUnavailable` without resetting the pose.
- `VioMeasurement` is silently dropped (locks in current behavior — see §8.9).
- Out-of-order chassis samples are counted as stale.
- IMU shock (>50 m/s² departure from gravity) inflates step noise and flags it.
- IMU free-fall (<3 m/s² absolute) inflates step noise and flags it.
- Normal driving (~1.2 m/s² lateral) does not trigger the shock branch.
- A `Δt > max_chassis_dt_seconds` gap skips the BetweenFactor and flags `ChassisGap`.
- `buildFusionConfig` ignores its `RuntimeConfig` argument (locks in current behavior — see §8.3).

The "locks in current behavior" tests are the ones that need to flip when the corresponding gap is fixed. Notably absent: tests for the sink fan-out path, multi-sink scenarios, marginal-covariance failure recovery, and any latency-budget assertions.

---

## 10. Quick-reference: files

- **Service:** `include/posest/fusion/FusionService.h`, `src/fusion/FusionService.cpp`
- **Sink interface:** `include/posest/fusion/IFusionOutputSink.h`
- **Output type:** `include/posest/MeasurementTypes.h:117` (`FusedPoseEstimate`)
- **Wire format (host side):** `src/teensy/TeensyService.cpp:100,668`, `include/posest/teensy/Protocol.h`
- **Wire format (firmware side):** `firmware/teensy41/include/Protocol.h:44,157`, `firmware/teensy41/src/Protocol.cpp:188`
- **CAN bridge:** `firmware/teensy41/src/CanBridge.cpp:28,248`, `firmware/teensy41/include/CanBridge.h`
- **Daemon wiring:** `src/runtime/Daemon.cpp:818-826,861-863,904`
- **Tests:** `test/test_fusion_service.cpp`
