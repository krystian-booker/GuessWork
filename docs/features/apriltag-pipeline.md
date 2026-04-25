# AprilTag Pipeline

**Status:** Partially implemented. The pipeline is wired end-to-end as a `VisionPipelineBase` consumer that runs libapriltag's `tag36h11` detector on each frame, publishes `AprilTagObservation` messages to the `MeasurementBus`, and is consumed by `FusionService`. The core detector parameters are already configurable from the database via `PipelineConfig::parameters_json`. The remaining gaps are all on the **pose-estimation side**: the pipeline still relies on libapriltag's per-tag homography solver (no multi-tag SQPNP, no LM refinement, no distortion model, no aggregated robot pose), it produces no per-measurement covariance, and `FusionService` therefore applies a static vision noise model. None of the configurability work depends on the (not-yet-built) web server — the parsing path already accepts arbitrary JSON parameters from SQLite.

This document covers the AprilTag detection consumer end-to-end: how a frame arrives, what gets computed, what gets published, how the database drives configuration, and what is missing to be feature-complete against the stated requirements (low-latency, independent-per-camera, multi-tag SQPNP + LM, dynamic covariance, web-configurable).

The plumbing-level architecture (threading, mailbox, fan-out) is *not* repeated here — see [`producer-consumer-architecture.md`](producer-consumer-architecture.md). This document only talks about what `AprilTagPipeline` does *inside* `process()`.

---

## 1. Requirements (target behavior)

The bar this implementation is reviewed against:

1. **Lowest-latency detection per camera.** Each camera owns one independent AprilTag consumer. Detection runs as soon as a fresh frame is available; nothing else (other cameras, other consumers) can stall it.
2. **Per-pipeline configuration loaded from the database.** Each enabled AprilTag pipeline reads its configuration from SQLite at startup. No file-based or hard-coded settings on the production path.
3. **Detector tuning surfaced and configurable from the website.** Even though the website/server isn't implemented yet, the pipeline must already accept the same parameters via the DB so the future web layer is a UI/HTTP problem only:

   | Parameter | Target value (high-precision, hardware-triggered) | Why |
   |-----------|---------------------------------------------------|-----|
   | `quad_decimate` | 1.0 or 1.5 | Lower decimation → higher-resolution corner detection → sub-pixel accuracy. Only raise to 2.0 if the host CPU is choking. |
   | `quad_sigma` | 0.0 | Hardware trigger + 300 µs exposure already produces crisp edges; software blur destroys gradient quality. |
   | `refine_edges` | true (1) | Fits a continuous line to pixel gradients instead of snapping to integer pixels. Drastically reduces pose jitter. |
   | `decode_sharpening` | 0.25 | Default. Recovers contrast on slightly dark exposures. |
   | `nthreads` | 3 or 4 | Dedicated cores for detection so the per-camera pipeline never drops a frame at 30 Hz. |

4. **Multi-tag PnP, not per-tag averaging.** When multiple tags are visible in one frame, the pipeline must aggregate **all** their 2D corners and solve for a single camera (and ultimately robot) pose using the OpenCV `SOLVEPNP_SQPNP` solver, then refine with `solvePnPRefineLM`. Per-tag pose averaging is mathematically flawed and does not eliminate flip ambiguity.
5. **Camera intrinsics and distortion** must come from the active calibration row in the database (fx, fy, cx, cy, distortion model + coefficients). Distortion-aware undistortion must run on the corner pixels before PnP.
6. **Camera-to-robot extrinsic** must be applied inside the pipeline so the value handed downstream is the **robot's** global pose, not the camera's.
7. **Dynamic 6×6 covariance per measurement.** The pipeline must compute a measurement covariance that scales with:
   - tag distance (Z² scaling — error grows quadratically with range),
   - tag count (heavier rotational covariance when only one tag is visible; tighter when ≥ 2),
   - RMS reprojection error from `solvePnPRefineLM`.
8. **Output payload to fusion** must contain: capture timestamp (steady_clock), `gtsam::Pose3`-equivalent field-to-robot pose, and the 6×6 covariance, attached to the current pose node as a `PriorFactor`.

---

## 2. Architecture overview

### 2.1 Where the pipeline sits

```
  V4L2Producer ── deliver() ──▶ LatestFrameSlot ──▶ AprilTagPipeline::process() ──▶ MeasurementBus
   (capture thread,                (drop-oldest,        (worker thread,                (typed pub/sub)
    per-camera)                     per-consumer)        per-pipeline)                       │
                                                                                              ▼
                                                                                       FusionService
                                                                                       (GTSAM / ISAM2)
```

`AprilTagPipeline` (`src/pipelines/AprilTagPipeline.cpp:159`) is a `VisionPipelineBase`, which is a `ConsumerBase` that also holds an `IMeasurementSink&`. The threading model, drop-oldest mailbox, and 1-to-many fan-out are inherited unchanged — see [`producer-consumer-architecture.md`](producer-consumer-architecture.md) §2 for the contract. What's specific to this document:

- **Each camera that has an AprilTag pipeline binding gets its own pipeline instance.** Wiring lives in `RuntimeGraph::build()`, which walks `RuntimeConfig::bindings` and calls `camera->addConsumer(pipeline)`. Two cameras → two pipelines → two worker threads. They share no state except the `MeasurementBus` they publish to (which is itself thread-safe).
- **The pipeline's `process()` is allowed to be slow.** If detection takes 25 ms, the producer keeps capturing at 60 FPS regardless; the pipeline simply observes sequence gaps because `LatestFrameSlot` drops stale frames in front of it. This is what makes the latency contract work without backpressure.

### 2.2 The pipeline's `process()` today

`AprilTagPipeline::processFrame` (`src/pipelines/AprilTagPipeline.cpp:191`):

1. Convert the frame's `cv::Mat` to 8-bit grayscale. Already-grayscale frames pass through; BGR/BGRA are converted with `cvtColor`. Anything else throws.
2. Wrap the gray buffer as an `image_u8_t` (zero-copy, just header fields pointing at `cv::Mat::data`).
3. Call `apriltag_detector_detect(detector_, &image)`.
4. For each detection, populate an `AprilTagDetection`:
   - `tag_id` from `detection->id`,
   - 4 image corners from `detection->p`,
   - if a calibration entry exists for the camera, call libapriltag's `estimate_tag_pose()` to fill `camera_to_tag` (a `Pose3d`) and the per-tag reprojection error,
   - `ambiguity` is currently set to `detection->hamming` (this is **wrong** — see §6.4),
   - `reprojection_error_px` is whatever `estimate_tag_pose` returned for that single tag.
5. Stuff all detections into an `AprilTagObservation` along with `camera_id`, `frame.sequence`, and `frame.capture_time`, and `publish()` it on the bus.

The detector is constructed once in the constructor with the parsed configuration applied directly to the libapriltag struct fields:

```cpp
detector_->nthreads          = config_.nthreads;
detector_->quad_decimate     = static_cast<float>(config_.quad_decimate);
detector_->quad_sigma        = static_cast<float>(config_.quad_sigma);
detector_->refine_edges      = config_.refine_edges;
detector_->decode_sharpening = config_.decode_sharpening;
detector_->debug             = config_.debug;
apriltag_detector_add_family(detector_, family_);
```

`tag36h11` is the only family wired up. Other libapriltag families (`tag25h9`, `tagStandard41h12`, etc.) are explicitly rejected by `parseAprilTagPipelineConfig` (`src/pipelines/AprilTagPipeline.cpp:137`).

### 2.3 What happens downstream

`FusionService` (`src/fusion/FusionService.cpp`) subscribes to the bus and consumes `AprilTagObservation` measurements. For each detection that has a `camera_to_tag` pose, it computes:

```
field_to_robot = field_to_tag · camera_to_tag⁻¹ · camera_to_robot
```

…and adds **one `gtsam::PriorFactor<Pose3>` per detection** to ISAM2, all keyed at the current pose. The noise model is the static `vision_sigmas` array on `FusionConfig`. There is no aggregation across detections in the pipeline — each tag is a separate prior. This is exactly the "average per-tag" anti-pattern that requirement (4) calls out, just expressed as N priors rather than an arithmetic mean. With ISAM2 the optimization will find a least-squares fit, but that fit is fundamentally constrained by the same per-tag flip ambiguity problem because each per-tag pose was produced independently from a 4-corner homography.

---

## 3. Configuration model (database-backed)

### 3.1 What's persisted

`PipelineConfig` (`include/posest/runtime/PipelineConfig.h`) stores:

- `id` — pipeline instance id (one per camera binding),
- `type` — must be `"apriltag"` for this pipeline,
- `enabled`,
- `parameters_json` — free-form JSON object holding the AprilTag-specific parameters.

The `parameters_json` object is parsed by `parseAprilTagPipelineConfig` (`src/pipelines/AprilTagPipeline.cpp:112`) and accepts the following keys (all optional; defaults shown):

| Key | Default | Validation |
|-----|---------|------------|
| `family` | `"tag36h11"` | Only `tag36h11` is accepted today. |
| `nthreads` | `1` | Must be `> 0`. |
| `quad_decimate` | `2.0` | Must be `> 0`. |
| `quad_sigma` | `0.0` | Must be `>= 0`. |
| `refine_edges` | `true` | — |
| `decode_sharpening` | `0.25` | Must be `>= 0`. |
| `debug` | `false` | — |
| `calibration_version` | `""` | If set, only that version of the camera's calibration is selected. |
| `field_layout_id` | `""` | Stored but **not currently read by the pipeline** (only `FusionService` reads `active_field_layout_id`). |
| `tag_size_m` | `0.1651` | Must be `> 0`. WPILib FRC tag size by default. |

Camera intrinsics are **not** in `parameters_json`. They live in their own `calibrations` table (`CameraCalibrationConfig`), keyed by `camera_id` + `version`. `ProductionPipelineFactory::createPipeline` (`src/runtime/ProductionFactories.cpp:68`) takes the active calibration row(s) for each camera and injects them into `AprilTagPipelineConfig::camera_calibrations` before constructing the pipeline. If a `calibration_version` is given in `parameters_json`, only that version is selected; otherwise every active row is taken.

The persistence path is:

```
sqlite (calibrations + pipelines tables)
    │
    ▼
SqliteConfigStore::loadRuntimeConfig()  →  RuntimeConfig
    │                                          │
    │                                          ▼
    │                          ProductionPipelineFactory::createPipeline
    │                                          │
    │                                          ▼
    │                        parseAprilTagPipelineConfig + applyCameraCalibrationContext
    │                                          │
    └──────────────────────────────────────────▶ AprilTagPipeline (constructed with applied config)
```

### 3.2 Independence per camera

Each `AprilTagPipeline` is constructed with its own `AprilTagPipelineConfig`. Two cameras can run with completely different `quad_decimate` / `nthreads` / `tag_size_m` values just by having two different `pipelines` rows in the database. There is no shared detector, no shared mutable state.

### 3.3 What happens when settings change

Currently: **nothing live**. The detector struct is initialized once in the `AprilTagPipeline` constructor and never rewritten. To pick up a settings change, the runtime graph has to be torn down and rebuilt (`RuntimeGraph::stop()` → re-`build()` → `start()`). This is acceptable for the V1 web flow ("save settings → restart pipeline"), but a future "tweak knobs while running" UX would need either a `reconfigure()` method on the pipeline or a destroy-and-replace path that doesn't restart the camera. Not yet built.

---

## 4. What's implemented vs. what's missing

### 4.1 Detector tuning — partially aligned

What works:

- Every parameter from the requirement table is reachable from the database via `parameters_json`.
- The validation layer rejects nonsense values (negative numbers, zero, unknown families) at parse time.
- Per-pipeline `nthreads` is honored and isolates CPU usage to that camera's pipeline.

What's wrong / missing:

- **Defaults are not aligned with the high-precision target.** `quad_decimate` defaults to `2.0`; the target is `1.0` or `1.5`. `nthreads` defaults to `1`; the target is `3` or `4`. These are *configurable*, so they're not blockers, but the out-of-the-box defaults will produce visibly worse pose accuracy than the target. The fix is one-line per parameter in `AprilTagPipelineConfig`.
- **No live reconfigure** (see §3.3).
- **Only `tag36h11`** is wired up. Adding more families is mechanical (libapriltag has `tag25h9_create()`, `tagStandard41h12_create()`, etc.) but not done.

### 4.2 Multi-tag PnP — not implemented

This is the largest gap. The current path is:

1. libapriltag detects each tag and reports its 4 image corners.
2. The pipeline calls libapriltag's `estimate_tag_pose()` **once per detection**, which internally solves a 4-point P3P/IPPE-square problem from the homography. This is single-tag-only by construction.
3. Each per-tag pose flows independently to `FusionService` as a separate `PriorFactor`.

What the requirement calls for:

1. Collect all 2D corners across **all** detections in the frame: 3 tags → 12 `(u, v)` points.
2. Look up the corresponding 3D world coordinates from the active field layout. Each tag's `field_to_tag` pose plus the known tag size gives the 4 corner positions in field coordinates; concatenate across tags.
3. **Undistort** the 2D corners using the active calibration's distortion model + coefficients.
4. `cv::solvePnP(objectPoints, imagePoints, K, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_SQPNP)` to get an initial camera pose. SQPNP is fast, robust on coplanar inputs (common when tags share a wall), and resolves the flip ambiguity that plagues single-tag P3P.
5. `cv::solvePnPRefineLM(objectPoints, imagePoints, K, distCoeffs, rvec, tvec)` to drive the reprojection error to its mathematical minimum.
6. Convert (rvec, tvec) → `field_to_camera` (or `camera_to_field`, depending on point convention), then apply `camera_to_robot` to get **`field_to_robot`**.
7. Compute the RMS reprojection residual from the refined solve (cv returns it; otherwise project + compare).

Single-tag frames still need a path. Two reasonable options:

- **(A)** Run the same SQPNP + LM with just 4 points. Works, but the pose's rotational uncertainty around the tag's optical axis is large — that uncertainty must be reflected in the covariance (§4.5).
- **(B)** Fall back to libapriltag's `estimate_tag_pose_orthogonal_iteration()`, which returns *both* pose hypotheses and their reprojection errors. The ratio `err_A / err_B` is the standard "tag pose ambiguity" metric — feed it into the covariance as a confidence weight, and only emit the pose if the ratio is below a threshold.

(A) is simpler and uniform; (B) is more conservative on pathological single-tag-far-away cases. Both are reasonable; pick one and document the choice.

### 4.3 Distortion model — not applied

`CameraCalibrationConfig` already carries `distortion_model` (e.g. `radtan`, `equidistant`) and `distortion_coefficients`, parsed by `parseKalibrYaml` and persisted by `SqliteConfigStore`. The AprilTag pipeline ignores both. With the current libapriltag-only path this is silently OK because `estimate_tag_pose` operates on raw pixel corners and does not attempt to undistort them — it just bakes the distortion error into the homography solve.

Once SQPNP enters the picture, distortion coefficients **must** be passed to `solvePnP` (or the corners must be undistorted via `cv::undistortPoints` first), otherwise the refinement converges on the wrong pose for any non-pinhole lens.

The data is already in `RuntimeConfig`; the wiring is the missing piece. `applyCameraCalibrationContext` (`src/runtime/ProductionFactories.cpp:18`) currently copies only `fx/fy/cx/cy` into the pipeline config — extend it to also copy `distortion_model` + `distortion_coefficients`, and extend `AprilTagCameraCalibration` with those fields.

### 4.4 Camera-to-robot extrinsic — applied in the wrong layer

The data exists: `RuntimeConfig::camera_extrinsics` (`CameraExtrinsicsConfig::camera_to_robot`), persisted in the `camera_extrinsics` table, populated from Kalibr camera-IMU calibration. `FusionService` reads it (`buildFusionConfig` walks the active calibrations and copies `camera_to_robot` into `FusionConfig::camera_to_robot`).

But the requirement is for the **AprilTag pipeline** to publish the **robot's** field pose, not the camera's. The pipeline therefore needs:

- The active `camera_to_robot` extrinsic injected at construction time (mirroring how intrinsics are injected today). `applyCameraCalibrationContext` is the natural place to do it.
- After SQPNP + LM produces `field_to_camera`, multiply by `camera_to_robot` and emit `field_to_robot` in the published observation.

This also simplifies fusion: `FusionService` would just take the published robot pose at face value instead of re-deriving it from per-tag camera poses.

### 4.5 Dynamic covariance — not implemented anywhere

Today the noise model handed to GTSAM is the static `FusionConfig::vision_sigmas` array, default `{0.15 m, 0.15 m, 0.12 m, 0.10 rad, 0.10 rad, 0.20 rad}`. This is identical for one blurry tag at 5 m and four crisp tags at 1 m. At 5 m/s the difference between those two cases is the difference between trusting vision and trusting the IMU.

The requirement is for the pipeline to compute, per frame, a **6×6 covariance matrix** that scales with:

- **Distance (Z).** Reprojection error in pixels translates to Z² metric error (textbook pinhole math). Use the mean detected-tag depth as the scale factor.
- **Tag count.** With 1 tag visible, rotational uncertainty around the optical axis is poorly constrained — inflate the rotational diagonal heavily. With ≥ 2 tags, both translation and rotation are well constrained — keep the covariance tight. With ≥ 4 well-spread tags, it can shrink further still.
- **RMS reprojection error.** `solvePnPRefineLM` returns the residual; high RMS means the corners didn't agree on a coherent pose (motion blur, bad calibration, partial occlusion) — inflate proportionally.

Plumbing changes required:

1. Extend `AprilTagObservation` with a `field_to_robot` pose (optional, set when the multi-tag solve succeeded) and a `std::array<double, 36> covariance`.
2. Compute both inside `AprilTagPipeline::processFrame` after the LM refinement.
3. In `FusionService::addAprilTags`, when the observation carries a `field_to_robot` + covariance, build a per-measurement `gtsam::noiseModel::Gaussian::Covariance(...)` and attach **one** `PriorFactor<Pose3>` (not N — the multi-tag solve already aggregated them). The existing per-detection-prior path becomes the fallback for legacy/single-tag-only outputs.

This change is what makes vision automatically fall back to IMU during high-speed motion blur or distant-tag viewing — the fusion layer just respects the covariance, no special-cased "trust vision unless …" logic.

### 4.6 Timestamp and time domain

`Frame::capture_time` is in `steady_clock` (V4L2 buffer timestamp converted to monotonic, see [`producer-consumer-architecture.md`](producer-consumer-architecture.md) §2.5). The pipeline propagates it unchanged into `AprilTagObservation::capture_time`, and `FusionService` uses it to time-order measurements. This is correct for the host-only fusion case.

What's open: the requirement mentions "the exact teensy time the shutter opened." `CameraTriggerEvent` already publishes Teensy-stamped trigger pulses on the bus (`include/posest/MeasurementTypes.h:81`), and `posest_teensy` populates `teensy_time_us` per pulse. Stitching the trigger event to the frame (and therefore to the AprilTag observation) at sub-millisecond precision is a separate piece of work; today the host steady_clock timestamp is the only time field on the observation.

### 4.7 Latency posture

The pipeline already meets the "lowest latency per camera" rule, with two caveats:

- **Grayscale conversion is on the pipeline thread.** If the V4L2 producer is configured with a Y8/Y16/GREY pixel format, `toGray8` is a no-op. If it's MJPEG or BGR, `cvtColor` runs once per frame on the pipeline worker (microseconds at typical resolutions, but it's there). For a fully optimized path, configure the camera to emit grayscale natively when AprilTag is the only consumer.
- **`cv::Mat::isContinuous()` cloning.** The pipeline `clone()`s the gray Mat if it isn't already continuous in memory. For V4L2 MMAP-backed Mats this is normally already continuous, so this branch shouldn't fire in production. Worth keeping the branch — safer than a libapriltag scan over a non-continuous buffer — but worth being aware of if profiling shows surprise allocations.

The blocking pieces (libapriltag detection + future SQPNP + LM) all run on the pipeline's own worker thread. None of it can stall the V4L2 producer thread.

### 4.8 Web configurability

The requirement is "configurable from the website but we haven't implemented the server yet." Status:

- ✅ The pipeline already reads its parameters from SQLite at startup.
- ✅ `parameters_json` is a free-form JSON object — no schema migration is needed to add new keys.
- ✅ `SqliteConfigStore::saveRuntimeConfig` is atomic and validated, so a future POST-from-website handler just hands a `RuntimeConfig` to `save()` and is done.
- ❌ `WebService` (`include/posest/runtime/WebService.h`) is currently a thin holder for telemetry state. It has no HTTP server, no route table, no settings handler. This is co-deferred with the video preview consumer (see [`producer-consumer-architecture.md`](producer-consumer-architecture.md) §6.2).
- ❌ No live reconfigure (§3.3). The web layer will need to either (a) trigger a graph rebuild on save, or (b) add a `reconfigure()` path on the pipeline.

---

## 5. Gap summary

| Requirement | Status | Notes |
|-------------|--------|-------|
| Per-camera independent AprilTag consumer | ✅ Complete | One `AprilTagPipeline` per binding, own thread, own mailbox, own config. |
| Lowest-latency frame delivery | ✅ Complete | Inherited from `ConsumerBase` + `LatestFrameSlot`. Verified by core pipeline tests. |
| DB-backed configuration per pipeline | ✅ Complete | `parameters_json` parsed by `parseAprilTagPipelineConfig`, calibrations injected by factory. |
| Detector tuning surface (decimate/sigma/refine/sharpen/threads) | ✅ Complete (defaults misaligned) | All 5 keys are reachable from JSON. Defaults still set to libapriltag conservative values, not the high-precision targets — see §4.1. |
| Tag family configurable | ⚠️ Partial | Only `tag36h11` is wired up; other families rejected. |
| Camera intrinsics from active DB calibration | ✅ Complete | `applyCameraCalibrationContext` injects `fx/fy/cx/cy` per camera. |
| Distortion model + coefficients applied before PnP | ❌ Missing | Data exists in `RuntimeConfig`; pipeline ignores it (§4.3). |
| Multi-tag SQPNP solve | ❌ Missing | Currently per-tag homography via libapriltag `estimate_tag_pose` (§4.2). |
| Levenberg-Marquardt refinement | ❌ Missing | (§4.2) |
| Field layout consumed by the pipeline | ❌ Missing | `field_layout_id` is parsed but not read; only `FusionService` reads field tags. (§4.2 step 2.) |
| Camera-to-robot extrinsic applied in pipeline | ❌ Missing | Currently applied in `FusionService`. Pipeline needs to publish robot pose, not camera-relative pose (§4.4). |
| Dynamic 6×6 covariance per measurement | ❌ Missing | `AprilTagObservation` has no covariance field; fusion uses static `vision_sigmas` (§4.5). |
| Single `PriorFactor` per frame (not per tag) | ❌ Missing | Fusion currently adds N priors, one per detection. Becomes one factor once the pipeline emits aggregated pose + covariance. |
| Pose ambiguity metric correctly populated | ❌ Wrong field | `AprilTagDetection::ambiguity` is set to libapriltag's *Hamming distance*, not pose ambiguity. (§6.4 below.) |
| Steady_clock timestamp on observation | ✅ Complete | Inherited from frame; same domain as IMU/wheel/Teensy. |
| Teensy-clock shutter timestamp on observation | ⚠️ Partial | `CameraTriggerEvent` carries `teensy_time_us`, but it isn't joined to the AprilTag observation today. |
| Web layer to drive configuration | ❌ Deferred | `WebService` has no HTTP server (§4.8). DB path is ready. |
| Live reconfigure of detector params | ❌ Missing | Detector built once at construction; settings change requires graph restart (§3.3). |

---

## 6. Suggested implementation order

If the goal is "ship the minimum viable accuracy upgrade first," a reasonable order:

1. **Fix the detector defaults** (`quad_decimate = 1.0` or `1.5`, `nthreads = 3`+) so the out-of-the-box behavior matches the target. One-line changes in `AprilTagPipelineConfig`. (§4.1)
2. **Plumb distortion coefficients** from `CameraCalibrationConfig` → `AprilTagCameraCalibration`. Required before any OpenCV PnP path is correct. (§4.3)
3. **Plumb the active field layout** into the pipeline (mirroring how intrinsics are injected). Resolve `field_layout_id` against `RuntimeConfig::field_layouts`. Required for multi-tag PnP. (§4.2)
4. **Plumb `camera_to_robot`** into the pipeline. Required to emit robot pose from the pipeline. (§4.4)
5. **Replace `estimate_tag_pose` with multi-tag SQPNP + LM.** Aggregate corners across detections, undistort, solve, refine, transform to robot frame, compute RMS. Emit `field_to_robot` on the observation. (§4.2)
6. **Compute and emit dynamic covariance.** Distance + tag-count + RMS scaling. Add covariance field to `AprilTagObservation`. (§4.5)
7. **Update `FusionService::addAprilTags`** to consume the aggregated pose + covariance as a single `PriorFactor` with a per-measurement noise model. Keep the per-tag fallback for backwards compatibility / single-tag-only frames if needed.
8. **Web layer** (separate work track, blocked on `WebService` HTTP).
9. **Live reconfigure** (separate work track, only useful with the web layer).

Steps 1–4 are pure plumbing and unblock step 5; step 5 unblocks step 6; steps 5–7 together are the "feature complete" line for the AprilTag → fusion path.

### 6.1–6.3 Notes for implementers

**6.1 SQPNP requires `≥ 3` correspondences.** With one tag (4 corners) it works; with zero detections, skip the solve and emit an empty observation. Don't degrade to the homography path for single-tag frames just to keep the code paths uniform — SQPNP handles 4-point input fine.

**6.2 Coplanar tags are common.** Tags mounted on a flat wall produce a coplanar 3D point set, which is the exact failure mode P3P-style solvers hit. SQPNP is specifically chosen because it handles coplanar inputs correctly. Don't substitute `SOLVEPNP_ITERATIVE` or `SOLVEPNP_EPNP` here.

**6.3 LM refinement is a hot path.** `cv::solvePnPRefineLM` will converge in 5–20 iterations for a good initial guess from SQPNP. Limit iterations and tolerance via `cv::TermCriteria` if profiling shows it eating the latency budget.

### 6.4 The `ambiguity` field bug

`AprilTagPipeline::processFrame` (`src/pipelines/AprilTagPipeline.cpp:230`) currently sets:

```cpp
out.ambiguity = static_cast<double>(detection->hamming);
```

`detection->hamming` is the number of bit errors corrected during AprilTag decoding — it's a *decoding* quality metric, not a *pose* quality metric. For a clean detection it's `0` regardless of how flippy the pose is.

The correct pose-ambiguity metric is the ratio of the two `estimate_tag_pose_orthogonal_iteration` reprojection errors (`err_A / err_B`). When that ratio is close to 1.0, both pose hypotheses fit the corners equally well and the detection is ambiguous; when it's small (e.g. `< 0.4`), one hypothesis dominates and the detection is reliable. Once the multi-tag SQPNP path lands, per-tag ambiguity stops mattering for the published pose, but the field is still useful for diagnostics and for single-tag fallback.

---

## 7. File reference

| File | Role |
|------|------|
| `include/posest/pipelines/AprilTagPipeline.h` | Class declaration + `AprilTagPipelineConfig` + `parseAprilTagPipelineConfig`. |
| `src/pipelines/AprilTagPipeline.cpp` | Detector construction, per-frame `processFrame`, libapriltag pose helper. |
| `include/posest/pipelines/VisionPipelineBase.h` | `ConsumerBase`-derived base; provides the worker thread + mailbox. |
| `include/posest/MeasurementTypes.h` | `AprilTagDetection`, `AprilTagObservation` payloads. |
| `include/posest/MeasurementBus.h` | Typed pub/sub the pipeline publishes onto. |
| `include/posest/runtime/PipelineConfig.h` | DB-row shape (`parameters_json` lives here). |
| `include/posest/runtime/RuntimeConfig.h` | `CameraCalibrationConfig`, `CameraExtrinsicsConfig`, `FieldLayoutConfig` (intrinsics, extrinsics, field tags). |
| `src/runtime/ProductionFactories.cpp` | `applyCameraCalibrationContext` — injects active calibrations into the pipeline config. |
| `src/fusion/FusionService.cpp` | Consumer of `AprilTagObservation`; per-tag `PriorFactor` path (current) + static noise model. |
| `src/config/SqliteConfigStore.cpp` | Persistence of pipelines, calibrations, extrinsics, field layouts. |
| `test/test_pipelines.cpp` | `AprilTagPipeline` config-parsing tests, blank-image test, rendered-tag detection test, intrinsics-injection test. |
| `test/test_fusion_service.cpp` | End-to-end fusion behavior (covers the per-tag prior path). |
