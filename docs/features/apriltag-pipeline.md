# AprilTag Pipeline

**Status:** Feature-complete on the pose-estimation path. The pipeline runs libapriltag's `tag36h11` detector, then hands the corner detections to `AprilTagPoseSolver`, which performs a multi-tag SQPNP + Levenberg–Marquardt solve (with distortion applied), or a single-tag fallback using libapriltag's orthogonal-iteration solver gated on the standard `err1/err2` pose-ambiguity ratio. Camera intrinsics, distortion coefficients, the active field layout, and the `camera_to_robot` extrinsic are all injected from SQLite at construction time, so the pipeline emits a `field_to_robot` pose plus a 6×6 covariance per frame. `FusionService` consumes that as a single `PriorFactor<Pose3>` per observation with a per-measurement Gaussian noise model. Detector defaults (`quad_decimate=1.0`, `nthreads=4`) are aligned with the high-precision target. Remaining gaps are now off the critical path: the web/HTTP layer is not yet implemented, live reconfigure requires a graph rebuild, only `tag36h11` is wired up, and the Teensy-stamped shutter timestamp is not yet joined to the AprilTag observation.

This document covers the AprilTag detection consumer end-to-end: how a frame arrives, what gets computed, what gets published, how the database drives configuration, and what is still open.

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

`AprilTagPipeline` (`src/pipelines/AprilTagPipeline.cpp`) is a `VisionPipelineBase`, which is a `ConsumerBase` that also holds an `IMeasurementSink&`. The threading model, drop-oldest mailbox, and 1-to-many fan-out are inherited unchanged — see [`producer-consumer-architecture.md`](producer-consumer-architecture.md) §2 for the contract. What's specific to this document:

- **Each camera that has an AprilTag pipeline binding gets its own pipeline instance.** Wiring lives in `RuntimeGraph::build()`, which walks `RuntimeConfig::bindings` and calls `camera->addConsumer(pipeline)`. Two cameras → two pipelines → two worker threads. They share no state except the `MeasurementBus` they publish to (which is itself thread-safe).
- **The pipeline's `process()` is allowed to be slow.** If detection takes 25 ms, the producer keeps capturing at 60 FPS regardless; the pipeline simply observes sequence gaps because `LatestFrameSlot` drops stale frames in front of it. This is what makes the latency contract work without backpressure.

### 2.2 The pipeline's `process()` today

`AprilTagPipeline::processFrame` (`src/pipelines/AprilTagPipeline.cpp:178`):

1. Convert the frame's `cv::Mat` to 8-bit grayscale. Already-grayscale frames pass through; BGR/BGRA are converted with `cvtColor`. Anything else throws.
2. Wrap the gray buffer as an `image_u8_t` (zero-copy, just header fields pointing at `cv::Mat::data` — with a `clone()` fallback if the Mat is non-continuous).
3. Call `apriltag_detector_detect(detector_, &image)`.
4. For each detection, populate an `AprilTagDetection` (tag id + 4 image corners). The per-tag `ambiguity` and `reprojection_error_px` fields stay at `0.0` here; ambiguity is only filled in by the single-tag fallback path of the solver.
5. Look up the active `AprilTagCameraCalibration` and `camera_to_robot` for `frame.camera_id`. If both are present, every detection that also has a known `field_to_tag` entry from the active field layout is appended to an `AprilTagPoseSolveInput`.
6. If at least one detection passed those filters, call `solveAprilTagPose(input)`. The solver returns either a `field_to_robot` pose + 6×6 covariance + RMS + solved-tag-count, or an empty result if the solve failed / a single-tag detection was dropped on ambiguity.
7. Stuff per-tag detections, the optional `field_to_robot`, the covariance, RMS, and `solved_tag_count` into an `AprilTagObservation`, and `publish()` it on the bus. The single-tag `ambiguity` ratio (when computed) is written back onto the originating detection for diagnostics.

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

`tag36h11` is the only family wired up. Other libapriltag families (`tag25h9`, `tagStandard41h12`, etc.) are explicitly rejected by `parseAprilTagPipelineConfig` (`src/pipelines/AprilTagPipeline.cpp:48`).

### 2.3 The pose solver

`AprilTagPoseSolver` (`src/pipelines/AprilTagPoseSolver.cpp`) owns all of the geometry. It is intentionally a free function so it can be unit-tested without a running pipeline. There are two paths:

**Multi-tag (`solveMultiTag`, ≥ 2 tags):**

1. Build the 4 tag-frame corner positions from `tag_size_m`, then transform each by the tag's `field_to_tag` to get **field-frame** 3D object points. Concatenate across all detections (3 tags → 12 points).
2. Build the intrinsic matrix `K` and distortion coefficients `D` from the active calibration. If the distortion model is `equidistant`/`fisheye`/`kannala_brandt`, undistort the corners up-front with `cv::fisheye::undistortPoints` and pass empty distortion to `solvePnP`. For `radtan` (or no model), pass `D` directly to `solvePnP` so the solver projects through the distortion model itself.
3. `cv::solvePnP(..., SOLVEPNP_SQPNP)` for the initial guess (handles coplanar tags, common when tags are wall-mounted).
4. `cv::solvePnPRefineLM(...)` to drive reprojection error to its minimum. Refinement failure falls back to the SQPNP result.
5. Project the 3D points back through `(rvec, tvec)` and compute the RMS reprojection residual.
6. `cam_T_field = (rvec, tvec)`; invert to get `field_T_camera`; compose with `camera_T_robot` to get `field_T_robot`.
7. Compute the mean detected-tag depth from the camera-frame Z of the object points and feed it, the RMS, and the tag count into `computeCovariance`.

**Single-tag (`solveSingleTag`, exactly 1 tag):**

1. Call libapriltag's `estimate_tag_pose_orthogonal_iteration` with a 50-iteration cap. This returns **two** pose hypotheses (`pose1`, `pose2`) and their reprojection errors (`err1`, `err2`).
2. Compute the standard tag-pose ambiguity ratio `err1 / err2` (clamped to `[0, 1]`). When `err2 ≈ 0` there is no alternative, so `ambiguity = 0`.
3. If `ambiguity > covariance.ambiguity_drop_threshold` (default 0.4), **drop the pose** — the observation is published with detections only, so fusion will not ingest a flippy single-tag pose. The ratio is still written back on the per-tag detection for diagnostics.
4. Otherwise convert `pose1` to `camera_to_tag`, compose `field_to_tag · (camera_to_tag)⁻¹ · camera_to_robot` to get `field_to_robot`, project the four corners back to compute the RMS, and compute the covariance using the per-tag depth.

The solver is bypassed entirely (returns empty) if no calibration is plumbed for the camera (`fx`/`fy` are zero). That makes context-less unit tests safe by construction.

### 2.4 What happens downstream

`FusionService::addAprilTags` (`src/fusion/FusionService.cpp:121`) now does the right thing:

1. If `observation.field_to_robot` is unset, it sets `kFusionStatusVisionUnavailable` and falls back to the current pose estimate — no factor added.
2. Otherwise it builds a `gtsam::Matrix6` from `observation.covariance`, wraps it in `gtsam::noiseModel::Gaussian::Covariance`, and adds **one** `PriorFactor<Pose3>` keyed at the current pose. If the noise model rejects the covariance (e.g. not positive-definite), the update fails gracefully with `kFusionStatusOptimizerError`.

This is the "single prior per frame" behavior the requirement called for. The per-tag prior path (and the static `vision_sigmas` it depended on) is gone.

---

## 3. Configuration model (database-backed)

### 3.1 What's persisted

`PipelineConfig` (`include/posest/runtime/PipelineConfig.h`) stores:

- `id` — pipeline instance id (one per camera binding),
- `type` — must be `"apriltag"` for this pipeline,
- `enabled`,
- `parameters_json` — free-form JSON object holding the AprilTag-specific parameters.

`parseAprilTagPipelineConfig` (`src/pipelines/AprilTagPipeline.cpp:48`) accepts the following keys (all optional; defaults shown):

| Key | Default | Validation |
|-----|---------|------------|
| `family` | `"tag36h11"` | Only `tag36h11` is accepted today. |
| `nthreads` | `4` | Must be `> 0`. |
| `quad_decimate` | `1.0` | Must be `> 0`. |
| `quad_sigma` | `0.0` | Must be `>= 0`. |
| `refine_edges` | `true` | — |
| `decode_sharpening` | `0.25` | Must be `>= 0`. |
| `debug` | `false` | — |
| `calibration_version` | `""` | If set, only that version of the camera's calibration (and matching extrinsic row) is selected. |
| `field_layout_id` | `""` | If empty, falls back to `RuntimeConfig::active_field_layout_id`. Now actively read by the factory. |
| `tag_size_m` | `0.1651` | Must be `> 0`. WPILib FRC tag size by default. |
| `covariance` | (object) | Object holding the covariance-tuning knobs below. |

Covariance tuning sub-object (`covariance.*`):

| Key | Default | Validation |
|-----|---------|------------|
| `base_sigma_translation_m` | `0.02` | Must be `> 0`. Reference translation σ at `reference_distance_m` and `reference_rms_px`. |
| `base_sigma_rotation_rad` | `0.02` | Must be `> 0`. Reference rotation σ at the same reference operating point. |
| `reference_distance_m` | `1.0` | Must be `> 0`. The "free" distance — beyond this, σ scales as Z²/Z_ref². |
| `reference_rms_px` | `1.0` | Must be `> 0`. RMS scaling reference — σ scales linearly with `rms / rms_ref` once over 1.0. |
| `single_tag_translation_mult` | `1.5` | Must be `> 0`. Multiplier applied to translation σ when only one tag was solved. |
| `single_tag_rotation_mult` | `5.0` | Must be `> 0`. Multiplier on rotation σ when only one tag was solved (the optical-axis ambiguity case). |
| `ambiguity_drop_threshold` | `0.4` | Must be in `(0, 1]`. Single-tag observations with `err1/err2 >` threshold are not emitted. |

Camera intrinsics, distortion, extrinsics, and field layout are **not** in `parameters_json`. They live in their own tables and are injected by the factory:

- `applyCameraCalibrationContext` (`src/runtime/ProductionFactories.cpp:18`) copies `fx/fy/cx/cy/distortion_model/distortion_coefficients` per camera from the active `calibrations` row.
- `applyCameraExtrinsicsContext` (`src/runtime/ProductionFactories.cpp:62`) copies `camera_to_robot` from `camera_extrinsics` whose `version` matches the active calibration version (or the pipeline's `calibration_version` filter, if set).
- `applyFieldLayoutContext` (`src/runtime/ProductionFactories.cpp:42`) copies all `field_to_tag` poses for the resolved layout id into `pipeline_config.field_to_tags`.

The persistence path is:

```
sqlite (calibrations + camera_extrinsics + field_layouts + pipelines)
    │
    ▼
SqliteConfigStore::loadRuntimeConfig()  →  RuntimeConfig
    │
    ▼
ProductionPipelineFactory::createPipeline
    │
    ▼
parseAprilTagPipelineConfig
  + applyCameraCalibrationContext
  + applyCameraExtrinsicsContext
  + applyFieldLayoutContext
    │
    ▼
AprilTagPipeline (constructed with applied config)
```

### 3.2 Independence per camera

Each `AprilTagPipeline` is constructed with its own `AprilTagPipelineConfig`. Two cameras can run with completely different `quad_decimate` / `nthreads` / `tag_size_m` / covariance tuning values just by having two different `pipelines` rows in the database. There is no shared detector, no shared mutable state.

### 3.3 What happens when settings change

Currently: **nothing live**. The detector struct, the calibration map, and the field layout map are all initialized once in the `AprilTagPipeline` constructor and never rewritten. To pick up a settings change, the runtime graph has to be torn down and rebuilt (`RuntimeGraph::stop()` → re-`build()` → `start()`). This is acceptable for the V1 web flow ("save settings → restart pipeline"), but a future "tweak knobs while running" UX would need either a `reconfigure()` method on the pipeline or a destroy-and-replace path that doesn't restart the camera. Not yet built.

---

## 4. What's implemented vs. what's missing

### 4.1 Detector tuning — complete

- Every parameter from the requirement table is reachable from the database via `parameters_json`.
- The validation layer rejects nonsense values (negative numbers, zero, unknown families) at parse time.
- Per-pipeline `nthreads` is honored and isolates CPU usage to that camera's pipeline.
- **Defaults are now aligned with the high-precision target** (`quad_decimate=1.0`, `nthreads=4`, `refine_edges=true`, `decode_sharpening=0.25`, `quad_sigma=0.0`).
- Only `tag36h11` is wired up. Other families are explicitly rejected. Adding more is mechanical (libapriltag has `tag25h9_create()`, `tagStandard41h12_create()`, etc.) but not done. Track as a deferred enhancement.

### 4.2 Multi-tag PnP — complete

`solveMultiTag` performs the full SQPNP + LM pipeline described in the requirements:

1. ✅ Aggregates all 2D corners across detections (3 tags → 12 points).
2. ✅ Looks up corresponding 3D world coordinates from `field_to_tags` (the active field layout) and the configured `tag_size_m`.
3. ✅ Distortion is honored. `radtan` coefficients are passed straight to `solvePnP`; `equidistant`/`fisheye`/`kannala_brandt` is undistorted up-front via `cv::fisheye::undistortPoints` and the solver runs on the rectified points.
4. ✅ `cv::solvePnP(..., SOLVEPNP_SQPNP)` for the initial guess.
5. ✅ `cv::solvePnPRefineLM(...)` for refinement; falls back gracefully if refinement throws.
6. ✅ `(rvec, tvec)` → `cam_T_field` → `field_T_camera` → `field_T_robot = field_T_camera · camera_T_robot`.
7. ✅ RMS reprojection residual is computed from a fresh `cv::projectPoints` pass.

The single-tag path uses libapriltag's `estimate_tag_pose_orthogonal_iteration` plus the `err1/err2` ambiguity ratio (option **B** from the original design choice). Single-tag observations with ambiguity above `covariance.ambiguity_drop_threshold` (default 0.4) are emitted with no `field_to_robot` so fusion will not ingest a flippy hypothesis. The diagnostic ratio is still written to the per-tag detection.

### 4.3 Distortion model — complete

`AprilTagCameraCalibration` carries `distortion_model` and `distortion_coefficients`, and `applyCameraCalibrationContext` copies them out of the active `CameraCalibrationConfig` row. The solver consumes them on every multi-tag solve. Tested by `AprilTagPipelineContext.CalibrationCopiesDistortionFields`.

### 4.4 Camera-to-robot extrinsic — applied in the pipeline

`applyCameraExtrinsicsContext` injects `camera_to_robot` per camera at construction time, version-matched against the active calibration row (or against the pipeline's `calibration_version` filter when one is set). The solver applies it after the PnP solve, so the published observation already carries `field_to_robot`. `FusionService` no longer derives the robot pose itself — it just trusts the published value.

### 4.5 Dynamic covariance — complete

`computeCovariance` (`src/pipelines/AprilTagPoseSolver.cpp:129`) produces a 6×6 diagonal covariance using:

- **Distance.** `distance_factor = max(1, (mean_distance / reference_distance)²)`. Below the reference distance the covariance is left at the base value; beyond, it scales as Z² as required.
- **RMS reprojection error.** `rms_factor = max(1, rms / reference_rms_px)`. Same "no shrinkage below reference" rule.
- **Tag count.** Currently a binary distinction: `single_tag_translation_mult` and `single_tag_rotation_mult` are applied when exactly one tag was solved; otherwise `1.0`. (Default rotation multiplier is 5×, reflecting the optical-axis rotational ambiguity inherent to single-tag PnP.)

The published 6×6 follows GTSAM's `Pose3` tangent ordering `[rx, ry, rz, tx, ty, tz]` with diagonal entries only. `FusionService` consumes the matrix directly via `gtsam::noiseModel::Gaussian::Covariance`. The static `vision_sigmas` array is no longer used on the AprilTag path.

Open refinements (nice-to-have, not blockers):

- **Cross-terms are zero.** A full per-corner uncertainty propagation through the PnP Jacobian would produce off-diagonal terms, especially in the rotation/translation coupling for a tag observed at the image edge. Diagonal-only is the standard practical compromise and it matches the requirement; revisit only if downstream diagnostics show the fusion overconfidently agreeing on a clearly wrong pose.
- **Tag-count gradient is binary, not continuous.** The requirement hints that "≥ 4 well-spread tags" should shrink the covariance further than 2 tags. Currently both produce the same `1.0` multiplier. A small lookup or `1/sqrt(N)` term would close this.
- **Single-tag rotational inflation is isotropic.** All three rotation diagonals get the same `single_tag_rotation_mult`, even though the actual ambiguity is concentrated around the tag's optical axis. Projecting the optical axis into the world frame and inflating only that component would be tighter.

### 4.6 Timestamp and time domain

`Frame::capture_time` is in `steady_clock` (V4L2 buffer timestamp converted to monotonic, see [`producer-consumer-architecture.md`](producer-consumer-architecture.md) §2.5). The pipeline propagates it unchanged into `AprilTagObservation::capture_time`, and `FusionService` uses it to time-order measurements. This is correct for the host-only fusion case.

What's still open: the requirement mentions "the exact teensy time the shutter opened." `CameraTriggerEvent` already publishes Teensy-stamped trigger pulses on the bus (`include/posest/MeasurementTypes.h:90`), and `posest_teensy` populates `teensy_time_us` per pulse. Stitching the trigger event to the frame (and therefore to the AprilTag observation) at sub-millisecond precision is a separate piece of work; today the host steady_clock timestamp is the only time field on the observation.

### 4.7 Latency posture

The pipeline already meets the "lowest latency per camera" rule, with two caveats:

- **Grayscale conversion is on the pipeline thread.** If the V4L2 producer is configured with a Y8/Y16/GREY pixel format, `toGray8` is a no-op. If it's MJPEG or BGR, `cvtColor` runs once per frame on the pipeline worker (microseconds at typical resolutions, but it's there). For a fully optimized path, configure the camera to emit grayscale natively when AprilTag is the only consumer.
- **`cv::Mat::isContinuous()` cloning.** The pipeline `clone()`s the gray Mat if it isn't already continuous in memory. For V4L2 MMAP-backed Mats this is normally already continuous, so this branch shouldn't fire in production. Worth keeping the branch — safer than a libapriltag scan over a non-continuous buffer — but worth being aware of if profiling shows surprise allocations.

The blocking pieces (libapriltag detection, SQPNP, LM, single-tag orthogonal iteration) all run on the pipeline's own worker thread. None of it can stall the V4L2 producer thread.

### 4.8 Web configurability

The requirement is "configurable from the website but we haven't implemented the server yet." Status:

- ✅ The pipeline already reads its parameters from SQLite at startup.
- ✅ `parameters_json` is a free-form JSON object — no schema migration is needed to add new keys (covariance tuning was added without a migration).
- ✅ `SqliteConfigStore::saveRuntimeConfig` is atomic and validated, so a future POST-from-website handler just hands a `RuntimeConfig` to `save()` and is done.
- ❌ `WebService` (`include/posest/runtime/WebService.h`) is currently a thin holder for telemetry state. It has no HTTP server, no route table, no settings handler. This is co-deferred with the video preview consumer (see [`producer-consumer-architecture.md`](producer-consumer-architecture.md) §6.2).
- ❌ No live reconfigure (§3.3). The web layer will need to either (a) trigger a graph rebuild on save, or (b) add a `reconfigure()` path on the pipeline.

---

## 5. Gap summary

| Requirement | Status | Notes |
|-------------|--------|-------|
| Per-camera independent AprilTag consumer | ✅ Complete | One `AprilTagPipeline` per binding, own thread, own mailbox, own config. |
| Lowest-latency frame delivery | ✅ Complete | Inherited from `ConsumerBase` + `LatestFrameSlot`. Verified by core pipeline tests. |
| DB-backed configuration per pipeline | ✅ Complete | `parameters_json` parsed by `parseAprilTagPipelineConfig`, calibrations / extrinsics / field layout injected by factory. |
| Detector tuning surface (decimate/sigma/refine/sharpen/threads) | ✅ Complete | Defaults aligned with high-precision target (`quad_decimate=1.0`, `nthreads=4`). |
| Tag family configurable | ⚠️ Partial | Only `tag36h11` is wired up; other families rejected at parse time. Mechanical to add. |
| Camera intrinsics from active DB calibration | ✅ Complete | `applyCameraCalibrationContext` injects `fx/fy/cx/cy` per camera. |
| Distortion model + coefficients applied before PnP | ✅ Complete | `radtan` passed to `solvePnP`; `equidistant`/fisheye undistorted via `cv::fisheye::undistortPoints` first. |
| Multi-tag SQPNP solve | ✅ Complete | `solveMultiTag` aggregates corners across all tags in `field_to_tags`. |
| Levenberg-Marquardt refinement | ✅ Complete | `cv::solvePnPRefineLM`; SQPNP result kept on refinement throw. |
| Single-tag pose path | ✅ Complete | libapriltag orthogonal-iteration with `err1/err2` ambiguity ratio; gated by configurable threshold. |
| Field layout consumed by the pipeline | ✅ Complete | `applyFieldLayoutContext` resolves `field_layout_id` (or `active_field_layout_id`) and copies tag poses. |
| Camera-to-robot extrinsic applied in pipeline | ✅ Complete | `applyCameraExtrinsicsContext` injects per-camera; solver composes after PnP. |
| Dynamic 6×6 covariance per measurement | ✅ Complete | Distance² + RMS + tag-count scaling. Diagonal-only; cross-terms left as future refinement. |
| Single `PriorFactor` per frame (not per tag) | ✅ Complete | `FusionService::addAprilTags` uses `gtsam::noiseModel::Gaussian::Covariance` with the published 6×6. |
| Pose ambiguity metric correctly populated | ✅ Complete (single-tag only) | Hamming-distance bug fixed. Multi-tag observations leave the per-tag `ambiguity` at `0.0`; that field is now diagnostic-only for single-tag fallback. |
| Steady_clock timestamp on observation | ✅ Complete | Inherited from frame; same domain as IMU/wheel/Teensy. |
| Teensy-clock shutter timestamp on observation | ⚠️ Partial | `CameraTriggerEvent` carries `teensy_time_us`, but it isn't joined to the AprilTag observation today. |
| Web layer to drive configuration | ❌ Deferred | `WebService` has no HTTP server. DB path is ready. |
| Live reconfigure of detector params | ❌ Missing | Detector built once at construction; settings change requires graph restart. |
| Tag-count gradient finer than 1 vs N | ⚠️ Partial | Currently binary (1 vs >1). Doc requirement hints at finer scaling for ≥4 well-spread tags. |
| Direction-aware single-tag rotational covariance | ⚠️ Partial | All rotation diagonals inflated by `single_tag_rotation_mult`. Optical-axis-only inflation is the textbook tighter version. |
| Per-corner uncertainty → off-diagonal covariance | ⚠️ Partial | Diagonal-only. Acceptable per current requirements; revisit if fusion shows overconfidence. |

---

## 6. Suggested next work

The "feature complete" line for the AprilTag → fusion path has been crossed: the pipeline emits `field_to_robot` + 6×6 covariance, fusion ingests it as a single per-frame `PriorFactor` with a per-measurement Gaussian noise model. What remains is on a different track:

1. **Web/HTTP service.** Stand up `WebService` with a route table that reads/writes `RuntimeConfig` through `IConfigStore::save()`. The DB schema is ready; this is HTTP plumbing, not pipeline work. Co-blocked with the video preview consumer.
2. **Live reconfigure.** Either (a) "save → graph rebuild" (cheap to implement, ~1 s of camera downtime per change), or (b) `AprilTagPipeline::reconfigure(AprilTagPipelineConfig)` that swaps the detector struct, calibration map, and field-tag map under a mutex without restarting the camera. (a) unblocks the web flow immediately; (b) is the production-quality version.
3. **Additional tag families.** Lift the `family != "tag36h11"` rejection. Each family needs a `<family>_create()` / `<family>_destroy()` mapping in `AprilTagPipeline`'s constructor/destructor. One-time effort.
4. **Teensy-stamped shutter time on the observation.** Subscribe `AprilTagPipeline` (or a small adapter) to `CameraTriggerEvent`, pair it with the matching frame by `trigger_sequence` ↔ `frame.sequence`, and add a `teensy_time_us` field to `AprilTagObservation`. Useful for sub-ms cross-camera and IMU-vision time alignment once VIO comes online.
5. **Covariance refinements.** In rough cost-benefit order:
   - Continuous tag-count scaling (e.g. multipliers `{1.0, 0.7, 0.55}` for `{1, 2, ≥4}` tags).
   - Optical-axis-only rotational inflation for single-tag (project the camera's view direction into world frame, inflate the rotational diagonal aligned with that axis only).
   - Off-diagonal terms from per-corner Jacobian propagation. Only worth doing if downstream telemetry shows the fusion ingesting clearly-wrong vision priors with high confidence.

### 6.1 Implementation notes (for whoever picks this up)

**Live reconfigure ordering.** The detector's `nthreads`, `quad_decimate`, `quad_sigma`, `refine_edges`, `decode_sharpening`, `debug` fields can all be rewritten between `apriltag_detector_detect` calls — libapriltag rebuilds its internal scratch buffers per detect call. So a `reconfigure()` that grabs a mutex and rewrites those fields plus `config_.camera_calibrations`, `config_.field_to_tags`, `config_.camera_to_robot` is safe with the existing single-worker thread model.

**Multi-family wiring.** `apriltag_detector_clear_families()` removes all attached families; then call the new family's `*_create()` and `apriltag_detector_add_family()`. Don't forget to track the owned family pointer for destruction.

**Trigger ↔ frame join.** Today neither `Frame` nor `CameraTriggerEvent` carries enough state to pair them deterministically — `CameraTriggerEvent::trigger_sequence` exists, but the frame side has no matching field. Adding `Frame::trigger_sequence` (populated by `V4L2Producer` from the kernel sequence counter) is the cleanest path; a fallback would be nearest-`steady_clock` matching with a small lookback window.

---

## 7. File reference

| File | Role |
|------|------|
| `include/posest/pipelines/AprilTagPipeline.h` | Class declaration + `AprilTagPipelineConfig` + `AprilTagCovarianceTuning` + `parseAprilTagPipelineConfig`. |
| `src/pipelines/AprilTagPipeline.cpp` | Detector construction, per-frame `processFrame`, JSON parsing/validation. |
| `include/posest/pipelines/AprilTagPoseSolver.h` | Solver input/output structs. |
| `src/pipelines/AprilTagPoseSolver.cpp` | Multi-tag SQPNP + LM, single-tag orthogonal-iteration fallback, covariance computation. |
| `include/posest/pipelines/VisionPipelineBase.h` | `ConsumerBase`-derived base; provides the worker thread + mailbox. |
| `include/posest/MeasurementTypes.h` | `AprilTagDetection`, `AprilTagObservation` (with `field_to_robot`, `covariance`, `reprojection_rms_px`, `solved_tag_count`). |
| `include/posest/MeasurementBus.h` | Typed pub/sub the pipeline publishes onto. |
| `include/posest/runtime/PipelineConfig.h` | DB-row shape (`parameters_json` lives here). |
| `include/posest/runtime/RuntimeConfig.h` | `CameraCalibrationConfig`, `CameraExtrinsicsConfig`, `FieldLayoutConfig` (intrinsics, extrinsics, field tags). |
| `src/runtime/ProductionFactories.cpp` | `applyCameraCalibrationContext` / `applyCameraExtrinsicsContext` / `applyFieldLayoutContext` — inject DB context into the pipeline config. |
| `src/fusion/FusionService.cpp` | `addAprilTags`: single `PriorFactor<Pose3>` per observation with per-measurement Gaussian noise model. |
| `src/config/SqliteConfigStore.cpp` | Persistence of pipelines, calibrations, extrinsics, field layouts. |
| `test/test_pipelines.cpp` | Config-parsing tests, blank-image / rendered-tag tests, single-tag-with-context test, calibration/extrinsics/field-layout context tests. |
| `test/test_fusion_service.cpp` | End-to-end fusion behavior including the per-measurement vision prior path. |
