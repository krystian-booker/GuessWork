# AprilTag Pipeline

**Status:** End-to-end functional. Each `AprilTagPipeline` is an independent low-latency consumer with per-camera intrinsics/extrinsics/field-layout context, a multi-tag SQPNP+LM solve path, a single-tag orthogonal-iteration path with ambiguity drop, and a dynamic 6×6 covariance feeding `FusionService` as a `gtsam::PriorFactor<gtsam::Pose3>`. Hardware-trigger timestamps tighten `Frame::capture_time` to true shutter time at the producer; per-tag `camera_to_tag` and `reprojection_error_px` are now populated; `AprilTagPipelineStats` (outcome counters, solve latency, mailbox drops, last RMS) is surfaced through `DaemonHealth`. Remaining gaps all require the (deferred) HTTP server: live re-config from the website and hot-reconfigure of detector knobs without a pipeline rebuild.

This document covers the AprilTag vision consumer end-to-end: what it is, how it integrates with the producer/consumer pipeline, what knobs are exposed, how the pose solve and dynamic covariance work, and what's missing before it is feature-complete. Generic plumbing (threads, mailboxes, fan-out, lifecycle) lives in [`producer-consumer-architecture.md`](producer-consumer-architecture.md); this doc only covers the AprilTag-specific layer.

---

## 1. Requirements (target behavior)

The bar this document is reviewing against:

1. **Lowest possible latency from shutter → fused pose.** Each AprilTag consumer must process the most recent frame available and never back-pressure its producer.
2. **Independent per-camera consumers.** Multi-camera setups (~6 cameras) each run their own `AprilTagPipeline` with their own configuration loaded from the SQLite store. No shared state between them.
3. **Configurable from the website.** The full set of detector + solver + covariance knobs must be addressable as data — JSON the web layer can read and write — even though the HTTP server is not yet built.
4. **Detector tuned for high precision** (hardware triggers handle motion blur upstream): `quad_decimate` 1.0–1.5, `quad_sigma` 0.0, `refine_edges` true, `decode_sharpening` 0.25, `nthreads` 3–4.
5. **Multi-tag solve done correctly.** When N≥2 tags are visible, aggregate all 4N image corners + 4N world corners and solve once with `cv::SOLVEPNP_SQPNP` followed by `cv::solvePnPRefineLM`. Do NOT solve per-tag and average — that is mathematically broken and doesn't resolve flip ambiguity.
6. **Single-tag handled honestly.** A lone tag is intrinsically ambiguous (planar PnP). Compute the ambiguity ratio and drop the result above a threshold; otherwise emit it with inflated rotational covariance along the optical axis.
7. **GTSAM payload.** What the pipeline publishes must be `(timestamp, gtsam::Pose3 field_to_robot, 6×6 covariance)`. The pose must already include the static camera→robot extrinsic; covariance must be in gtsam tangent ordering `[rx, ry, rz, tx, ty, tz]`.
8. **Dynamic covariance.** Trust scales with: distance (Z²), tag count (single tag → much higher rotational uncertainty; many tags → tightly constrained), and reprojection RMS (the solver's own quality signal).

---

## 2. Architecture

### 2.1 Where it sits in the pipeline graph

```
┌──────────── capture thread ───────────┐    ┌──── AprilTagPipeline worker ───┐
│ V4L2Producer.grabFrame() →            │    │                                │
│ Frame{capture_time, sequence,         │ ──▶│ slot_.take()  (drop-oldest)    │
│       camera_id, image,               │    │ processFrame(frame)            │
│       teensy_time_us?, trigger_seq?}  │    │   ├─ libapriltag detect        │
└───────────────────────────────────────┘    │   ├─ multi-tag SQPNP + LM      │
                                             │   │   OR single-tag orth-iter  │
                                             │   ├─ dynamic 6×6 covariance    │
                                             │   ├─ camera→robot extrinsic    │
                                             │   └─ MeasurementBus.publish()  │
                                             └────────────┬───────────────────┘
                                                          │ AprilTagObservation
                                                          ▼
                                              ┌──── FusionService ────────────┐
                                              │ gtsam::PriorFactor<Pose3>      │
                                              │ + Gaussian noise from cov     │
                                              └────────────────────────────────┘
```

| Component | File | Role |
|-----------|------|------|
| `AprilTagPipeline` | `include/posest/pipelines/AprilTagPipeline.h`, `src/pipelines/AprilTagPipeline.cpp` | The consumer. Owns the libapriltag detector + family handles; runs `processFrame` on the inherited `ConsumerBase` worker thread. |
| `AprilTagPoseSolver` | `include/posest/pipelines/AprilTagPoseSolver.h`, `src/pipelines/AprilTagPoseSolver.cpp` | Pure pose solve + dynamic covariance. No threading, no libapriltag detection — just inputs → outputs. Unit-testable in isolation. |
| `VisionPipelineBase` | `include/posest/pipelines/VisionPipelineBase.h` | `ConsumerBase` + `IMeasurementSink` reference. Provides the worker thread, drop-oldest mailbox, and the publish hook. |
| `AprilTagObservation` | `include/posest/MeasurementTypes.h:43` | The struct published onto `MeasurementBus`. Carries timestamps, per-tag detections, the aggregated `field_to_robot` pose, and the 6×6 covariance. |
| `ProductionPipelineFactory` | `src/runtime/ProductionFactories.cpp` | Parses the SQLite-stored JSON config and injects `RuntimeConfig` context (intrinsics, extrinsics, field layout) into `AprilTagPipelineConfig`. |
| `FusionService` | `src/fusion/FusionService.cpp:121` | Subscribes to the bus, converts `AprilTagObservation` into a `gtsam::PriorFactor<gtsam::Pose3>` with `gtsam::noiseModel::Gaussian::Covariance`, and updates the iSAM2 graph. |

### 2.2 Per-pipeline isolation

Each binding in `RuntimeConfig::bindings` produces one `AprilTagPipeline` instance. `RuntimeGraph::build()` creates a separate object per binding, with its own:

- Worker thread (inherited from `ConsumerBase`).
- `LatestFrameSlot` mailbox (drop-oldest).
- libapriltag `apriltag_detector_t*` and family handle (constructed in the ctor, destroyed in the dtor).
- `AprilTagPipelineConfig` — copied from the SQLite-loaded config with that camera's calibration, extrinsics, and field-tag layout patched in.

There is no shared mutable state across AprilTag pipelines, no cross-pipeline locks. The producer fan-out (`producer-consumer-architecture.md` §4) feeds each pipeline its own `shared_ptr<const Frame>`; the image data is shared (cheap), the detection state is not.

Latency consequence: a slow pipeline (e.g. one camera staring at 8 distant tags with quad_decimate=1.0) drops frames in its own mailbox without affecting any other pipeline or any producer.

### 2.3 The latency contract for AprilTag specifically

The pipeline inherits the [latency contract](producer-consumer-architecture.md#23-the-latency-contract) from `VisionPipelineBase`:

- `deliver()` is non-blocking — it forwards into `LatestFrameSlot::put()` and returns.
- `processFrame()` runs on the worker thread and may take as long as it takes.
- If `processFrame` hasn't returned by the time the next frame arrives, the older frame is dropped and the worker eventually pulls the newest. `LatestFrameSlot::droppedCount()` reports the per-pipeline drop count.

The detector's `nthreads` is the key knob for keeping `processFrame` under the inter-frame budget; on the target hardware (Mac Mini class) 3–4 threads typically holds 60 FPS at 1280×720 with 1–2 quad decimation.

---

## 3. Detector configuration

All knobs are defaults-good values matching the recommended targets (§1.4) and are overridable via the `pipelines.parameters_json` blob in SQLite. The schema is `AprilTagPipelineConfig` (`include/posest/pipelines/AprilTagPipeline.h:38`):

| Key | Default | Range | Purpose |
|-----|---------|-------|---------|
| `family` | `"tag36h11"` | one of `tag36h11`, `tag25h9`, `tagStandard41h12`, `tagCircle21h7`, `tagCustom48h12` | Tag family bound at construction. |
| `nthreads` | 4 | >0 | libapriltag detector thread pool. |
| `quad_decimate` | 1.0 | >0 | Image downsampling before quad detection. Lower = sharper corners, higher CPU. |
| `quad_sigma` | 0.0 | ≥0 | Gaussian blur σ before quad detection. Hardware triggers already produce crisp frames; keep at 0. |
| `refine_edges` | true | bool | Sub-pixel corner refinement via gradient line fitting. Non-negotiable for low pose jitter. |
| `decode_sharpening` | 0.25 | ≥0 | Bit-decode contrast recovery for short-exposure frames. |
| `debug` | false | bool | libapriltag's debug-image dump. Leave off in production. |
| `tag_size_m` | 0.1651 | >0 | Physical edge length (FRC tag default 6.5"). |
| `family`-level fields above are **per-pipeline** and apply to every camera bound to it.

Validation lives in `parseAprilTagPipelineConfig` (`src/pipelines/AprilTagPipeline.cpp:77`); every range above throws `std::invalid_argument` if violated. The SQLite layer rejects malformed JSON before it ever reaches the pipeline (see `test_config_schema.cpp:701`).

### 3.1 Covariance tuning block

`parameters_json.covariance` (optional object) overrides `AprilTagCovarianceTuning`:

| Key | Default | Meaning |
|-----|---------|---------|
| `base_sigma_translation_m` | 0.02 | Reference translational σ at distance 1 m, RMS 1 px, well-spread multi-tag. |
| `base_sigma_rotation_rad` | 0.02 | Reference rotational σ in the same regime. |
| `reference_distance_m` | 1.0 | Distance at which the distance factor is 1. |
| `reference_rms_px` | 1.0 | RMS at which the RMS factor is 1. |
| `single_tag_translation_mult` | 1.5 | Translational inflation when only one tag is visible. |
| `single_tag_rotation_mult` | 5.0 | Rotational inflation along the optical axis when only one tag is visible. |
| `ambiguity_drop_threshold` | 0.4 | Single-tag ambiguity ratio (`err1/err2`) above which the pose is dropped. |
| `multi_tag_decay_k` | 2.0 | Exponential decay rate of the per-tag-count multiplier toward `well_spread_floor_mult`. |
| `well_spread_floor_mult` | 0.5 | Asymptotic floor of the multiplier as N → ∞. Must be in (0, 1]. |

These fields are exactly what a future website settings page would render — every covariance behavior in §5 is a closed-form function of the values above.

### 3.2 Per-camera context (injected, not in the JSON)

The pipeline JSON does *not* hold intrinsics, extrinsics, or the field layout. Those are normalized rows in the SQLite store and are merged into `AprilTagPipelineConfig` by `ProductionPipelineFactory::createPipeline` before construction (`src/runtime/ProductionFactories.cpp:118`):

- `RuntimeConfig::calibrations` (rows where `active=true` and `version` matches `parameters_json.calibration_version` if specified) → `camera_calibrations[camera_id]`. Carries `fx, fy, cx, cy, distortion_model, distortion_coefficients`.
- `RuntimeConfig::camera_extrinsics` (matching the active calibration version) → `camera_to_robot[camera_id]`.
- `RuntimeConfig::field_layouts[active_field_layout_id_or_override].tags` → `field_to_tags[tag_id]`.

A pipeline runs without intrinsics/extrinsics for a given camera by simply not populating that map entry; `processFrame` will then publish detections (corners + tag id) without any pose solve. The pose solve is only attempted for tags that have *both* a calibration entry for the source camera *and* a field-layout entry for that tag id.

---

## 4. The pose solve

Source: `src/pipelines/AprilTagPoseSolver.cpp`. Entry point: `solveAprilTagPose(input)`.

### 4.1 Multi-tag path (N ≥ 2)

`solveMultiTag` (`src/pipelines/AprilTagPoseSolver.cpp:186`):

1. **Aggregate all corners.** For each detected tag, the 4 corners-in-tag-frame `(±half, ±half, 0)` are transformed to world coordinates with `field_T_tag` (from the field layout) and stacked into one big `vector<cv::Point3d>` of size 4N. The matching 4N image corners (libapriltag's sub-pixel-refined `det->p[]`) form `vector<cv::Point2d>`.
2. **Distortion handling.**
   - `radtan` (or empty `distortion_model`): pass `D` through `cv::solvePnP` directly — OpenCV applies the radial-tangential model internally.
   - `equidistant` / `fisheye` / `kannala_brandt`: undistort image points via `cv::fisheye::undistortPoints` first, then pass an empty `D` to SQPNP. SQPNP itself is a pinhole solver; this preserves correctness without forking the solver.
3. **Solve.** `cv::solvePnP(..., cv::SOLVEPNP_SQPNP)` — Sequential Q-PnP. Robust to coplanar points (common when tags share a wall) and natively eliminates flip ambiguity.
4. **Refine.** `cv::solvePnPRefineLM` runs a Levenberg-Marquardt iteration to drive reprojection error to its mathematical minimum. On exception the SQPNP result is kept.
5. **Reprojection RMS.** Re-project the 4N world corners with the refined pose and compute `sqrt(mean(dx² + dy²))` over all corners. This is the quality signal fed into the covariance.
6. **Mean tag distance.** Average Z in the camera frame across all 4N corners. Feeds the distance factor.
7. **Compose extrinsic.** `field_T_camera = (cam_T_field).inv()`; `field_T_robot = field_T_camera * camera_T_robot`. The published pose is *robot* pose — the static camera→robot transform is already applied.

Tests covering this path (all in `test/test_apriltag_pose_solver.cpp`):

- `MultiTagSqpnpRecoversIdentityPose`, `MultiTagSqpnpRecoversTranslatedRobot` — synthetic 2-tag projection round-trips to <1 mm.
- `MultiTagSqpnpHandlesRadtanDistortion` — non-trivial radial-tangential distortion.
- `MultiTagSqpnpHandlesEquidistant` — fisheye undistortion path.
- `EmptyInputProducesEmptyOutput`, `NoCalibrationProducesEmptyOutput` — guard rails.

### 4.2 Single-tag path (N = 1)

`solveSingleTag` (`src/pipelines/AprilTagPoseSolver.cpp:283`):

1. Call libapriltag's `estimate_tag_pose_orthogonal_iteration` — this returns *two* hypotheses (`pose1`, `pose2`) and their reprojection errors (`err1`, `err2`). Planar PnP from a single square is intrinsically two-fold ambiguous; the orthogonal iteration is what libapriltag has tuned for this.
2. **Ambiguity = `err1 / err2`** (clamped to [0, 1]). Closer to 1 = the two hypotheses fit the corners equally well = ambiguous. Closer to 0 = `pose1` is clearly better.
3. If ambiguity > `covariance.ambiguity_drop_threshold` (default 0.4), the pipeline still publishes the detection corners but **omits `field_to_robot`** (the `optional<Pose3d>` is left empty). FusionService treats this as `kFusionStatusVisionUnavailable` and applies no factor. The ambiguity value itself is always reported via `AprilTagDetection::ambiguity` and `AprilTagPoseSolveOutput::single_tag_ambiguity`.
4. Otherwise compose the extrinsic the same way as the multi-tag path and emit the pose.
5. Reprojection RMS is computed manually from the kept hypothesis (project tag-frame corners with intrinsics, compare to libapriltag's sub-pixel corners).

Tests:

- `SingleTagPathReportsAmbiguityFromOrthogonalIteration` — round-trips a rendered tag through libapriltag and asserts the ambiguity is in [0, 1].
- `SingleTagDropsWhenThresholdBelowReportedAmbiguity` — confirms the drop behavior.

### 4.3 Why the two paths are different solvers

SQPNP wants ≥ 4 non-coplanar points for a unique solution; a single tag gives 4 *coplanar* corners and is theoretically two-fold ambiguous. Running SQPNP on it would still pick one hypothesis but lose the ambiguity signal — the orthogonal-iteration path keeps both hypotheses around so we can see the ratio and decide whether to drop.

For N ≥ 2 tags, SQPNP's coplanar-robustness handles the case where all tags happen to be on the same plane (e.g. one wall) without falling back to two-fold ambiguity. That is the point of preferring SQPNP over `EPNP`/`ITERATIVE`.

---

## 5. Dynamic covariance

The covariance is a **diagonal** 6×6 matrix in gtsam Pose3 tangent ordering `[rx, ry, rz, tx, ty, tz]`, row-major. Off-diagonals are zero. The relevant code is `covarianceFactors` (`src/pipelines/AprilTagPoseSolver.cpp:141`) and `computeSingleTagCovariance` / `computeCovariance`.

### 5.1 Three multiplicative factors

Each diagonal sigma is built as:

```
σ_t = base_sigma_translation_m × distance_factor × rms_factor × tag_count_mult_t
σ_r = base_sigma_rotation_rad  × distance_factor × rms_factor × tag_count_mult_r
```

| Factor | Formula | Rationale |
|--------|---------|-----------|
| `distance_factor` | `max(1, (mean_distance_m / reference_distance_m)²)` | Triangulation error scales quadratically with depth. A tag at 4 m is 16× less trusted than at 1 m. |
| `rms_factor` | `max(1, rms_px / reference_rms_px)` | Solver's own quality signal. High residuals → poor pose, scale up. |
| `tag_count_mult` | `floor + (single - floor) · exp(-(N-1) / k)` | Smooth interpolation: at N=1, multiplier = `single_tag_*_mult`; as N grows, decays exponentially toward `well_spread_floor_mult`. |

The per-axis tag-count multiplier is asymmetric: translation uses `single_tag_translation_mult` (default 1.5), rotation uses `single_tag_rotation_mult` (default 5.0). A lone tag introduces almost all of its uncertainty in *yaw* / optical-axis rotation (§5.2), not in position.

### 5.2 Single-tag rotational anisotropy

For multi-tag, the rotational diagonal is uniform across the three rotation axes. For single-tag, it is *not* — `computeSingleTagCovariance` (`src/pipelines/AprilTagPoseSolver.cpp:369`) inflates only the variance along the camera's optical axis (`+Z` in the camera frame), because that's the axis a planar tag is least informative about.

The construction:

1. Two variances: `var_xy = σ_r²` (camera X/Y axes) and `var_z = (σ_r · single_tag_rotation_mult)²` (camera Z = optical axis).
2. Build a diagonal 3×3 in *camera* frame: `diag(var_xy, var_xy, var_z)`.
3. Rotate to world via `field_T_camera`: `Σ_world_R = R · diag(...) · Rᵀ`.
4. Write only the diagonal of `Σ_world_R` into the output covariance's rotational block.

Off-diagonals of the rotated 3×3 are non-zero in general, but the published covariance discards them (zero off-diagonals are easier for FusionService's `gtsam::noiseModel::Gaussian::Covariance` to handle without conditioning issues, and the trace is preserved). The trace-invariance property is asserted by `SingleTagInflationDegradesGracefullyAcrossObliqueAxes`.

Tests:

- `CovarianceScalesQuadraticallyWithDistance` — 1 m → 2 m, variance grows by ~16×.
- `CovarianceScalesWithReprojectionRms` — perturbed corners → larger covariance.
- `CovarianceShrinksWithMoreTags`, `CovarianceFloorAsymptoteWithLargeN` — N=2,4,8 monotonically tighter; N=8 sits near the asymptotic floor.
- `SingleTagOpticalAxisInflationCameraAlignedWithWorldZ` and `…WorldX` — anisotropy rotates correctly with the camera.
- `SingleTagCovarianceOffDiagonalsStayZero` — diagonals only.

### 5.3 What gets published

`AprilTagObservation::covariance` is the 6×6 row-major `std::array<double, 36>`. `FusionService::addAprilTags` (`src/fusion/FusionService.cpp:121`) copies it into a `gtsam::Matrix6` and constructs `gtsam::noiseModel::Gaussian::Covariance`, which becomes the noise of the `gtsam::PriorFactor<gtsam::Pose3>` attached to the current pose key in iSAM2. If the matrix is not positive-definite, the factor is skipped and `kFusionStatusOptimizerError` is set on the next emitted estimate.

---

## 6. The published `AprilTagObservation`

Defined in `include/posest/MeasurementTypes.h:43`. The pipeline always publishes — even on no-detection or drop-by-ambiguity — so the bus reflects every consumed frame:

| Field | Source | Notes |
|-------|--------|-------|
| `camera_id` | `frame.camera_id` | One pipeline per camera; the binding determines this. |
| `frame_sequence` | `frame.sequence` | Producer-assigned monotonic sequence (with gaps from drop-oldest). |
| `capture_time` | `frame.capture_time` | `steady_clock` time point. The shutter time when `ProducerBase` got a kernel/PTP/native stamp; userspace approximation otherwise. |
| `teensy_time_us` | `frame.teensy_time_us` | Optional Teensy-domain microsecond shutter time, populated by `CameraTriggerCache` when a hardware trigger pulse pairs with the frame. |
| `trigger_sequence` | `frame.trigger_sequence` | Optional Teensy pulse sequence number, same source. |
| `detections[]` | libapriltag | Per-tag: `tag_id`, the 4 sub-pixel `image_corners_px`, `ambiguity` (set only on the single-tag path), `reprojection_error_px` (currently always 0 — see §7). |
| `field_to_robot` | solver | `optional<Pose3d>`. Empty on no-detection / no-calibration / ambiguity-drop. |
| `covariance` | solver | 6×6 row-major in gtsam tangent order. |
| `reprojection_rms_px` | solver | RMS over all solved corners; 0 if unsolved. |
| `solved_tag_count` | solver | 0, 1, or N. Distinct from `detections.size()` — tags missing from the field layout are detected but not solved. |

The hardware-trigger pairing happens upstream in `ProducerBase` via `CameraTriggerCache`; if no Teensy is wired or the pairing window misses, both optional fields stay `nullopt` and the steady-clock `capture_time` is the only timestamp. See `include/posest/CameraTriggerCache.h`.

---

## 7. Gap analysis vs. the stated requirements

| Requirement | Status | Notes |
|-------------|--------|-------|
| Independent per-pipeline thread + drop-oldest mailbox | ✅ Complete | Inherited from `VisionPipelineBase` / `ConsumerBase`. See [`producer-consumer-architecture.md`](producer-consumer-architecture.md). |
| Per-pipeline detector instance, no cross-pipeline sharing | ✅ Complete | `apriltag_detector_t*` constructed in ctor, destroyed in dtor. |
| Detector knobs (`quad_decimate`, `quad_sigma`, `refine_edges`, `decode_sharpening`, `nthreads`) configurable | ✅ Complete | All in `AprilTagPipelineConfig`, all validated, all defaulted to recommended targets. |
| Family selection from a fixed set | ✅ Complete | 5 families bound; unknown family rejected at parse time. |
| Multi-tag SQPNP + LM refinement | ✅ Complete | `solveMultiTag`. Tested on identity, translated, radtan, fisheye. |
| Single-tag with libapriltag orthogonal iteration + ambiguity drop | ✅ Complete | `solveSingleTag` + `ambiguity_drop_threshold`. |
| Distortion handling (pinhole / radtan / equidistant / kannala-brandt) | ✅ Complete | radtan via `solvePnP`, fisheye via `cv::fisheye::undistortPoints` first. |
| Per-camera intrinsics injected from SQLite | ✅ Complete | `applyCameraCalibrationContext` in `ProductionFactories`, version-filterable. |
| Per-camera camera→robot extrinsic injected and pre-applied | ✅ Complete | `applyCameraExtrinsicsContext`; `field_T_robot = field_T_camera * camera_T_robot` baked into the published pose. |
| Field-layout (`field_to_tag` per `tag_id`) injected from SQLite | ✅ Complete | `applyFieldLayoutContext`; `parameters_json.field_layout_id` overrides the active one. |
| Dynamic covariance: distance Z² | ✅ Complete | Tested. |
| Dynamic covariance: tag count (smooth, single → floor) | ✅ Complete | Tested. |
| Dynamic covariance: reprojection RMS | ✅ Complete | Tested. |
| Single-tag rotational anisotropy along optical axis | ✅ Complete | Rotated to world via `field_T_camera`. Tested across orientations. |
| GTSAM payload: `(timestamp, Pose3, 6×6 covariance)` | ✅ Complete | `FusionService` consumes it as `gtsam::PriorFactor<gtsam::Pose3>` with `noiseModel::Gaussian::Covariance`. |
| Hardware-trigger shutter timestamp surfaced on the observation | ✅ Complete | `teensy_time_us` + `trigger_sequence` propagate from `Frame` when paired. |
| Use Teensy shutter time (not steady-clock) as the GTSAM timestamp | ✅ Complete | `ProducerBase` now overwrites `frame->capture_time` with the cached trigger event's host-domain timestamp (still `steady_clock`, but tightened from "host saw the frame" to "shutter fired"). `FusionService` gets the precise stamp without any FusionService-side change. See §7.1. |
| **Live re-config from website** | ❌ Deferred | No HTTP server in `WebService`; no hot-reload path. Config is read from SQLite once at daemon start and frozen. See §7.2. |
| `AprilTagDetection::camera_to_tag` populated | ✅ Complete | Multi-tag uses the layout-implied `cam_T_tag = cam_T_field · field_T_tag` (signals which tag disagrees with consensus); single-tag uses the orthogonal-iteration result. See §7.3. |
| Per-tag `reprojection_error_px` populated | ✅ Complete | Multi-tag splits the global residual sum into 4-corner buckets; single-tag uses the existing per-tag RMS. See §7.3. |
| Pipeline runtime telemetry (frames in / dropped / detections / solve latency) | ✅ Complete | `AprilTagPipelineStats` (outcome counters, mailbox drops, solve latency, last RMS) on `DaemonHealth::apriltag_pipelines`, surfaced via `IVisionPipeline::pipelineStats()`. See §7.4. |
| Hot reconfiguration of detector params at runtime | ❌ Missing | `apriltag_detector_t*` is built once in the ctor; changing `nthreads` / `quad_decimate` requires a pipeline rebuild. See §7.5. |

### 7.1 Teensy shutter timestamp adopted at the producer — **DONE**

`Timestamp = std::chrono::steady_clock::time_point`, so `TriggerStamp::host_time` (already converted to host domain by `TeensyService::timestampFromTeensyTime` using a PTP-style sync offset) and the kernel-supplied `frame->capture_time` live in the *same* domain. The fix was a 4-line change in `ProducerBase::runLoop`'s cache-pairing block: when a pulse pairs with a frame, also overwrite `frame->capture_time = stamp->host_time`. `cache->lookup` already requires `host_time <= capture_time` and `capture_time - host_time <= match_window`, so the swap can only move the timestamp earlier (toward true shutter), never into the future.

`FusionService` and `AprilTagPipeline` needed zero changes — `FusionService::timestampOf` already returns `value.capture_time` for AprilTag observations, and the pipeline copies `frame.capture_time` straight through. The improvement propagates automatically.

Caveat (not blocking, called out for future ordering work): `FusionService::acceptTimestamp` enforces strict per-bus monotonicity. With Teensy-stamped AprilTag observations (~µs precision) and kernel-stamped wheel/IMU samples (~ms jitter) interleaving on the same bus, the `stale_measurements` counter may tick up slightly more often. The proper fix is per-source ordering at the bus level, out of scope here.

### 7.2 Website-driven live reconfiguration

The schema side is ready: `parameters_json` is a freeform JSON blob in SQLite, and every detector + covariance knob has a documented key, a default, and a validation range (§3). A future REST endpoint reading/writing `pipelines.parameters_json` would round-trip cleanly.

What's missing:

1. **HTTP server.** `WebService` is a thin facade over `RuntimeConfig` + `TelemetrySnapshot`; no routes, no socket, no lib dependency. Same blocker as [`producer-consumer-architecture.md`](producer-consumer-architecture.md) §6.2.
2. **Hot-reload.** Even with a server, applying a new config requires either (a) rebuilding the pipeline (drop the old detector, build a new one, re-bind the consumer to the producer) or (b) supporting in-place updates of detector knobs (libapriltag exposes the fields directly on `apriltag_detector_t*`, so `nthreads`/`quad_decimate`/`quad_sigma`/`refine_edges`/`decode_sharpening`/`debug` can all be poked live; only `family` requires teardown). The plumbing for either approach is unwritten.

The tractable first step is: when (1) lands, give the website a "save config + restart pipelines" button. Live updates without rebuild come later.

### 7.3 Per-tag detection metadata — **DONE**

`AprilTagPoseSolveOutput` carries a parallel-indexed `std::vector<PerTagOutput>` (one entry per solver-input tag). The pipeline scatters it onto `observation.detections[k].camera_to_tag` and `.reprojection_error_px` using the existing `solver_index_to_detection_index` map.

Semantic choice for multi-tag (deliberate): `cam_T_tag = cam_T_field · field_T_tag` — the *layout-implied* per-tag pose from the global SQPNP solve. The associated reprojection error then measures "how much does *this tag's* corners disagree with the consensus pose," which is the right signal for spotting a misprinted tag, a drifted wall mount, or a layout-file typo. An independent per-tag PnP would give near-zero residuals dominated by detection noise — wrong signal.

Single-tag uses libapriltag's orthogonal-iteration result directly. The per-tag entry is emitted *even when* the global pose is dropped by the ambiguity threshold (the per-tag pose + residual stand independently of the global drop decision).

### 7.4 Pipeline telemetry — **DONE**

`AprilTagPipelineStats` (`include/posest/pipelines/PipelineStats.h`) carries:

- `frames_processed`, plus a 5-way outcome breakdown: `frames_no_detection`, `frames_dropped_no_calibration`, `frames_dropped_by_ambiguity`, `frames_solved_single`, `frames_solved_multi`.
- `mailbox_drops` (read on the fly from `ConsumerBase::droppedByMailbox()` so callers see one consistent struct).
- `last_solve_latency_us`, `max_solve_latency_us` (wall-clock from `processFrame` entry to publish exit).
- `last_reprojection_rms_px`.

Surfacing path: `IVisionPipeline::pipelineStats()` is a virtual hook returning a `std::variant<std::monostate, AprilTagPipelineStats>` (forward-compatible for future Charuco/Aruco/etc. without `dynamic_pointer_cast`). `RuntimeGraph::pipelines()` exposes the ordered pipeline list; `DaemonController::refreshHealth` enumerates uniformly and appends each `AprilTagPipelineStats` to `DaemonHealth::apriltag_pipelines`. `healthToJson` emits the array under the `apriltag_pipelines` key, available via `posest_daemon --health`.

`TelemetrySnapshot` is intentionally untouched — it has no live writer today and that surface lands with the HTTP server.

### 7.5 Hot-reconfigure individual knobs

`apriltag_detector_t*` is mutated only at construction time. Live updates would require a per-pipeline `reconfigure(AprilTagPipelineConfig)` method protected by a mutex around the `apriltag_detector_detect` call (or a frame-boundary synchronization point). The libapriltag API itself is happy — its fields are plain `int`/`float`/`bool` on the struct — but our wrapper hasn't exposed the entry point. Same blocker as §7.2 (no driver yet).

---

## 8. Summary

The detector + solver + covariance stack matches the stated requirements end-to-end and is covered by a focused unit-test suite in `test_apriltag_pose_solver.cpp` and `test_pipelines.cpp`. The pipeline plugs into the existing producer/consumer plumbing without bespoke threading code; multi-camera scaling is a matter of writing more bindings. Hardware-trigger timestamps tighten `Frame::capture_time` at the producer; per-tag pose + residual surface in every published observation; pipeline telemetry counters appear in `DaemonHealth`.

The remaining work all lives behind the (deferred) HTTP server: website-driven configuration (§7.2) and hot-reconfigure of detector knobs without a pipeline rebuild (§7.5). Neither touches the pose solve itself.

---

## 9. File reference

| File | Role |
|------|------|
| `include/posest/pipelines/AprilTagPipeline.h` | `AprilTagPipelineConfig`, `AprilTagCovarianceTuning`, `AprilTagPipeline` class. |
| `src/pipelines/AprilTagPipeline.cpp` | JSON parsing, libapriltag setup, `processFrame` driver. |
| `include/posest/pipelines/AprilTagPoseSolver.h` | Solver inputs / outputs, public `computeSingleTagCovariance` (test surface). |
| `src/pipelines/AprilTagPoseSolver.cpp` | SQPNP+LM multi-tag solve, orthogonal-iteration single-tag solve, dynamic covariance. |
| `include/posest/pipelines/VisionPipelineBase.h` | Worker thread + measurement sink shared base. |
| `include/posest/MeasurementTypes.h` | `AprilTagDetection`, `AprilTagObservation`. |
| `src/runtime/ProductionFactories.cpp` | Parses SQLite-stored JSON, injects per-camera context, instantiates the pipeline. |
| `src/fusion/FusionService.cpp` | Bus subscriber that turns `AprilTagObservation` into a `gtsam::PriorFactor<gtsam::Pose3>`. |
| `test/test_apriltag_pose_solver.cpp` | Solver + covariance unit tests (synthetic projection round-trip, covariance scaling, ambiguity drop). |
