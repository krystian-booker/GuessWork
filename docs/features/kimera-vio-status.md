# Kimera-VIO Integration — Status Review

Snapshot of where the Kimera-VIO + `FusionService` integration stands against the
requirements brief. Walks the brief section-by-section, calling out what is in
the tree, what is partial, and what is still missing.

Files audited:
- `include/posest/vio/{IVioBackend,FakeVioBackend,KimeraVioConfig,KimeraVioConsumer,AirborneCovariance}.h`
- `src/vio/{AirborneCovariance,FakeVioBackend,KimeraBackend,KimeraVioConsumer}.cpp`
- `share/posest/kimera/{Frontend,Backend,Imu}Params.yaml`
- `include/posest/TeeSink.h`, `test/test_tee_sink.cpp`
- `test/test_kimera_vio_consumer.cpp`, `test/test_airborne_covariance.cpp`
- `cmake/FindKimeraVIO.cmake`, `CMakeLists.txt`
- `src/runtime/Daemon.cpp`, `src/runtime/ProductionFactories.cpp`, `src/runtime/RuntimeGraph.cpp`
- `src/fusion/FusionService.cpp` (`addVio`)
- `include/posest/runtime/RuntimeConfig.h` (`VioConfig`, `FusionConfig`)
- `include/posest/MeasurementTypes.h` (`VioMeasurement`, `ImuSample`, `ToFSample`)

---

## 1. Architectural pieces

### Implemented

- **`IVioBackend` abstraction** (`include/posest/vio/IVioBackend.h`).
  Backend-neutral interface with non-blocking `tryPushFrame` / `tryPushImu`,
  output-callback contract documented for any-thread emission, and a
  `VioBackendOutput` struct that carries `teensy_time_us`, `world_T_body`,
  6×6 covariance, `tracking_ok` and a status string. Comments call out the
  gtsam Pose3 tangent convention `[rx, ry, rz, tx, ty, tz]` and the
  open verification of Kimera's `state_covariance_lkf_` ordering.

- **`KimeraVioConsumer`** (`include/posest/vio/KimeraVioConsumer.h`,
  `src/vio/KimeraVioConsumer.cpp`).
  - Inherits from `ConsumerBase`, so it plugs into the existing
    producer→consumer plumbing and respects the non-blocking `deliver()`
    contract.
  - Owns three logical threads: the `ConsumerBase` worker (frames),
    the IMU drainer (`imuDrainerLoop`), and the backend's output callback.
  - Drains IMU samples up to each frame's Teensy timestamp via
    `drainImuUpTo()` so the backend sees IMU bracketing the frame.
  - Emits `VioMeasurement`s as `between(prev, curr)` deltas; first frame
    is intentionally skipped.
  - Per-frame airborne state lookup keyed by `teensy_time_us` so
    out-of-thread backend callbacks recover the correct gate decision.
  - `KimeraVioStats` exposes frames_pushed/dropped, IMU pushed/dropped,
    outputs received/published/skipped/inflated, and ground-distance
    misses.
  - Covariance strategy enum (`kAbsolute`, `kDelta`, `kScaled`) with a
    PSD floor on the `kDelta` path.

- **`FakeVioBackend`** (`include/posest/vio/FakeVioBackend.h`,
  `src/vio/FakeVioBackend.cpp`). Deterministic synthetic-trajectory
  backend used by tests and Linux CI; supports tracking-ok ramps and
  forced-backpressure injection.

- **`AirborneTracker` + `inflate()`** (`include/posest/vio/AirborneCovariance.h`,
  `src/vio/AirborneCovariance.cpp`).
  - Schmitt-trigger hysteresis: `above_m=0.15` (6 in), `below_m=0.127`
    (5 in), 50 ms `kSettling` window — exactly the brief's spec.
  - Three states: `kGrounded`, `kAirborne`, `kSettling`.
  - Diagonal-only inflation with cap (1e6 default) so PSD is preserved.
  - Missing-sample policy: holds state, increments a counter, still
    expires settle timer — covered by tests.

- **`TeeSink`** (`include/posest/TeeSink.h`).
  Per-overload publisher-side fan-out so a single
  `IMeasurementSink&` reference in `TeensyService` can multicast `ImuSample`
  to both the fusion bus and a dedicated VIO bus, while keeping
  `MeasurementBus`'s single-consumer contract intact. Tested in
  `test/test_tee_sink.cpp`.

- **GTSAM-side ingestion**. `FusionService::addVio()`
  (`src/fusion/FusionService.cpp:684-748`) consumes `VioMeasurement`,
  reconstructs the 6×6 covariance, accepts it as a Gaussian noise model
  (with PSD fallback to `vio_default_sigmas`), and adds the
  `BetweenFactor<gtsam::Pose3>` against the existing pose chain.
  Honors `enable_vio` and `tracking_ok`.

- **Parameter YAML scaffolding** (`share/posest/kimera/`).
  All three files exist with the brief's defaults populated:
  - `FrontendParams.yaml`: `feature_detector_type: 1`,
    `max_features_per_frame: 800`, `klt_win_size: 31`, `klt_max_level: 4`,
    `max_feature_age: 12`, `quality_level: 0.001`, `min_distance: 15`,
    `ransac_threshold_mono: 1.5`.
  - `BackendParams.yaml`: `useSmartFactors: true`, `huberLossThreshold: 1.0`,
    `linearizationMode: 1` (HESSIAN). Casing convention warning is
    documented in-file.
  - `ImuParams.yaml`: `accelerometer_noise_density: 0.02`,
    `gyroscope_noise_density: 0.005`, `imu_integration_sigma: 1e-8`,
    `nominal_sampling_time_s: 0.001`.
  - YAMLs are installed alongside the daemon (`CMakeLists.txt:122-126`).

- **Build wiring** (`CMakeLists.txt`).
  - `posest_vio_core` always builds (FakeVioBackend, AirborneCovariance,
    KimeraVioConsumer) — Linux CI runs the consumer suite without
    Kimera installed.
  - `posest_vio_kimera` is opt-in (`POSEST_BUILD_VIO`, default ON only on
    Apple) and gated by `cmake/FindKimeraVIO.cmake`.

- **Tests**. `test/test_kimera_vio_consumer.cpp` covers first-frame skip,
  tracking-ok=false drop, airborne inflation, IMU forwarding, and
  missing-`teensy_time_us` drop. `test/test_airborne_covariance.cpp`
  covers the hysteresis + inflation primitives. `test/test_tee_sink.cpp`
  covers the fan-out.

---

## 2. Gaps and missing work

### A. The Kimera adapter is a stub

`src/vio/KimeraBackend.cpp` compiles but is **non-functional**:

- `tryPushFrame`, `tryPushImu`, `start`, `stop` all early-return without
  touching a real `VIO::Pipeline`. Bodies are `TODO(KIMERA-API)`
  comments showing the intended call shapes (`fillSingleCameraQueue`,
  `fillSingleImuQueue`, `registerBackendOutputCallback`,
  `MonoImuPipeline`).
- Kimera include paths are commented out — the file does not currently
  depend on any Kimera header.
- The `BackendOutput → VioBackendOutput` translation (extracting
  `W_State_Blkf_.pose_` and the upper-left 6×6 of `state_covariance_lkf_`)
  is described in comments but not implemented.
- No YAML loader call (`VIO::VioParams params(param_dir_)`).
- No verification test confirming Kimera's leading 6×6 covariance block
  matches gtsam's `[rx,ry,rz,tx,ty,tz]` ordering — this is flagged in
  `IVioBackend.h:30-34` as load-bearing.

**Net effect:** even with `POSEST_BUILD_VIO=ON` and a Kimera install, no
frames or IMU samples reach Kimera. The adapter is a placeholder.

### B. KimeraVioConsumer is not wired into the daemon

The runtime never constructs a `KimeraVioConsumer`. Verified by grepping
`src/runtime/`, `app/`, and `src/teensy/`:

- `Daemon::loadAndBuild` (`src/runtime/Daemon.cpp:853-916`) creates a
  single `MeasurementBus` and wires `TeensyService` directly to it. No
  `TeeSink`, no second VIO IMU bus, no consumer construction.
- `ProductionFactories::createPipeline` for `type == "vio"` returns
  `PlaceholderVioPipeline` (`ProductionFactories.cpp:132-133`). That
  placeholder always publishes `tracking_ok=false`, so `FusionService`
  drops it. The real `KimeraVioConsumer` is never instantiated and
  never subscribed to a camera as a frame consumer.
- `RuntimeGraph` (`src/runtime/RuntimeGraph.cpp`) wires camera→pipeline
  only; there is no parallel "camera→KimeraVioConsumer" path.

**Net effect:** the production binary cannot run real VIO today, even
with a hypothetical real KimeraBackend. Daemon integration is the next
plumbing step.

### C. RuntimeConfig (SQLite) does not persist Kimera tunables

`VioConfig` in `include/posest/runtime/RuntimeConfig.h:123-141` covers
ToF, IR LED, slot index — but **none** of the `KimeraVioConfig` fields:

- `param_dir`
- `AirborneThresholds` (above_m, below_m, settle window)
- `inflation_factor`, `inflation_cap`
- `CovarianceStrategy`, `covariance_scale_alpha`
- `imu_buffer_capacity`, `airborne_lookup_capacity`

The YAMLs are static files on disk. The brief calls for "database of
the VIO required for this specific high-speed, repetitive-carpet
environment" — i.e., these knobs should round-trip through
`SqliteConfigStore` so the website can tune airborne thresholds,
covariance strategy, and inflation without recompiling. That schema
extension and validator coverage is missing.

There is also no template/generator that emits a tuned
`Frontend/Backend/ImuParams.yaml` from `RuntimeConfig` — the YAMLs are
hand-edited.

### D. ImuParams.yaml has unfilled production fields

Explicit TODOs in `share/posest/kimera/ImuParams.yaml`:

- `accelerometer_random_walk` and `gyroscope_random_walk` — required for
  long-horizon VIO drift; currently relying on Kimera defaults.
- `IMUtoBodyT_BS` — the body↔IMU extrinsic. Comment notes it should be
  populated at runtime from `RuntimeConfig`, but no plumbing exists.
  (`RuntimeConfig::FusionConfig::imu_extrinsic_body_to_imu` does exist —
  it just isn't pushed into the YAML at start.)

### E. ImuSample wire path drops `teensy_time_us`

`ImuSample` (`MeasurementTypes.h:72-78`) carries only a host
`Timestamp`. The Teensy wire format includes `teensy_time_us` but
`TeensyService` strips it before publishing onto the bus.

`KimeraVioConsumer::drainImuUpTo` works around this by converting the
*frame* `teensy_time_us` to host time and comparing in steady_clock
(`KimeraVioConsumer.cpp:164-217`, with TODO(IMU-T)). When the IMU is
finally pushed to the backend, the consumer reuses `frame_teensy_us` as
the IMU timestamp, which is **not** the IMU's real capture time.
Acceptable for `FakeVioBackend`; a small but real source of error for
Kimera's pre-integration once the adapter goes live.

The fix is wire-level: extend `ImuSample` to carry `teensy_time_us` and
populate it in `TeensyService`.

### F. ToF used only for the airborne gate, not as a depth input

The brief says: *"If Kimera supports a ToF sensor for our single mono
camera, it should be used to improve the VIO."*

Current state: ToF is exclusively a covariance-inflation gate via
`AirborneTracker`. It is never fed to Kimera as a depth/range channel.

This is partly a Kimera capability question — Kimera-VIO upstream is a
mono+IMU and stereo+IMU pipeline; there is no first-class single-pixel
ToF input. Practical paths if a future depth signal is wanted:
1. **Scale prior**: inject ToF as a unary prior on the body's z-height in
   `FusionService` (separate from Kimera). Requires no Kimera changes.
2. **Kimera RGB-D mode**: only available if a depth *image* exists, which
   the VL53L4CD does not produce.
3. **Custom factor inside Kimera**: invasive.

A decision/design note on which (if any) of these we will pursue is
missing from the repo.

### G. Verification gaps

Even what is implemented could use stronger acceptance tests before
flight:

- No round-trip test that asserts `state_covariance_lkf_` upper-left
  6×6 actually matches gtsam tangent ordering. Comments say "verify
  before enabling on hardware"; the test does not exist.
- No integration test wiring `KimeraVioConsumer` end-to-end through
  `FusionService` (i.e., delta in → `BetweenFactor` posted → estimate
  updated). `test_fusion_service.cpp` and
  `test_kimera_vio_consumer.cpp` cover the two halves separately.
- No telemetry surface. `KimeraVioStats` is exposed by the consumer but
  not threaded into `DaemonHealth` for the web/JSON status endpoint
  (compare to how fusion stats are surfaced in `Daemon.cpp:561-563`).

### H. Build/CI coverage on Linux

`POSEST_BUILD_VIO` defaults to `OFF` on Linux. No Linux CI job builds
`posest_vio_kimera`, so even after the stub is filled in the adapter
will not get a compile-check on PRs unless a developer runs locally
with Kimera installed. Worth tracking as a follow-up once the adapter
is real.

---

## 3. Status by requirement

| Requirement | State |
|---|---|
| Consumer-model integration (single-consumer, plugs into existing pipeline) | **Class implemented**, not wired into the daemon graph |
| Mono camera + IMU from Teensy | Frames + IMU plumbing in the consumer; not subscribed to a real camera at runtime |
| ToF used to improve VIO (if Kimera supports it) | Used only for airborne gate; no depth path into Kimera |
| Hardware-timestamp end-to-end (`teensy_time_us`) | Frames yes; IMU drops `teensy_time_us` on the bus, worked around with frame time |
| `BetweenFactor<Pose3>` + 6×6 covariance to `FusionService` | Implemented in `FusionService::addVio` |
| ToF airborne logic: > 6 in airborne, 50 ms settle, near-infinite covariance | Implemented (`AirborneTracker`, `inflate`, integrated into `KimeraVioConsumer`) |
| `KimeraVioConsumer` initializes Kimera, pushes frames + IMU | Class exists; `KimeraBackend::start/tryPush*` are TODO stubs |
| Extracts optimized pose + marginal covariance from Kimera | Plumbed via `VioBackendOutput`; real extraction not implemented |
| Applies airborne covariance inflation | Implemented |
| Pushes final `VioMeasurement` to `FusionService` | Implemented (via `IMeasurementSink`) |
| Parameter file generation (database-driven) | YAMLs exist with brief-spec defaults; no SQLite-backed schema, no generator |
| BMI088-tuned IMU params (noise densities, integration sigma) | Present in `ImuParams.yaml`; random-walk + body-IMU extrinsic still TODO |
| Frontend defaults (GFTT, 800 features, KLT 31/4, age 12, quality 0.001, min_dist 15, RANSAC 1.5) | All present in `FrontendParams.yaml` |
| Backend defaults (smart factors, Huber 1.0, HESSIAN linearization) | Present in `BackendParams.yaml` |

---

## 4. Suggested next steps (in dependency order)

1. **Land the real `KimeraBackend`** — fill in the
   `TODO(KIMERA-API)` bodies, register the output callback, parse YAMLs,
   translate `BackendOutput → VioBackendOutput`. Add a Linux CI job
   that builds with `POSEST_BUILD_VIO=ON` against a pinned Kimera tag.
2. **Add a covariance-ordering verification test** against the installed
   Kimera (or a recorded fixture).
3. **Wire `ImuSample::teensy_time_us`** end-to-end and remove the
   `TODO(IMU-T)` shortcut in `KimeraVioConsumer::drainImuUpTo`.
4. **Extend `RuntimeConfig::VioConfig`** with the `KimeraVioConfig`
   fields, migrate `SqliteConfigStore`, validate, and surface them in
   the website.
5. **Hook a YAML generator** that writes
   `Frontend/Backend/ImuParams.yaml` from `RuntimeConfig` at daemon
   start (or before `KimeraBackend::start`), so DB tunables actually
   reach Kimera. Include random-walk + body-IMU extrinsic.
6. **Wire `KimeraVioConsumer` into `Daemon::loadAndBuild`**: build a
   second `MeasurementBus` for VIO IMU, install a `TeeSink` between
   `TeensyService` and the two buses, construct the
   `KimeraVioConsumer`, subscribe it to the configured VIO camera, and
   route its output to `FusionService`'s measurement bus. Surface
   `KimeraVioStats` through `DaemonHealth`.
7. **Decide on a ToF-to-VIO path** beyond the airborne gate (most
   likely: a z-height unary prior in `FusionService`, gated by ToF
   range_status), and document the choice.
