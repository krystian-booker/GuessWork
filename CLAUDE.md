# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project status & hardware testing

Implementation status (Phases 1–7 of the pose-estimation roadmap done — the
full software roadmap) and the staged real-hardware bring-up/test plan live
in **`docs/hardware-bringup.md`** — update it as hardware tests are
completed. The CAN wire contract is **`docs/can-protocol.md`** ↔
`firmware/src/can_payloads.h`. **Every coordinate frame, clock domain,
covariance convention and the degraded-modes matrix: `docs/pose_pipeline.md`**
(frame mix-ups are the #1 bug source — read it before touching pose math).

## Platform constraints

macOS / Apple Silicon only — CMake hard-fails on non-Apple. The pipeline depends on CoreVideo / IOSurface / Metal / VideoToolbox, and a system install of the FLIR **Spinnaker SDK** at `/usr/local/{include,lib}` is required to link `gw_producer` (and therefore everything that pulls it in, including `guesswork` and `gw_tests`). Homebrew OpenSSL at `/opt/homebrew/opt/openssl@3` is used by the vendored libdatachannel WebRTC stack; Homebrew **OpenCV** (`brew install opencv`) is required by `gw_apriltag`.

## Build

Out-of-tree builds live in `build/` (Debug), `build-tsan/`, `build-asan/`, and `build-fresh/` (used by Playwright). The VS Code tasks in `.vscode/tasks.json` are the canonical recipes; on the CLI:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build -j
```

Important CMake options:
- `-DGW_ENABLE_TSAN=ON` / `-DGW_ENABLE_ASAN=ON` — sanitizers (configure into separate build dirs).
- `-DGW_BUILD_WEB=ON` — runs `npm install && npm run build` in `web/` at configure time and embeds `web/dist/` into the `guesswork` binary via CMakeRC, defining `GW_HAS_EMBEDDED_WEB=1`. Defaults to ON for Release, OFF otherwise. In Debug builds without this flag the server exposes only `/api/*` and the React app is served by the Vite dev server.
- `-DGW_BUILD_TESTS=OFF` — disable gtest fetch + `gw_tests` target.
- `-DGW_BUILD_BASALT=ON` — clone and build the Basalt VIO toolchain via `ExternalProject_Add` (see `cmake/basalt.cmake`). **Default OFF.** Calibration moved to Kalibr (Dockerized); this flag exists only for future VIO experimentation with `basalt_vio`. Adds 15–30 min to the first configure/build because Basalt bootstraps its bundled vcpkg and builds the full transitive dep tree. Requires `cmake ≥ 3.24` and `ninja` on PATH; installs into `build/basalt-install/`.
- `-DGW_KALIBR_DOCKER_IMAGE` / `-DGW_KALIBR_TARGET_DEFAULT` — override the docker image tag (default `guesswork/kalibr:latest`) and AprilGrid target path baked into the suggested-command builder. The image is built out-of-band by `docker/kalibr/build.sh`; `install.sh` runs it automatically if Colima is up, otherwise prints `colima start && docker/kalibr/build.sh`.

The vendored libdatachannel stack (libsrtp, libjuice, mbedtls/plog) is configured in `cmake/libdatachannel.cmake`; `cmake/embed_web.cmake` drives the React embed; `cmake/basalt.cmake` drives the optional Basalt source build (VIO only).

## Database schema

The cameras table schema is `CREATE TABLE IF NOT EXISTS` only — we don't write migrations during early development. **When you bump the schema, pull, or branch-switch and the existing DB lacks new columns, run `./build/guesswork --reset-db` to start fresh.** The DB lives at `~/.guesswork/guesswork.db`; calibration recordings under `~/.guesswork/calibrations/<session>/`.

## Run

- `./build/guesswork` — headless: starts the camera pipeline, the WebRTC stream encoder, and the Crow HTTP server. Flags: `--port`, `--stream-width`, `--stream-height`, `--stream-fps`, `--stream-bitrate`. SQLite DB at `$HOME/.guesswork/guesswork.db` (auto-created).
- `cd web && npm run dev` — Vite on :5173, proxies `/api` → `:8080`. Use this for frontend dev against a separately-running `guesswork`.

## Test

Unit tests (GoogleTest):
```bash
ctest --test-dir build --output-on-failure
./build/gw_tests --gtest_filter='FrameChannelTest.*'   # single suite/test
```

`gw_tests` links `gw_core`, `gw_encoder`, and `gw_server` — so it requires the Spinnaker dylib at link time even though the tests themselves don't talk to a camera.

End-to-end (Playwright, in `web/`):
```bash
cd web && npx playwright test
```
The Playwright config spins up **two** webservers: `../build-fresh/guesswork --port 8080` and `npm run dev` on :5173. You must build `guesswork` into `build-fresh/` before running e2e tests, or edit the path in `web/playwright.config.ts`.

## Architecture

### Producer → FrameChannel → Consumer pipeline (`src/core`, `src/producer`, `src/consumer`)

The data plane is a single-producer / multi-consumer "latest-only" channel. One `IProducer` (`SpinnakerProducer`) owns a `FrameChannel`; any number of `IConsumer`s subscribe and pull. When the producer publishes faster than a consumer reads, the channel atomically swaps the held frame and the lagging consumer simply skips ahead — intermediate frames are never queued.

**Frame ownership** is non-trivial and is documented in `src/core/frame.hpp`. Key points:
- `Frame` is intrusively refcounted with a type-erased recycle callback. The owner (pool) passes a `void* this` + a static thunk.
- Buffers are `CVPixelBufferRef`s backed by IOSurfaces (see `core/iosurface_buffer.hpp`) so frames can be shared zero-copy between CPU, Metal (preview), and VideoToolbox (encoder).
- `aux_data_` is an opaque `void*` slot the producer uses for SDK-side state; `core/` never interprets it.
- `SpinnakerUserBufferPool` runs in DMA mode: it pre-registers IOSurface base addresses as Spinnaker user buffers and routes the returned `ImagePtr` into the slot whose base matches. The held `ImagePtr` is what gets released when the `Frame`'s refcount hits zero.
- `FrameChannel::next_frame` uses a try-retain protocol — between loading `latest_` and bumping its refcount, the slot can be recycled; consumers re-load on failure.

### Encoder + WebRTC fan-out (`src/encoder`, `src/server/stream_consumer.*`, `src/server/webrtc_peer.*`)

`StreamConsumer` is an `IConsumer` that owns the encode pipeline: `Mono8ToNv12` (vImage rescale + chroma-neutral UV plane) → `H264Encoder` (VideoToolbox, low-latency Baseline, Annex-B output with SPS/PPS prepended on keyframes) → broadcast to all registered `WebRtcPeer`s. **Encoding only runs when at least one peer is connected**; with zero peers the worker pulls and immediately releases frames. New peers trigger a keyframe request.

WebRTC signaling is non-trickle HTTP: browser POSTs an SDP offer to `/api/stream/offer`, `WebRtcPeer::create_answer` blocks until ICE gathering completes, response is the SDP answer. `WebRtcPeer` close handling routes through `StreamConsumer::closed_pending_` so destruction happens on the encode thread, never inside a libdatachannel callback.

### HTTP server (`src/server`)

Crow (Pimpl'd in `http_server.cpp`) with four route groups: `/api/status` (`PipelineStatsView`, rolling 1s FPS), `/api/stream/offer` (WebRTC signaling), `/api/cameras` (CRUD, talks to `CameraRepository` → `Database`), and `/api/cameras/<id>/calibration*` (recording sessions + uploaded Kalibr camchain YAML). In `GW_HAS_EMBEDDED_WEB` builds, `static_assets.cpp` serves the embedded React bundle with an SPA fallback (unknown non-`/api/*` paths return `index.html`). In non-embedded builds the static route is a no-op.

`Database` is a thread-safe SQLite handle; concurrent HTTP workers serialize through `with_handle()`. Schema (`cameras` table) is created on open via `CREATE TABLE IF NOT EXISTS`. `CameraRepository` throws `DuplicateNameError` on `UNIQUE(name)` conflict; the route layer maps it to HTTP 409.

### Calibration (`src/consumer/rosbag_recording_consumer.*`, `src/consumer/rosbag_writer.*`, `src/server/calibration_supervisor.*`, `src/server/routes_calibration.*`)

Intrinsic camera calibration is driven by Kalibr's `kalibr_calibrate_cameras`, which runs headless in a Docker container (`guesswork/kalibr:latest` — built from `docker/kalibr/Dockerfile`, a vendored copy of Kalibr's upstream `Dockerfile_ros1_20_04`). Our app records a ROS1 v2.0 bag (`calibration.bag`) into `~/.guesswork/calibrations/<session_id>/`, copies the default AprilGrid YAML in as `target.yaml`, and shows the user a `docker run … kalibr_calibrate_cameras …` command that mounts the session dir at `/data`. After Kalibr writes `camchain-calibration.yaml` next to the bag, the user uploads it back through `PUT /api/cameras/<id>/calibration` (Content-Type `application/x-yaml`); the YAML text is stored verbatim in `cameras.calibration_json`.

`RosbagWriter` is a minimal in-app ROS1 v2.0 bag writer: multi-connection (`sensor_msgs/Image` Mono8 + `sensor_msgs/Imu` topics, uncompressed), no ROS install required on the host (see `src/consumer/rosbag_writer.cpp` for the record-framing details). `RosbagRecordingConsumer` is the `IConsumer` (lives in `gw_consumer`) that drives the single-camera intrinsics flow; `MultiTopicBagRecorder` records N cameras + `/imu0` for extrinsics sessions. `CalibrationSupervisor` owns at most one intrinsics consumer per camera (via `CameraSupervisor::frame_channel_for(id)`) plus at most ONE extrinsics session system-wide.

**Camera-IMU extrinsics** (`src/server/kalibr_imu_job.*`, `docker/kalibr/calibrate_imu.sh`): `POST /api/calibration/extrinsics/recording {"camera_ids":[…]}` records the cameras + IMU into one bag with **Teensy-clock timestamps** (requires hw-sync enabled on each camera + Teensy armed with fw=2 telemetry/IMU healthy). The supervisor writes `manifest.json`, `imu.yaml` (from the `imu_config` row, ×10 noise inflation) and, for the single-camera flow, `camchain.yaml` generated from stored intrinsics. `DELETE …/recording` auto-runs `kalibr_calibrate_imu_camera` (pair flow runs `kalibr_calibrate_cameras` first for cam-cam extrinsics); the per-camera blocks of `camchain-imucam-calibration.yaml` are re-keyed to `cam0:` and stored in `cameras.imu_extrinsics_json`. `gw_calibration` (`src/calibration/calibration_store.*`, yaml-cpp PRIVATE) parses camchain YAML into typed structs — downstream consumers (AprilTag, VIO) use those, never raw text. Cameras carry a `role` column (`apriltag`|`vio_left`|`vio_right`, vio roles unique) patched via `PUT /api/cameras/<id>`.

### AprilTag detection (`src/apriltag`, `src/server/apriltag_supervisor.*`)

`gw_apriltag` turns role='apriltag' cameras into field-frame robot-pose measurements on a `TagPoseBus` (`MeasurementBus<TagPoseMeasurement>`, 6×6 covariance in GTSAM Pose3 body-tangent convention `[ω, t]`). Per camera, an `AprilTagConsumer` (IConsumer) detects tag36h11 **zero-copy on the locked Mono8 IOSurface** (one detector per consumer, decimate 2, 4 threads ≈ 12 ms/frame at 2048×1536 on M4), then `tag_pose_estimator` (pure, OpenCV cpp-only) undistorts corner points, solves SQPNP (≥2 tags, field-frame corners → `T_cam_field` directly) or IPPE_SQUARE (single tag, ambiguity-gated), chains `T_field_robot = inverse(T_cam_field) · T_cam_imu · inverse(T_robot_imu)`, and propagates σ=0.7 px corner noise through a numeric Jacobian. **All frame conventions are documented in the comment block atop `src/apriltag/tag_pose_estimator.cpp`** — note OpenCV's IPPE_SQUARE object frame is image-aligned (+Y down, +Z into the scene; empirically verified), so `det->p[0..3]` pairs directly with the object points.

Consumer lifecycle rides `CameraSupervisor::register_consumer_factory`: factories run inside the slot start/stop path (attach after StreamConsumer, detach before producer destruction — never a dangling FrameChannel; survives USB unplug). Role/calibration changes rebuild the extra consumers in place via `on_camera_updated`; manual calibration uploads and Kalibr jobs notify it. `ApriltagSupervisor` owns the bus + hot-swappable `SharedTagConfig` (active field layout, `T_robot_imu` from `imu_config.t_imu_robot_json` — schema `{"T_robot_imu": [[4×4 row-major]]}`, parsed/validated by `gw::calib::parse_t_robot_imu`). Calibration source preference: `imu_extrinsics_json` (refined intrinsics + `T_cam_imu`) → `calibration_json` (intrinsics-only: detection + per-tag status ranges run, publish gated as `no_extrinsics_chain`).

Field layouts: `field_layouts` table (verbatim WPILib AprilTagFieldLayout JSON, exactly one active, enforced in `FieldLayoutRepository`), auto-seeded from `data/field_layouts/` (`GW_FIELD_LAYOUT_DEFAULT`) on first boot. Adding this table was additive — no `--reset-db` was needed. API: `GET/POST/DELETE /api/field-layouts`, `POST /api/field-layouts/<id>/activate`, `GET /api/apriltag/status` (per-camera rates, latency, gating reasons, last tags + ranges). Deps: AprilRobotics `apriltag` via FetchContent (forced `-O2` even in Debug — an -O0 detector costs 50 ms/frame), OpenCV via Homebrew (`brew install opencv`), both linked PRIVATE — no OpenCV/apriltag types in public headers.

### Stereo VIO (`src/vio`, `src/server/vio_supervisor.*`, `cmake/openvins.cmake`)

`gw_vio` runs **OpenVINS** (stereo+IMU MSCKF) on the `vio_left`/`vio_right` cameras, publishing `VioOdometry{t_ns, epoch, T_odom_imu, cov6×6 (GTSAM body-tangent, same convention as TagPoseMeasurement), tracked_features}` on a `VioBus`. **Fusion contract: only difference consecutive poses with EQUAL `epoch`** — the odom origin resets on every (re)init and `epoch` increments; absolute VIO poses are never fused.

OpenVINS is built from source by `cmake/openvins.cmake` (ExternalProject, pinned master SHA — v2.7 doesn't compile against Homebrew Ceres 2.2; macOS arm64 fixes: drop the `boost_system` component for Boost ≥ 1.90, `-include cassert`). Build deps: `brew install eigen boost ceres-solver` (+ opencv). It produces ONE `libov_msckf_lib.dylib`; headers under `openvins-install/include/open_vins/`. **OpenVINS is GPL-3.0** — distributed guesswork binaries are effectively GPLv3. `use_aruco=false` is mandatory at runtime (built with ENABLE_ARUCO_TAGS=OFF).

Data flow: per-camera `VioFeederConsumer`s (ConsumerFactory products, slot-lifecycle-safe) fast-copy frames into the `StereoSyncPairer` (exact `camera_ts_ns` equality match — both cams share one trigger group; zero-stamp frames dropped); one `OpenVinsRunner` thread drains pairs + the `ImuBus` and feeds `VioManager` (the camera path has no internal locking — single-threaded feeding by design). Calibration is consumed programmatically: `VioConfigBuilder` maps stored camchain-imucam blocks + `imu_config` noise into `VioManagerOptions` (Kalibr `T_cam_imu` → OpenVINS `q_ItoC`+`p_IinC` with NO inversion; downsampling halves intrinsics in the mapping, matching upstream's YAML loader). `VioSupervisor` gates enablement (both roles, parseable extrinsics, `guesswork_meta.reprojection_error_std_px ≤ vio_config.max_reproj_std_px`) and rebuilds the runner on fingerprint changes; divergence (feature collapse / covariance explosion / >2 s frame gap) auto-reinits. API: `GET /api/vio/status`, `GET/PUT /api/vio/config` (single-row `vio_config` table), `POST /api/vio/restart`.

### CAN bridge (`firmware/src/can_bridge.*`, `src/core/rio_clock_sync.*`, `src/server/routes_can.*`)

fw=3 adds a CAN bridge on Teensy **CAN3** (pins 30/31, TJA1051T/3 — the only FD-capable FlexCAN on Teensy 4.x). The controller streams WPILib **ChassisSpeeds** (robot-frame vx/vy/ω + `rio_time_us` FPGA sample stamp — drive-type-agnostic, no wheel math in this repo) at 50–100 Hz → CAN RX ISR stamps arrival with `now_us64()` → ODOM telemetry (0x03) → host publishes `ChassisSpeeds` on `TeensyManager::odom_bus()`. Downlink: `TeensyManager::send_pose` writes a POSE packet (0x10) host→Teensy on the **telemetry CDC's reverse direction** → one FD frame / classic XY+THETA pair with a rolling counter (controller staleness rule: counter frozen >200 ms). **The wire contract is `firmware/src/can_payloads.h`** — freestanding (no Arduino.h), compiled by firmware, `gw_server`, and `gw_tests` alike (`docs/can-protocol.md` is the prose copy; change together).

Two runtime modes in one firmware, persisted in the single-row `can_config` table (additive — no `--reset-db`) and pushed as `CAN_MODE mode=off|classic|fd` on update + every reconnect: **`roborio`** = classic CAN 2.0 @ 1 Mbps (8-byte frames, int16 milli-units, separate STAMP frame paired by counter, RIO u32 µs wrap-extended on the Teensy) and **`systemcore`** = CAN FD 1 Mbps/4 Mbps (24-byte float32 frames; TJA1051 caps at 5 Mbps). FlexCAN_T4 gotchas (can_bridge.cpp is the ONLY TU that may include FlexCAN_T4.h): both class constructors set the per-TU ISR dispatch pointer `_CAN3` at static init, so `set_mode` reassigns it before `begin()`; **never call `events()`** — it permanently moves RX callbacks out of ISR context and ruins arrival stamps; FD needs `CLK_80MHz` + `setRegions(32)`; runtime mode switching is unofficial (power-cycle fallback; resync self-heals).

Time sync lives ONLY on the host: `gw::RioClockSync` (pure, unit-tested) fits `teensy ≈ rio + offset(t)` from per-250 ms bucket minima of (arrival − rio) over a 6 s window — min rejects one-sided transit jitter, the fit tracks crystal drift, and backward-rio-jump / >50 ms offset-step resets handle controller reboots (~1 s re-warm-up, arrival-stamp fallback meanwhile). `ChassisSpeeds.t_ns` = mapped RIO sample time when healthy, else CAN arrival. API: `GET/PUT /api/can/config` (PUT returns `pushed`/`push_error`; DB update succeeds with the Teensy offline), `GET /api/can/status` (mode, odom rate, counters, clock_sync), `POST /api/can/pose` (bench downlink until Phase 6). fw=2↔fw=3 compat: heartbeat parse is length-gated (17 vs 34 bytes), CAN_MODE push is gated on `fw_version ≥ 3`.

### Fusion engine (`src/fusion`, `src/server/fusion_supervisor.*`, `cmake/gtsam.cmake`)

`gw_fusion` runs one **GTSAM `IncrementalFixedLagSmoother`** (iSAM2-backed, ~2 s lag, Pose3 states every ≥25 ms on the Teensy clock) fusing TagPoseBus (absolute `PriorFactor` + Huber, **Mahalanobis-gated** χ²₆ against the smoother marginal), VioBus (equal-epoch relative deltas conjugated into the robot frame via `T_robot_imu`, Adjoint covariance transport, `BetweenFactor` + Huber) and OdomBus chassis speeds (integrated body twist `BetweenFactor` + Cauchy, slip-robust, soft z/roll/pitch planarity). Output: planar constant-twist extrapolation to "Teensy-now" (`TeensyNowEstimator`, host-steady↔Teensy EMA) capped at `max_extrapolation_ms` → `TeensyManager::send_pose` at `output_hz`. All three buses already publish covariances in GTSAM Pose3's retract convention — they flow into the graph **unchanged**.

GTSAM is built from source by `cmake/gtsam.cmake` (ExternalProject, pinned **4.3a1**, BSD license): Boost-free build (`GTSAM_ENABLE_BOOST_SERIALIZATION=OFF`, `GTSAM_USE_BOOST_FEATURES=OFF` — dodges the Homebrew Boost 1.90 hazard; if it regresses, both flags ON are safe at this tag), **`IncrementalFixedLagSmoother` lives in core gtsam at 4.3a1** (unstable OFF), system Eigen, TBB off, `-include cassert` (same libc++ fix as OpenVINS). GTSAM appears ONLY inside `fusion_engine.cpp`'s Pimpl.

Engine rules worth knowing: single-threaded by design (`FusionSupervisor`'s engine thread owns it; 3 bus drainers feed an internal queue; tests drive it directly); init = geometric medoid of the first 5 kTeensy tags (kHost tags always rejected); **VIO betweens are emitted lazily** onto in-lag historical keys (OpenVINS publishes ~50–100 ms behind the pulse stamp); every new key gets an odom between or a constant-velocity bridge ×10 noise (connectivity invariant — prevents `IndeterminantLinearSystemException`, plus QR factorization + try/catch→auto-reinit); VIO per-sample delta noise comes from config sigmas, NOT published-cov differencing (no cross-cov ⇒ not PSD-safe; published cov only health-gates); collision mode (>50% of the last `collision_window` gated tags rejected) opens the gate and inflates VIO/odom ×`collision_inflation` — tags are the truth source; full reinit on pos-std > `reinit_pos_std_m`, solver throw, NaN, or 5 s unresolved collision. `T_robot_imu` unset disables VIO ingestion only. IMU preintegration is a reserved Phase 7 seam (`feed_imu` commented in fusion_engine.hpp). API: `GET /api/fusion/status`, `GET/PUT /api/fusion/config` (single-row `fusion_config` table, additive; PUT returns `restarted` — engine-relevant changes rebuild, output_hz/max_extrapolation are live-applied), `POST /api/fusion/reset`. An imu-config PUT also reloads fusion (T_robot_imu gating).

### Hardening & ops (Phase 7)

- **Latency**: `/api/fusion/status.latency` carries four stages (`tag_pulse_to_fusion`, `queue_wait`, `solve`, `pose_staleness`); **`pose_staleness` p95 is the trigger-pulse→pose-on-CAN headline (< 50 ms target)**. Shared ring helper: `gw::LatencyStats` (src/core/latency_stats.hpp, not thread-safe — owner locks). The `TeensyNowEstimator` is fed by both the odom AND tag drainers (tag feed is early-biased by detect latency — only shortens extrapolation; it keeps teensy-now alive through CAN-odom death).
- **Degraded modes**: `mode` string from pure `derive_fusion_mode` (src/server/fusion_mode.hpp); the matrix lives in docs/pose_pipeline.md §6 and its rows mirror `tests/test_fusion_mode.cpp` 1:1. No publish before init and no publish while teensy-now is unwarmed are deliberate (CAN counter staleness signals both).
- **Allan refinement** (`/api/imu/allan/*`): record a static IMU log (32-byte binary records → `~/.guesswork/imu_logs/`), analyze (`gw::compute_allan` — overlapping ADEV, N at τ=1 s on the −1/2 slope, K at τ=3 s on +1/2, Kalibr units), apply worst-axis values into imu_config. ≥3 h static data for a credible K (overnight recommended); motion/short-data/fit warnings come back in the response.
- **Config snapshot** (`GET /api/config/export`, `POST /api/config/import`): full robot identity incl. calibration blobs (verbatim strings). Import = non-destructive merge (cameras by serial w/ pin pre-clear for swaps, groups/layouts by name, snapshot-active layout activated last), per-section error report, then propagation (CameraSupervisor notifications → apriltag/vio/fusion reloads → CAN-mode push). Logic is HTTP-free in src/server/config_snapshot.{hpp,cpp} (unit-tested round trip).
- **Soak**: `scripts/soak_check.sh` polls status endpoints → CSV + PASS/FAIL verdict; the sanitizer-soak procedure is bring-up Stage 8. **GTSAM/OpenVINS ExternalProjects do NOT inherit sanitizer flags** (independent configures — uninstrumented dylibs; acceptable: engine thread is GTSAM's only user). The no-hardware TSAN target is `gw_tests --gtest_filter='FusionSupervisor*'`.

### React frontend (`web/`)

Vite + React 18 + react-router. Three pages: `/` (StreamPage, the WebRTC video), `/cameras` (CRUD UI), and `/cameras/:id/calibrate` (CalibratePage — live preview + record/stop, suggested Kalibr docker command, camchain YAML upload; `js-yaml` parses the camchain client-side for display). API client wrappers live in `web/src/api/`. The dev server proxies `/api` to `:8080`; the production build is embedded into the binary by CMake — there is no separate static-file deploy step.

## Conventions worth knowing

- Namespaces: `gw::` for core/producer/consumer/encoder/app, `gw::server::` for HTTP/DB/WebRTC, `gw::encoder::` for the encode primitives.
- Headers that would pull in Crow, libdatachannel, Spinnaker, or VideoToolbox into downstream TUs use Pimpl (`HttpServer`, `WebRtcPeer`, `SpinnakerProducer`, `H264Encoder` exposes VT through its `.hpp` but is consumed only by `StreamConsumer`). Preserve this when adding code — these SDKs add minutes of compile time per consumer.
- `.mm` files are Objective-C++ with ARC enabled (`-fobjc-arc` is added per `COMPILE_LANGUAGE:OBJCXX`). Plain `.cpp` is C++20, no ARC.
- The project-wide warning flags are `-Wall -Wextra -Wpedantic`. `gw_server` locally relaxes `-Wno-pedantic -Wno-deprecated-literal-operator` because Crow's `"GET"_method` literal operator trips them.
- Sanitizer-friendly env for ASan runs is in `.vscode/launch.json` (`MallocNanoZone=0`, `detect_leaks=0` because of CoreFoundation noise). TSan runs set `halt_on_error=1:second_deadlock_stack=1`.
