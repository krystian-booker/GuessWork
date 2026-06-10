# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

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

### React frontend (`web/`)

Vite + React 18 + react-router. Three pages: `/` (StreamPage, the WebRTC video), `/cameras` (CRUD UI), and `/cameras/:id/calibrate` (CalibratePage — live preview + record/stop, suggested Kalibr docker command, camchain YAML upload; `js-yaml` parses the camchain client-side for display). API client wrappers live in `web/src/api/`. The dev server proxies `/api` to `:8080`; the production build is embedded into the binary by CMake — there is no separate static-file deploy step.

## Conventions worth knowing

- Namespaces: `gw::` for core/producer/consumer/encoder/app, `gw::server::` for HTTP/DB/WebRTC, `gw::encoder::` for the encode primitives.
- Headers that would pull in Crow, libdatachannel, Spinnaker, or VideoToolbox into downstream TUs use Pimpl (`HttpServer`, `WebRtcPeer`, `SpinnakerProducer`, `H264Encoder` exposes VT through its `.hpp` but is consumed only by `StreamConsumer`). Preserve this when adding code — these SDKs add minutes of compile time per consumer.
- `.mm` files are Objective-C++ with ARC enabled (`-fobjc-arc` is added per `COMPILE_LANGUAGE:OBJCXX`). Plain `.cpp` is C++20, no ARC.
- The project-wide warning flags are `-Wall -Wextra -Wpedantic`. `gw_server` locally relaxes `-Wno-pedantic -Wno-deprecated-literal-operator` because Crow's `"GET"_method` literal operator trips them.
- Sanitizer-friendly env for ASan runs is in `.vscode/launch.json` (`MallocNanoZone=0`, `detect_leaks=0` because of CoreFoundation noise). TSan runs set `halt_on_error=1:second_deadlock_stack=1`.
