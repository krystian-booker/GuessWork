# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Platform constraints

macOS / Apple Silicon only — CMake hard-fails on non-Apple. The pipeline depends on CoreVideo / IOSurface / Metal / VideoToolbox, and a system install of the FLIR **Spinnaker SDK** at `/usr/local/{include,lib}` is required to link `gw_producer` (and therefore everything that pulls it in, including `guesswork` and `gw_tests`). Homebrew OpenSSL at `/opt/homebrew/opt/openssl@3` is used by the vendored libdatachannel WebRTC stack.

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

### React frontend (`web/`)

Vite + React 18 + react-router. Three pages: `/` (StreamPage, the WebRTC video), `/cameras` (CRUD UI), and `/cameras/:id/calibrate` (CalibratePage — live preview + record/stop, suggested Kalibr docker command, camchain YAML upload; `js-yaml` parses the camchain client-side for display). API client wrappers live in `web/src/api/`. The dev server proxies `/api` to `:8080`; the production build is embedded into the binary by CMake — there is no separate static-file deploy step.

## Conventions worth knowing

- Namespaces: `gw::` for core/producer/consumer/encoder/app, `gw::server::` for HTTP/DB/WebRTC, `gw::encoder::` for the encode primitives.
- Headers that would pull in Crow, libdatachannel, Spinnaker, or VideoToolbox into downstream TUs use Pimpl (`HttpServer`, `WebRtcPeer`, `SpinnakerProducer`, `H264Encoder` exposes VT through its `.hpp` but is consumed only by `StreamConsumer`). Preserve this when adding code — these SDKs add minutes of compile time per consumer.
- `.mm` files are Objective-C++ with ARC enabled (`-fobjc-arc` is added per `COMPILE_LANGUAGE:OBJCXX`). Plain `.cpp` is C++20, no ARC.
- The project-wide warning flags are `-Wall -Wextra -Wpedantic`. `gw_server` locally relaxes `-Wno-pedantic -Wno-deprecated-literal-operator` because Crow's `"GET"_method` literal operator trips them.
- Sanitizer-friendly env for ASan runs is in `.vscode/launch.json` (`MallocNanoZone=0`, `detect_leaks=0` because of CoreFoundation noise). TSan runs set `halt_on_error=1:second_deadlock_stack=1`.
