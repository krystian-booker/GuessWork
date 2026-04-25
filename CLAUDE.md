# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & test (Conan 2 + CMake, out-of-source under `build/Release`)

First-time / dependency change:

```bash
conan install . --build=missing -s build_type=Release
```

This generates `CMakeUserPresets.json` (gitignored) pulling in the Conan preset at `build/Release/generators/CMakePresets.json`.

Configure, build, test:

```bash
cmake --preset conan-release
cmake --build --preset conan-release -j
ctest --preset conan-release --output-on-failure
```

Run a single GTest case directly (faster iteration than ctest):

```bash
./build/Release/posest_tests --gtest_filter='MockPipeline.SlowConsumerDropsButProducerIsNotBlocked'
```

### OpenCV + Conan on headless Linux

`conanfile.txt` disables most optional OpenCV features (wayland/ffmpeg/gstreamer/tiff/webp/gtk/etc.) and disables TBB on GTSAM. Adding further OpenCV features will likely reintroduce `xorg/system` and other system pulls — verify with `conan graph info .` before enabling. Top-level deps are: `opencv`, `gtest`, `sqlite3`, `nlohmann_json`, `apriltag`, `yaml-cpp`, `gtsam`.

### Teensy 4.1 firmware

Firmware lives under `firmware/teensy41/` (PlatformIO). The host-side wire format in `include/posest/teensy/Protocol.h` is authoritative — change it there first, then mirror in firmware. Build/upload/monitor:

```bash
pio run -d firmware/teensy41
pio run -d firmware/teensy41 -t upload
pio device monitor -d firmware/teensy41 -b 921600
```

## Architecture

The CMake libraries are split by runtime area (see `CMakeLists.txt` for full link graph):

- **`posest_core`** — the reusable pipeline plumbing: interfaces (`IFrameProducer`, `IFrameConsumer`), the `Frame` value type, the `LatestFrameSlot` mailbox, the `ProducerBase` / `ConsumerBase` threaded base classes, and the `MeasurementBus` (typed pub/sub for non-frame measurements like IMU/wheel/odometry). Depends on OpenCV for `cv::Mat`.
- **`posest_camera`** — generic camera abstractions: `CameraConfig` structs and the `CameraProducer` abstract base class that sits between `ProducerBase` and concrete backends. The `CameraProducer` enforces an open → configure → stream → grab → stop → close lifecycle.
- **`posest_config`** — runtime configuration persistence and validation. SQLite is the only persistent config backend; `SqliteConfigStore` creates/migrates the schema and atomically saves/loads `RuntimeConfig`. Also parses Kalibr/field-layout YAML/JSON via `CalibrationParsers`.
- **`posest_runtime`** — runtime graph construction (`RuntimeGraph`), camera/pipeline factory interfaces (`Factories.h`), telemetry snapshot, and the web/control facade (`WebService`).
- **`posest_pipelines`** — vision pipeline implementations: `VisionPipelineBase` (a `ConsumerBase` specialization that publishes onto the `MeasurementBus`), `AprilTagPipeline`, and the placeholder pipelines used as defaults. Links `apriltag`.
- **`posest_calibration`** — dataset recording (`CalibrationRecorder`) and Kalibr Docker invocation helpers used by the daemon's calibration subcommands.
- **`posest_fusion`** — measurement ingestion (`FusionService` subscribes to the `MeasurementBus`) and fusion output sink boundary (`IFusionOutputSink`). Links GTSAM.
- **`posest_teensy`** — Teensy USB protocol codec (`Protocol.h`), `SerialTransport` / `FakeSerialTransport`, and `TeensyService` which adapts protocol frames to the `MeasurementBus` and consumes fused poses.
- **`posest_v4l2`** — V4L2 camera backend (Linux only): `V4L2Producer` opens a device by stable path (`/dev/v4l/by-id/` or `/dev/v4l/by-path/`), applies UVC controls from config, uses MMAP streaming with zero-copy decode input, and converts V4L2 kernel timestamps to `steady_clock`. Includes `V4L2DeviceEnumerator`. Depends on `posest_camera` + system V4L2 headers.
- **`posest_daemon_lib`** + **`posest_daemon`** — the single-process runtime: `Daemon.cpp` owns lifecycle (`Created → LoadedConfig → Built → Running → Stopping → Stopped`), `ProductionFactories` wires real producers/pipelines (V4L2 + AprilTag), and `app/posest_daemon.cpp` is the executable. The same binary also dispatches one-shot config subcommands (`calibrate-camera`, `import-field-layout`, `record-kalibr-dataset`, `make-kalibr-bag`, `calibrate-camera-imu`) — see `daemonUsage()` for the full surface.
- **`posest_mock`** — `MockProducer` / `MockConsumer` / `MockCameraProducer` used by the test suite to exercise the core pipeline and camera contracts.

Deeper feature-level architecture docs live in `docs/features/` (e.g. `producer-consumer-architecture.md`, `camera-producer.md`). `docs/ignore_this_folder/` is exactly what it sounds like — don't read or edit it.

### The latency contract (important)

The entire design exists to keep the producer thread unblocked while still letting consumers process "the newest available frame" when they're ready. The moving parts:

1. **`ProducerBase`** owns the capture thread. Subclasses implement only `captureOne(out, out_capture_time)`. The base handles fan-out, sequence numbering, and timestamp fallback.
2. **`IFrameConsumer::deliver()` MUST be non-blocking.** `ProducerBase::runLoop` calls it synchronously from the capture thread — any blocking call there would stall the producer and every other consumer subscribed to it. `ConsumerBase::deliver` satisfies this by forwarding straight into a `LatestFrameSlot`.
3. **`LatestFrameSlot`** is a single-slot mailbox with **drop-oldest** semantics. `put()` never blocks; if an unread frame is pending, it is dropped (and `droppedCount()` is bumped). `take()` blocks until a frame arrives or `shutdown()` is called. This is why consumers see sequence gaps — it's the intended behavior, not a bug.
4. **`ConsumerBase`** owns a worker thread that loops `slot_.take()` → `process(frame)`. Subclasses only implement `process()`, which may be arbitrarily slow.

Wiring order is fixed: `addConsumer()` must happen before `start()`. Hot-add during capture is explicitly not supported — `runLoop` snapshots the subscriber list once at startup.

### Timestamp discipline

All `Frame::capture_time` values live in the `std::chrono::steady_clock` domain, regardless of source. This is load-bearing for cross-camera math (multi-camera fusion, VIO). Contract for `ProducerBase` subclasses:

- If the backend exposes a native timestamp (V4L2 buffer timestamp with `V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC`, GenICam/PTP hardware stamp, vendor SDK metadata), **convert it to `steady_clock` and assign to `out_capture_time`**. For cameras with their own epoch, do a one-shot startup calibration (capture `steady_clock::now()` and the camera tick together, store the offset) and apply it per-frame.
- If `out_capture_time` is left as `std::nullopt`, the base stamps `steady_clock::now()` immediately after `captureOne()` returns. This is the userspace-approximation fallback and is subject to scheduler jitter — real producers should always supply a hardware/kernel stamp when one exists.

`MockProducer` demonstrates the subclass-supplied path by reporting the *scheduled* deadline (when the "shutter" fires) rather than the post-capture return time.

### Startup / shutdown ordering

The canonical sequence (mirrored by tests in `test/test_mock_pipeline.cpp`):

1. Construct producer and consumers.
2. `producer.addConsumer(c)` for each.
3. `consumer.start()` for each (workers waiting on mailboxes).
4. `producer.start()` (capture thread begins fanning out).
5. `producer.stop()` — joins the capture thread.
6. Brief sleep to let consumers drain any in-flight frame.
7. `consumer.stop()` — calls `slot_.shutdown()`, which unblocks the waiting `take()` and returns `nullptr`, causing the worker to exit.

`ProducerBase::~ProducerBase` and `ConsumerBase::~ConsumerBase` both call `stop()`, so leaked instances still join cleanly — but don't rely on destruction ordering for correctness; stop explicitly.

### Camera configuration (SQLite)

Runtime configuration is loaded through `posest::config::IConfigStore`; the persistent implementation is `SqliteConfigStore`. The store uses versioned migrations, strict validation before save, and atomic full-config replacement in a single transaction. Camera device paths should use stable `/dev/v4l/by-id/` or `/dev/v4l/by-path/` names when possible.

Supported control names: `exposure_auto`, `exposure_absolute`, `gain`, `brightness`, `contrast`, `saturation`, `sharpness`, `white_balance_auto`, `white_balance_temperature`, `backlight_compensation`, `focus_auto`, `focus_absolute`, `power_line_frequency`.

### Adding new camera backends

Subclass `CameraProducer` and implement the 7 hooks (`openDevice`, `applyFormat`, `applyControls`, `startStream`, `grabFrame`, `stopStream`, `closeDevice`). Do **not** override `captureOne()` — it is `final` and delegates to `grabFrame()`. Create a new library (e.g. `posest_avfoundation`) following the `posest_v4l2` pattern, and register it in `ProductionCameraFactory` so the daemon can build it.

### MeasurementBus and non-frame data

Frames flow producer → consumer through `LatestFrameSlot` (drop-oldest, single slot). Non-frame measurements (IMU samples, wheel/robot odometry, AprilTag detections) flow through `MeasurementBus` instead — a typed pub/sub fan-out used by `TeensyService` (publisher), vision pipelines (publishers, via `VisionPipelineBase`), and `FusionService` (subscriber). When wiring a new measurement source or sink, prefer the bus over ad-hoc shared state, and follow the same non-blocking publisher rule that applies to `IFrameConsumer::deliver()`.

### Code style and conventions

C++20, warnings-as-listed in `CMakeLists.txt` (`-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion`). 4-space indent, braces same-line. Types/test suites `PascalCase`, functions `camelCase`, namespaces lowercase, private members trailing underscore (`running_`). Keep platform-specific code isolated to its backend library and CMake-guarded (see `posest_v4l2`). Public API headers go in `include/posest/...` mirroring the `src/<area>/...` layout.
