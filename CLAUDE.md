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

`conanfile.txt` disables most optional OpenCV features (wayland/ffmpeg/gstreamer/tiff/webp/gtk/etc.). Adding further OpenCV features will likely reintroduce `xorg/system` and other system pulls — verify with `conan graph info .` before enabling.

## Architecture

The CMake libraries are split by runtime area:

- **`posest_core`** — the reusable pipeline plumbing: interfaces (`IFrameProducer`, `IFrameConsumer`), the `Frame` value type, the `LatestFrameSlot` mailbox, and the `ProducerBase` / `ConsumerBase` threaded base classes. Depends on OpenCV for `cv::Mat`.
- **`posest_camera`** — generic camera abstractions: `CameraConfig` structs and the `CameraProducer` abstract base class that sits between `ProducerBase` and concrete backends. The `CameraProducer` enforces an open → configure → stream → grab → stop → close lifecycle.
- **`posest_config`** — runtime configuration persistence and validation. SQLite is the only persistent config backend; `SqliteConfigStore` creates/migrates the schema and atomically saves/loads `RuntimeConfig`.
- **`posest_runtime`** — runtime graph construction, camera/pipeline factory interfaces, and web/control facade.
- **`posest_fusion`** — measurement ingestion and fusion output boundaries.
- **`posest_teensy`** — Teensy USB protocol codec and fusion output adapter.
- **`posest_v4l2`** — V4L2 camera backend (Linux only): `V4L2Producer` opens a device by stable path (`/dev/v4l/by-id/` or `/dev/v4l/by-path/`), applies UVC controls from config, uses MMAP streaming with zero-copy decode input, and converts V4L2 kernel timestamps to `steady_clock`. Depends on `posest_camera` + system V4L2 headers.
- **`posest_mock`** — `MockProducer` / `MockConsumer` used by the test suite to exercise the core pipeline contracts.

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

Subclass `CameraProducer` and implement the 7 hooks (`openDevice`, `applyFormat`, `applyControls`, `startStream`, `grabFrame`, `stopStream`, `closeDevice`). Do **not** override `captureOne()` — it is `final` and delegates to `grabFrame()`. Create a new library (e.g. `posest_avfoundation`) following the `posest_v4l2` pattern.
