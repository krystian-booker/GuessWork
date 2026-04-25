# Camera Producer Subsystem

**Status:** Feature-complete against the stated requirements. Capability descriptors, live control updates, reconnect-on-unplug, hardware-trigger abstraction, and device enumeration are all implemented. Pixel-format coverage has been expanded from 2 to 9 formats.

This document covers the generic frame producer interface, the camera-specific lifecycle abstraction layered on top of it, and the V4L2 implementation that ships today. It is structured as: requirements → architecture → what is implemented → status against the requirements.

---

## 1. Requirements (target behavior)

The system is a **producer/consumer pipeline** where lowest-possible latency from sensor exposure to consumer is the primary design constraint. Cameras are producers; vision pipelines (and recording, telemetry, etc.) are consumers.

The following requirements are the bar this document is reviewing against:

1. A **single generic producer interface** describes how every camera works. Concrete backends (V4L2, GenICam, Spinnaker, AVFoundation, MIPI CSI, …) implement that interface.
2. The rest of the application is **completely backend-agnostic**. No `if (type == "v4l2")` branches outside the factory.
3. The interface gives **full control of the camera** — including:
   - auto/manual exposure
   - auto/manual gain
   - auto/manual white balance
   - other useful settings (brightness, contrast, sharpness, focus, backlight comp, power-line frequency, …)
4. The interface provides:
   - frame retrieval
   - initialization / device open
   - destruction / device close
   - **reinitialization on unplug**
5. The interface declares whether the camera is **hardware-triggered** or not.
6. If a backend does not support a feature (e.g. hardware trigger), it simply does not implement that method. A **capabilities descriptor** is returned so the UI can render the right controls without per-backend `if` trees.

---

## 2. Architecture

### 2.1 Producer/consumer split

```
                           ┌─ ConsumerA (LatestFrameSlot ─→ worker thread ─→ process())
IFrameProducer ─ deliver()─┼─ ConsumerB (LatestFrameSlot ─→ worker thread ─→ process())
   (capture thread)        └─ ConsumerC ...
```

- `IFrameProducer` (`include/posest/IFrameProducer.h:14`) — pure interface: `id()`, `addConsumer()`, `start()`, `stop()`. Knows nothing about cameras.
- `IFrameConsumer` (`include/posest/IFrameConsumer.h:15`) — pure interface: `deliver(FramePtr)` is invoked synchronously on the producer's capture thread and **must be non-blocking**.
- `Frame` (`include/posest/Frame.h:12`) — value type with `capture_time` (steady_clock), monotonic `sequence`, source `camera_id`, and `cv::Mat image`. Distributed to consumers as `std::shared_ptr<const Frame>`.

### 2.2 The latency contract

The hard rule: **the producer's capture thread is never allowed to stall**. This is what the moving parts enforce:

1. **`ProducerBase`** (`include/posest/ProducerBase.h:26`, `src/core/ProducerBase.cpp`) — owns the capture thread and the subscriber list, assigns sequence numbers, and applies the timestamp fallback. Subclasses implement only `captureOne(out, out_capture_time)`. The subscriber list is snapshotted once at `start()`; hot-add during capture is unsupported by design.
2. **`LatestFrameSlot`** (`include/posest/LatestFrameSlot.h:22`, `src/core/LatestFrameSlot.cpp`) — single-slot mailbox with **drop-oldest** semantics:
   - `put()` never blocks. If an unread frame is pending, it is replaced and `dropped_` is incremented.
   - `take()` blocks until a frame arrives or `shutdown()` is called.
   - This is the core latency primitive: a slow consumer can never back-pressure the capture thread, and when the consumer is ready it always gets the **newest** available frame.
3. **`ConsumerBase`** (`include/posest/ConsumerBase.h:18`, `src/core/ConsumerBase.cpp`) — owns the consumer's worker thread, forwards `deliver()` straight into its `LatestFrameSlot`, and drains it on the worker. Subclasses implement only `process(const Frame&)`, which may be arbitrarily slow.

Consequence consumers must understand: **sequence numbers can have gaps**. That is the design, not a bug. `LatestFrameSlot::droppedCount()` reports how many frames were skipped per consumer.

### 2.3 Timestamp discipline

All `Frame::capture_time` values are in `std::chrono::steady_clock`. Cross-camera math (multi-camera fusion, VIO) requires a single time domain, so backends that expose a native timestamp **must** convert it into `steady_clock` and assign it to `out_capture_time`. If left `nullopt`, `ProducerBase::runLoop` (`src/core/ProducerBase.cpp:55`) stamps `steady_clock::now()` immediately after `captureOne()` returns — that is the userspace fallback and is subject to scheduling jitter.

For backends with their own epoch, the contract calls for a one-shot calibration at startup (`steady_clock::now()` and the camera tick captured together) plus per-frame offset application.

### 2.4 Lifecycle ordering

Canonical sequence (mirrored by `test/test_mock_pipeline.cpp`):

1. Construct producer and consumers.
2. `producer.addConsumer(c)` for every consumer.
3. `consumer.start()` for each — workers begin waiting on their mailboxes.
4. `producer.start()` — capture thread begins fanning out.
5. `producer.stop()` — joins capture thread.
6. Brief drain.
7. `consumer.stop()` — calls `slot_.shutdown()`, unblocks `take()` returning `nullptr`, worker exits.

Both base destructors call `stop()` for safety, but explicit ordering is required for deterministic teardown.

---

## 3. The generic camera abstraction

Two layers sit between `ProducerBase` and any concrete backend:

### 3.1 `CameraConfig` — the configuration value type

`include/posest/CameraConfig.h:21`

```c++
struct CameraFormatConfig {
    int width;
    int height;
    double fps;
    std::string pixel_format;   // "mjpeg", "yuyv", ...
};

struct CameraControlEntry {
    std::string name;           // canonical control name (see §3.3)
    std::int32_t value;
};

struct CameraConfig {
    std::string id;             // logical camera id (used as Frame::camera_id)
    std::string type;           // backend selector — "v4l2", future: "genicam", "spinnaker"
    std::string device;         // backend-specific path/URI (e.g. /dev/v4l/by-id/...)
    bool enabled;
    CameraFormatConfig format;
    std::vector<CameraControlEntry> controls;
};
```

`CameraConfig` is **the only structure backends receive** from the rest of the app. The factory uses `type` to dispatch to a backend; everything else (`device`, `format`, `controls`) is interpreted by the backend.

This is a clean separation today: `type` strings live in exactly one place (`ProductionCameraFactory::createCamera()`, `src/runtime/ProductionFactories.cpp:43`) and the rest of the runtime passes `CameraConfig` and `IFrameProducer` around without ever inspecting `type`.

### 3.2 `CameraProducer` — the lifecycle abstraction

`include/posest/CameraProducer.h:20`, `src/camera/CameraProducer.cpp`

Abstract intermediate between `ProducerBase` and concrete backends. It enforces a **fixed lifecycle**:

```
start():  openDevice → applyFormat → applyControls → startStream → spawn capture thread
stop():   join capture thread → stopStream → closeDevice
```

Hooks subclasses must implement (all `protected virtual`):

| Hook | Purpose |
|------|---------|
| `openDevice()` | Acquire device handle (open file descriptor, bind GenICam node, …). |
| `applyFormat()` | Negotiate width/height/fps/pixel format. |
| `applyControls()` | Apply each entry in `config().controls`. |
| `startStream()` | Begin streaming (allocate buffers, start DMA, …). |
| `grabFrame(out, ts)` | Block until next frame ready, fill `out`, optionally set `ts`. Return `false` to end stream. |
| `stopStream()` | Stop streaming. |
| `closeDevice()` | Release handle and buffers. |

`captureOne()` is `final` and simply forwards to `grabFrame()` — subclasses cannot bypass the lifecycle.

Failure path in `start()` (`src/camera/CameraProducer.cpp:18`) is correct: if any of `applyFormat`/`applyControls`/`startStream` throws, `closeDevice()` is invoked before re-throwing.

### 3.3 Canonical control names

Defined implicitly by the V4L2 mapping table (`src/v4l2/V4L2Producer.cpp:50`, `controlNameToCid()`) and validated/persisted by `SqliteConfigStore`. Currently:

```
exposure_auto              white_balance_auto         focus_auto
exposure_absolute          white_balance_temperature  focus_absolute
gain                       backlight_compensation     power_line_frequency
brightness                 contrast                   saturation
sharpness
```

Note this list is the **vocabulary** the rest of the app uses — it is intentionally generic (UVC-shaped) and should map naturally onto other backends (GenICam `ExposureAuto`, `Gain`, `BalanceWhiteAuto`, etc.).

---

## 4. The V4L2 backend

`include/posest/V4L2Producer.h`, `src/v4l2/V4L2Producer.cpp`. Linux-only, gated by `POSEST_HAS_V4L2`.

### 4.1 Device lifecycle

| Step | Implementation |
|------|----------------|
| `openDevice()` (`V4L2Producer.cpp:77`) | `open(O_RDWR)` on `config().device`. Verifies `V4L2_CAP_VIDEO_CAPTURE` and `V4L2_CAP_STREAMING` capabilities via `VIDIOC_QUERYCAP`. |
| `applyFormat()` (`V4L2Producer.cpp:104`) | `VIDIOC_S_FMT` with width/height/pixfmt. Stores driver-coerced values in `negotiated_*`. Then best-effort `VIDIOC_S_PARM` to set framerate (drivers often ignore this). |
| `applyControls()` (`V4L2Producer.cpp:131`) | Iterates `config().controls`, looks up CID via `controlNameToCid()`, issues `VIDIOC_S_CTRL`. Unknown names log and skip; failed sets log and continue. |
| `startStream()` (`V4L2Producer.cpp:151`) | Requests **4 MMAP buffers** (`kBufferCount`), `mmap`s them, queues them all, fires `VIDIOC_STREAMON`. |
| `grabFrame()` (`V4L2Producer.cpp:199`) | Loops on `poll(POLLIN, 500ms)` so `isRunning()` is checked periodically. On readiness, `VIDIOC_DQBUF` → timestamp conversion → format-specific decode → `VIDIOC_QBUF` re-enqueue. |
| `stopStream()` (`V4L2Producer.cpp:256`) | `VIDIOC_STREAMOFF`. |
| `closeDevice()` (`V4L2Producer.cpp:262`) | `munmap` each buffer, `close(fd)`. |

### 4.2 Timestamps

Kernel buffers are stamped with `CLOCK_MONOTONIC` when `V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC` is set. Because `std::chrono::steady_clock` on Linux is also `CLOCK_MONOTONIC`, the conversion is a direct construction:

```c++
auto dur = std::chrono::seconds(tv_sec) + std::chrono::microseconds(tv_usec);
return std::chrono::steady_clock::time_point(
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(dur));
```

This is guarded by a `static_assert(std::chrono::steady_clock::is_steady)` at file scope. Buffers without the monotonic flag fall back to the post-`grab` `steady_clock::now()` stamp applied by `ProducerBase`.

### 4.3 Pixel-format support

Nine formats are wired up via the `kPixelFormats` table (`V4L2Producer.cpp:110`):

| Format | Path |
|--------|------|
| `MJPEG` | `cv::imdecode(raw, IMREAD_COLOR)` — input wrapped over the MMAP buffer (zero-copy in), output is a fresh `cv::Mat`. |
| `YUYV`  | `cv::cvtColor(yuyv, out, COLOR_YUV2BGR_YUYV)` — input wrapped over the MMAP buffer (zero-copy in), output is a fresh `cv::Mat`. |
| `GREY`  | Mono 8-bit; wrapped and copied into an owning `cv::Mat`. |
| `NV12`  | `cv::cvtColor(nv12, out, COLOR_YUV2BGR_NV12)`. |
| `BGR3`  | Copied into an owning BGR `cv::Mat`. |
| `bayer_rggb8` / `bayer_grbg8` / `bayer_gbrg8` / `bayer_bggr8` | `cv::cvtColor` with the matching `COLOR_BayerXX2BGR` code — covers common mono machine-vision sensors. |

`pixelFormatFromString()` rejects anything not in the table with `std::runtime_error`. `test/test_v4l2_producer.cpp` asserts each format round-trips through the helper.

### 4.4 Buffer ownership

The decode path wraps the kernel buffer with a non-owning `cv::Mat` for the source, then emits the decoded frame as an owning `cv::Mat`. The MMAP buffer is re-queued **before** `grabFrame` returns, so the kernel reclaims it immediately. The owning output is what travels through the pipeline. This is correct: the consumer never holds a pointer into kernel memory.

---

## 5. Where the abstraction holds (and where it leaks)

### 5.1 Backend-agnosticism check

Greppable evidence the rest of the app does not know about V4L2:

| Surface | Knows about V4L2? |
|---------|-------------------|
| `RuntimeGraph` (`src/runtime/RuntimeGraph.cpp`) | No — only uses `IFrameProducer` and `ICameraBackendFactory`. |
| `Daemon` (`src/runtime/Daemon.cpp`) | No — only `IFrameProducer` and `CameraConfig`. |
| Pipelines (`src/pipelines/...`) | No — only `Frame`. |
| Calibration recorder | No — only `IFrameConsumer`. |
| Configuration store / validator | Treats `type` and `device` as opaque strings. |
| `ProductionCameraFactory::createCamera()` | **Yes** — this is the one allowed dispatch site. |

This part of the requirement is met: the V4L2 backend can be replaced or supplemented without touching anything outside `ProductionFactories.cpp` and the new backend's library.

### 5.2 Tests

- `test/test_v4l2_producer.cpp` — covers the static helpers (`controlNameToCid`, `timevalToSteadyClock`). Does not cover the live device path; that requires a real V4L2 node and is left to the integration environment.
- `test/test_producer_base.cpp` — covers the `ProducerBase`/consumer fan-out, sequencing, mailbox drop semantics, timestamp fallback.
- `test/test_mock_pipeline.cpp` — exercises full producer→consumer pipelines with `MockProducer`/`MockConsumer`.
- `test/test_runtime_graph.cpp` — exercises the factory + graph wiring with a `RecordingProducer` stub.

The `MockProducer`/`MockConsumer` pair (`include/posest/MockProducer.h`, `include/posest/MockConsumer.h`) are how the rest of the test suite exercises the contracts without needing a camera.

---

## 6. Status vs. the requirements

All four introspection/runtime-control/resilience requirements that were previously open have landed. The abstraction remains clean — everything is additive on `CameraProducer` with `virtual` defaults so backends can opt in per feature.

### 6.1 Requirement: hardware-triggered vs. free-running declaration  →  **IMPLEMENTED**

- `enum class TriggerMode { FreeRun, External, Software }` in `CameraConfig.h:23`, with `triggerModeToString` / `triggerModeFromString` helpers in `src/camera/CameraConfig.cpp:16`.
- `CameraConfig::trigger_mode` carries the requested mode (`CameraConfig.h:48`, default `FreeRun`).
- `virtual void setTriggerMode(TriggerMode)` on `CameraProducer` (`CameraProducer.h:83`). The base default accepts `FreeRun` as a no-op and throws `NotSupportedError` for `External` / `Software` (`CameraProducer.cpp:107`). The V4L2 backend keeps that default behavior since UVC webcams are free-running (`V4L2Producer.cpp:477`).
- Support is surfaced on the capability descriptor via `trigger_modes`, `current_trigger_mode`, and `supports_set_trigger_mode` (`CameraCapabilities.h:101`).

Future machine-vision backends (FLIR/Spinnaker, GenICam, Allied Vision) override `setTriggerMode()` and report the additional modes in their capabilities.

### 6.2 Requirement: re-initialization on unplug  →  **IMPLEMENTED**

`grabFrame()` now returns an `enum class GrabResult { Ok, EndOfStream, TransientError, Stopping }` (`CameraProducer.h:23`) so the base can distinguish a hard EOS from a transient I/O error.

- `ReconnectPolicy { interval_ms, max_attempts }` in `CameraConfig.h:36`. `max_attempts == 0` means retry forever (mirrors `TeensyConfig`); `interval_ms == 0` disables reconnect entirely.
- `CameraProducer::attemptReconnect()` (`CameraProducer.cpp:192`) runs the full ladder: `stopStream → closeDevice → interruptible sleep → openDevice → applyFormat → applyControls → startStream`. Exceptions at any step are caught, logged, and counted; the ladder retries until success, `max_attempts` exhaustion, or a `stop()` signal.
- The reconnect sleep is interruptible via a condition variable on `reconnect_cv_` so `stop()` tears the camera down promptly even mid-backoff.
- V4L2 `grabFrame()` reports `TransientError` on `POLLERR|POLLHUP|POLLNVAL` or `VIDIOC_DQBUF` failure (`V4L2Producer.cpp:370`, `388`), and `Stopping` when `isRunning()` flipped during the call (`V4L2Producer.cpp:364`).
- Connection state is exposed as `ConnectionState { Disconnected, Connecting, Streaming, Failed }` (`CameraCapabilities.h:13`) via `capabilities().live.state`, alongside `disconnect_count`, `reconnect_attempts`, and `successful_connects` counters (`CameraCapabilities.h:78`).

Tests: `test/test_camera_producer.cpp:135` covers the reconnect-on-transient path, interruptible-sleep on `stop()`, and `max_attempts` exhaustion transitioning the state to `Failed`.

### 6.3 Requirement: capability descriptor for the UI  →  **IMPLEMENTED**

`include/posest/CameraCapabilities.h` defines the full descriptor family:

- `PixelFormatOption` → `FrameSizeOption` → `FrameRateRange` (discrete + continuous) for the `(format, width, height, fps)` matrix.
- `ControlDescriptor` with `type` (Integer/Boolean/Menu), `min`/`max`/`step`/`default_value`/`current_value`, `read_only`, `auto_mode`, `auto_companion` (links e.g. `exposure_auto` ↔ `exposure_absolute`), and enumerated `menu_entries` for menu-typed controls.
- `DeviceHint { kind, description }` with `DeviceHintKind { DevicePath, SerialNumber, NetworkAddress, DiscoveryName }` so the UI knows whether to render a file picker (V4L2), a serial field (Spinnaker), etc. — this closes the §6.6 "wart" from earlier drafts.
- `LiveStats` with `state`, disconnect/reconnect counters, `measured_fps` (EWMA maintained by `ProducerBase`-adjacent hooks), `last_frame_time`, and `last_error`.
- Top-level `CameraCapabilities { camera_id, backend, device_hint, pixel_formats, controls, trigger_modes, supports_* flags, current_format, current_trigger_mode, live }`.

`virtual CameraCapabilities capabilities() const` on `CameraProducer` (`CameraProducer.h:73`). The base implementation fills `camera_id`, `backend`, cached `current_format` / `current_trigger_mode`, and `live` under `caps_mu_`; backends override to populate the static catalogs.

The V4L2 implementation (`V4L2Producer.cpp:524`) enumerates:

- pixel formats via `VIDIOC_ENUM_FMT`
- sizes via `VIDIOC_ENUM_FRAMESIZES` (both discrete and stepwise)
- frame rates via `VIDIOC_ENUM_FRAMEINTERVALS`
- control metadata via `VIDIOC_QUERYCTRL` + `VIDIOC_G_CTRL` + `VIDIOC_QUERYMENU`

### 6.4 Requirement: full live control (set exposure, gain, WB at runtime)  →  **IMPLEMENTED**

```c++
virtual void setControl(const std::string& name, std::int32_t value);
virtual std::optional<std::int32_t> getControl(const std::string& name) const;
```

on `CameraProducer.h:78`. Defaults throw `NotSupportedError` (`CameraProducer.cpp:97`). Mode flags (`exposure_auto`, `white_balance_auto`, `focus_auto`) use the same API, so auto↔manual switching is covered.

The V4L2 backend implements both with `VIDIOC_S_CTRL` / `VIDIOC_G_CTRL` (`V4L2Producer.cpp:447`, `465`) under `fd_mu_` (`V4L2Producer.h:62`). The mutex guards the fd against races with `openDevice` / `closeDevice` during reconnect, but is intentionally not held across `poll()` in `grabFrame()` so live-control calls never block the capture thread.

Unknown names: `setControl` throws `std::invalid_argument`; `getControl` returns `std::nullopt`.

### 6.5 Requirement: frame retrieval / init / destroy  →  **MET**

`grabFrame()` / `openDevice()` / `closeDevice()` are present and exercised through the lifecycle. ✅

### 6.6 Requirement: rest of app is backend-agnostic  →  **MET**

The factory branch (`if (config.type == "v4l2")`) is correct and the only such branch. ✅ The earlier "wart" — `CameraConfig::device` being an opaque string with no indication of how to populate it — is now resolved by `DeviceHint` in the capability descriptor (§6.3).

### 6.7 Other practical gaps — resolved

- **Negotiated format surfaced.** `CameraProducer::setCurrentFormat()` is called by `applyFormat()` in `V4L2Producer.cpp:254` and exposed through `currentFormat()` and `capabilities().current_format`.
- **Pixel-format catalog expanded.** See §4.3 — MJPEG, YUYV, GREY, NV12, BGR3, and four Bayer variants (RGGB/GRBG/GBRG/BGGR 8-bit).
- **Failed control sets are propagated.** `applyControls()` now returns `std::vector<ControlSetError>` (`CameraProducer.h:100`). `CameraProducer::start()` and `attemptReconnect()` capture the list, stash the first failure in `live_stats_.last_error`, and expose the full vector via `lastApplyErrors()` (`CameraProducer.h:91`). The V4L2 backend collects both unknown-name and ioctl-failure cases (`V4L2Producer.cpp:273`).
- **Device enumeration implemented.** `posest::v4l2::enumerateDevices()` (`include/posest/V4L2DeviceEnumerator.h`) walks `/sys/class/video4linux/video*`, verifies `V4L2_CAP_VIDEO_CAPTURE` + `V4L2_CAP_STREAMING` via `VIDIOC_QUERYCAP`, and resolves each result to the most stable path under `/dev/v4l/by-id/` when available. Returns a `std::vector<CameraCapabilities>` ready for the UI camera picker.
- **`start()` re-entrancy tested.** `test/test_camera_producer.cpp` exercises `start → stop → start → stop` with a mock backend and asserts each hook was invoked exactly twice.

### 6.8 Remaining tradeoff (not a gap)

- **Per-frame decoding cost is on the producer thread.** MJPEG decode in particular can be 2–10 ms at 1080p. This is a deliberate latency tradeoff (decode-once-then-fan-out beats decode-per-consumer), but means the producer thread is not "pure capture." If the future load includes 2× 4K MJPEG cameras at 60 fps on the same machine, decoding off-thread will become necessary.

---

## 7. Summary: feature-completeness scorecard

| Requirement | Status | Notes |
|---|---|---|
| Generic producer interface | ✅ Complete | `IFrameProducer`, clean and minimal. |
| Backend-agnostic rest of app | ✅ Complete | One factory branch; everything else uses `IFrameProducer` + `CameraConfig`. |
| Frame retrieval / init / teardown | ✅ Complete | `CameraProducer` lifecycle, V4L2 implementation. |
| Auto/manual exposure, gain, WB, etc. (config-time) | ✅ Complete | UVC control vocabulary, `applyControls()`. |
| Live (runtime) control updates | ✅ Complete | `setControl()` / `getControl()` on `CameraProducer`; V4L2 uses `VIDIOC_S_CTRL` / `VIDIOC_G_CTRL` under `fd_mu_`. |
| Reinitialization on unplug | ✅ Complete | `GrabResult::TransientError` + `ReconnectPolicy` + `attemptReconnect()` ladder with interruptible backoff. |
| Hardware-trigger declaration / control | ✅ Complete | `TriggerMode { FreeRun, External, Software }`, `setTriggerMode()`, capability-descriptor surfaced. |
| Capability descriptor for UI | ✅ Complete | `CameraCapabilities` enumerates formats, controls, trigger modes, live stats, and `DeviceHint`. |
| Stable timestamp domain (`steady_clock`) | ✅ Complete | Enforced by `ProducerBase`, V4L2 honors monotonic flag. |
| Drop-oldest, never-stall mailbox | ✅ Complete | `LatestFrameSlot`. |
| Pixel-format coverage | ✅ Complete | MJPEG, YUYV, GREY, NV12, BGR3, and four Bayer variants (9 total). |
| Device enumeration | ✅ Complete | `posest::v4l2::enumerateDevices()` returns `CameraCapabilities[]` for every usable `/dev/video*`. |
| Negotiated-format readback | ✅ Complete | `currentFormat()` + `capabilities().current_format`. |
| Failed-control propagation | ✅ Complete | `applyControls()` returns `std::vector<ControlSetError>`; surfaced via `lastApplyErrors()` + `live.last_error`. |
| Connection-state telemetry | ✅ Complete | `ConnectionState` + disconnect/reconnect counters in `LiveStats`. |

**Overall:** feature-complete against the stated requirements. The architecture did not need restructuring — every addition slotted in as a `virtual` on `CameraProducer` with a safe default, keeping non-V4L2 backends free to opt in incrementally. The camera-config UI can now be built directly against `CameraCapabilities` without any per-backend branching.

---

## 8. File reference

| File | Role |
|------|------|
| `include/posest/IFrameProducer.h` | Generic frame producer interface. |
| `include/posest/IFrameConsumer.h` | Generic frame consumer interface. |
| `include/posest/Frame.h` | Frame value type (image + steady_clock timestamp + sequence + camera id). |
| `include/posest/ProducerBase.h` / `src/core/ProducerBase.cpp` | Capture-thread + fan-out base. |
| `include/posest/ConsumerBase.h` / `src/core/ConsumerBase.cpp` | Mailbox-driven consumer base. |
| `include/posest/LatestFrameSlot.h` / `src/core/LatestFrameSlot.cpp` | Drop-oldest single-slot mailbox. |
| `include/posest/CameraConfig.h` / `src/camera/CameraConfig.cpp` | `CameraConfig`, `TriggerMode`, `ReconnectPolicy`, and `triggerMode*` helpers. |
| `include/posest/CameraCapabilities.h` | Capability descriptor types (`CameraCapabilities`, `ControlDescriptor`, `PixelFormatOption`, `DeviceHint`, `LiveStats`, `ConnectionState`). |
| `include/posest/CameraProducer.h` / `src/camera/CameraProducer.cpp` | Camera lifecycle abstraction, reconnect ladder, live-control API, `capabilities()`. |
| `include/posest/V4L2Producer.h` / `src/v4l2/V4L2Producer.cpp` | V4L2 backend (Linux) — grab, live controls, capability enumeration. |
| `include/posest/V4L2DeviceEnumerator.h` / `src/v4l2/V4L2DeviceEnumerator.cpp` | Device discovery for the UI picker. |
| `include/posest/MockCameraProducer.h` / `src/mock/MockCameraProducer.cpp` | Test double used by `test_camera_producer.cpp` to exercise lifecycle + reconnect paths without a real device. |
| `src/runtime/ProductionFactories.cpp` | The single backend-dispatch site (`type` → concrete class). |
| `src/runtime/RuntimeGraph.cpp` | Wires cameras → pipelines via the generic interfaces only. |
| `test/test_producer_base.cpp` | ProducerBase / fan-out / sequencing / drop tests. |
| `test/test_camera_producer.cpp` | Lifecycle, `start()` re-entrancy, reconnect ladder, `setControl` / `setTriggerMode` defaults. |
| `test/test_v4l2_producer.cpp` | V4L2 static helpers (CID mapping, timestamp conversion, pixel-format table). |
