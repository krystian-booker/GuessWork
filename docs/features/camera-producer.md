# Camera Producer Subsystem

**Status:** Partially feature-complete. Core abstraction and V4L2 backend are solid; several explicit requirements (capability descriptors, live control updates, hotplug recovery, hardware-trigger abstraction) are missing.

This document covers the generic frame producer interface, the camera-specific lifecycle abstraction layered on top of it, and the V4L2 implementation that ships today. It is structured as: requirements → architecture → what is implemented → gap analysis against the requirements → recommended additions to reach feature-complete.

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

| Format | Path |
|--------|------|
| `MJPEG` | `cv::imdecode(raw, IMREAD_COLOR)` — input wrapped over the MMAP buffer (zero-copy in), output is a fresh `cv::Mat`. |
| `YUYV`  | `cv::cvtColor(yuyv, out, COLOR_YUV2BGR_YUYV)` — input wrapped over the MMAP buffer (zero-copy in), output is a fresh `cv::Mat`. |

`pixelFormatFromString()` (`V4L2Producer.cpp:33`) currently rejects anything else with `std::runtime_error`. Other USB cameras commonly emit GREY, NV12, BGR3, or raw Bayer — none of those are wired up yet.

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

## 6. Gap analysis vs. the requirements

This is the part to read carefully — the abstraction is mostly there, but several explicit requirements are not yet met.

### 6.1 Requirement: hardware-triggered vs. free-running declaration  →  **MISSING**

The producer/camera interface has **no concept of trigger mode**. The Teensy domain has a `CameraTriggerConfig` (used to drive GPIO pulses out of the Teensy and to time-align frames during calibration recording), but:

- `CameraConfig` does not carry a `trigger_mode` field.
- `CameraProducer` has no `setTriggerMode()` / `isHardwareTriggered()` method.
- The V4L2 backend never sets `V4L2_CID_PRIVACY` / vendor trigger controls and never asserts that a UVC camera is in a particular trigger state.

For UVC webcams this is mostly fine (they are not hardware-triggered). For machine-vision cameras (FLIR/Spinnaker, GenICam, Allied Vision), trigger mode is a first-class control and the abstraction needs to express it.

**Suggested shape:** add an enum to `CameraConfig` (`TriggerMode { Free, External, Software }`) and a virtual `setTriggerMode()` on `CameraProducer` that defaults to throwing `not_supported`. Surface support via the capability descriptor (§6.3).

### 6.2 Requirement: re-initialization on unplug  →  **MISSING**

Today, on disconnect:

- `poll()` typically returns immediately with `POLLERR`/`POLLHUP`; the `pollfd.events = POLLIN` mask means we ignore these and the next `VIDIOC_DQBUF` fails with `ENODEV`.
- `grabFrame()` returns `false` (`V4L2Producer.cpp:222`).
- `ProducerBase::runLoop()` interprets `false` as end-of-stream and exits the loop.
- The capture thread terminates silently. Consumers continue to wait on empty mailboxes. There is no surfaced error, no retry, no reopen.

The Teensy service has explicit reconnect plumbing (`reconnect_interval_ms`, `reconnect_attempts`, `disconnects` in stats) — cameras have none of that.

**Suggested shape:**
- Distinguish "end-of-stream" from "transient I/O error" in the `captureOne()` return contract (e.g. an enum or exception).
- Add a reconnect policy to `CameraConfig` (`reconnect_interval_ms`, `max_reconnect_attempts`, etc.).
- In `CameraProducer`, on transient failure, run `stopStream → closeDevice → sleep → openDevice → applyFormat → applyControls → startStream` with backoff.
- Expose `connection_state` in the capability/health descriptor and surface it in telemetry alongside the existing Teensy reconnect metrics.

### 6.3 Requirement: capability descriptor for the UI  →  **MISSING**

The user explicitly asked for "some object that can be returned for the frontend that states what the camera backend is capable of so the UI knows what to display." Nothing of the sort exists today.

What the UI would need to render a configuration page without per-backend `if` trees:

- list of supported pixel formats and their `(width, height, fps)` matrices
- list of supported controls (name, type, range/min/max/step/default, current value, whether currently in auto mode)
- list of supported trigger modes
- whether the device is currently connected, current negotiated format, last-frame timestamp / fps

None of this is exposed from `IFrameProducer` or `CameraProducer`. The V4L2 backend has the data internally (`negotiated_pixfmt_`, `negotiated_width_`, `negotiated_height_`) but doesn't surface it.

**Suggested shape:** introduce `struct CameraCapabilities { ... }` and a virtual `CameraCapabilities capabilities() const` on `CameraProducer`. Each backend fills in what it knows; the UI renders strictly off this struct. For V4L2, populate by enumerating with `VIDIOC_ENUM_FMT`, `VIDIOC_ENUM_FRAMESIZES`, `VIDIOC_ENUM_FRAMEINTERVALS`, and `VIDIOC_QUERYCTRL` / `VIDIOC_QUERY_EXT_CTRL`.

### 6.4 Requirement: full live control (set exposure, gain, WB at runtime)  →  **PARTIALLY MISSING**

Today, controls are applied **once** during `start()` (`CameraProducer::start` → `applyControls()`). There is no public API for runtime adjustment. If the UI wants to slide an exposure control, the only path is: edit `RuntimeConfig`, persist, restart the camera (and likely the whole runtime graph). That is unacceptable for an interactive tuning UI.

**Suggested shape:** add to `CameraProducer`:

```c++
virtual void setControl(const std::string& name, std::int32_t value);
virtual std::optional<std::int32_t> getControl(const std::string& name) const;
```

Both default to `not_supported`. The V4L2 backend implements them with `VIDIOC_S_CTRL` / `VIDIOC_G_CTRL` (taking care to use atomic locking around the fd if needed — `ioctl` is generally safe to call from another thread but the buffer ring is not).

Mode flags (`exposure_auto`, `white_balance_auto`, `focus_auto`) are already in the control vocabulary, so this single API covers auto vs. manual switching too.

### 6.5 Requirement: frame retrieval / init / destroy  →  **MET**

`grabFrame()` / `openDevice()` / `closeDevice()` are present and exercised through the lifecycle. ✅

### 6.6 Requirement: rest of app is backend-agnostic  →  **MET (with one wart)**

The factory branch (`if (config.type == "v4l2")`) is correct and the only such branch. ✅

The wart: `CameraConfig::device` is just a string and its meaning is backend-defined. That is fine in principle but the UI has no way to know whether it should be a file picker (V4L2: `/dev/...`), a serial number (Spinnaker), a CIDR (GigE), or a discovery name. The capability descriptor (§6.3) should also describe how `device` is meant to be populated.

### 6.7 Other practical gaps

- **Negotiated format not surfaced.** V4L2 may coerce the requested width/height/pixel format. The negotiated values are kept inside the producer but never reported back. Consumers and UI both could benefit from seeing them.
- **Pixel-format catalog is small.** Only `mjpeg` and `yuyv` are implemented. GREY (mono machine-vision sensors), NV12, BGR3, raw Bayer (RG10/GR12 etc.) are not supported. If the planned hardware uses any of those, this is a blocker.
- **Per-frame decoding cost is on the producer thread.** MJPEG decode in particular can be 2–10 ms at 1080p. This is a deliberate latency tradeoff (decode-once-then-fan-out beats decode-per-consumer), but means the producer thread is not "pure capture." If the future load includes 2× 4K MJPEG cameras at 60 fps on the same machine, decoding off-thread will become necessary.
- **Failed control sets are only printed to stderr.** `applyControls()` continues silently on failure (`V4L2Producer.cpp:144`). With a capability descriptor and live-control API in place, failures should be propagated to the caller / surfaced via telemetry.
- **No device enumeration.** The UI cannot ask "what cameras are plugged in?" — the user has to know `/dev/v4l/by-id/...` paths up front. This is part of the same gap as the capability descriptor.
- **No `start()` re-entrancy.** Calling `start()` twice on a `CameraProducer` after a deliberate `stop()` works (each call goes through `openDevice` again), but there are no tests for it; should be added once reconnect logic lands.

---

## 7. Summary: feature-completeness scorecard

| Requirement | Status | Notes |
|---|---|---|
| Generic producer interface | ✅ Complete | `IFrameProducer`, clean and minimal. |
| Backend-agnostic rest of app | ✅ Complete | One factory branch; everything else uses `IFrameProducer` + `CameraConfig`. |
| Frame retrieval / init / teardown | ✅ Complete | `CameraProducer` lifecycle, V4L2 implementation. |
| Auto/manual exposure, gain, WB, etc. (config-time) | ✅ Complete | UVC control vocabulary, `applyControls()`. |
| **Live (runtime) control updates** | ❌ Missing | No `setControl()`/`getControl()` API. |
| **Reinitialization on unplug** | ❌ Missing | Capture thread silently exits on disconnect. |
| **Hardware-trigger declaration / control** | ❌ Missing | No `TriggerMode` field, no API. |
| **Capability descriptor for UI** | ❌ Missing | UI cannot introspect what a backend supports. |
| Stable timestamp domain (`steady_clock`) | ✅ Complete | Enforced by `ProducerBase`, V4L2 honors monotonic flag. |
| Drop-oldest, never-stall mailbox | ✅ Complete | `LatestFrameSlot`. |
| Pixel-format coverage | ⚠️ Limited | Only MJPEG and YUYV. |
| Device enumeration | ❌ Missing | No way to discover devices for the UI. |
| Negotiated-format readback | ❌ Missing | Held internally, not exposed. |

**Overall:** the core architecture and the V4L2 implementation are solid and the abstraction does not leak. The missing pieces are all on the **introspection / runtime-control / resilience** axis — exactly the surface a configuration UI needs. Before writing the camera-config UI, the four ❌ items in §6.1–§6.4 (capability descriptor, live `setControl`/`getControl`, hardware-trigger abstraction, reconnect-on-unplug) should land. None of them require restructuring the existing code; they are all additive on `CameraProducer`.

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
| `include/posest/CameraConfig.h` | `CameraConfig` / `CameraFormatConfig` / `CameraControlEntry`. |
| `include/posest/CameraProducer.h` / `src/camera/CameraProducer.cpp` | Camera lifecycle abstraction layered over `ProducerBase`. |
| `include/posest/V4L2Producer.h` / `src/v4l2/V4L2Producer.cpp` | V4L2 backend (Linux). |
| `src/runtime/ProductionFactories.cpp` | The single backend-dispatch site (`type` → concrete class). |
| `src/runtime/RuntimeGraph.cpp` | Wires cameras → pipelines via the generic interfaces only. |
| `test/test_producer_base.cpp` | ProducerBase / fan-out / sequencing / drop tests. |
| `test/test_v4l2_producer.cpp` | V4L2 static helpers (CID mapping, timestamp conversion). |
