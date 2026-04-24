# Producer/Consumer Architecture

**Status:** Core plumbing is feature-complete for static graphs. The multi-threading model, drop-oldest mailbox, and 1-to-many fan-out all match the stated latency and topology requirements. The primary gap is **dynamic subscription** — the subscriber list is frozen at `start()`, which blocks the "attach a live video feed consumer on demand" use case without a restart.

This document covers the subsystem that moves `Frame` objects from any source (camera, mock, future file replay) to any sink (vision pipelines, calibration recorder, future video-preview feed). Camera-specific lifecycle and backend details live in [`camera-producer.md`](camera-producer.md); this document stays one layer up — the generic plumbing that all producers and consumers share.

---

## 1. Requirements (target behavior)

The requirements this document reviews the implementation against:

1. **Every producer and every consumer runs on its own thread.** No single thread may service more than one of them.
2. **Minimal capture-to-process latency.** When a consumer is ready to process, it must receive the *most recent* frame, not the oldest queued one.
3. **Producer must never be blocked by a slow consumer.** The producer thread is purely capture + hand-off; it owns the timing budget.
4. **Many producers.** Target hardware ships with roughly **6 cameras** running simultaneously, each with independent framerate and latency.
5. **1-to-many fan-out.** A single producer can have multiple consumers attached; the typical case is 1-to-1 (one camera → one vision pipeline), but auxiliary consumers (e.g. a web UI video feed, calibration recorder) must be able to subscribe to the same producer without impacting the primary consumer.

---

## 2. Architecture overview

### 2.1 The four moving parts

```
     ┌────────────────── capture thread ──────────────────┐
     │                                                    │
     │   captureOne()  →  stamp + sequence  →  fan-out    │
     │                                           │        │
     └───────────────────────────────────────────┼────────┘
                                                 │ synchronous,
                                                 │ non-blocking deliver()
                                                 ▼
             ┌────────────────┐       ┌────────────────┐       ┌────────────────┐
             │ LatestFrameSlot│       │ LatestFrameSlot│       │ LatestFrameSlot│
             │   (drop-oldest)│       │   (drop-oldest)│       │   (drop-oldest)│
             └───────┬────────┘       └───────┬────────┘       └───────┬────────┘
                     │ take() (blocks)        │                        │
                     ▼                        ▼                        ▼
             ┌────────────────┐       ┌────────────────┐       ┌────────────────┐
             │ worker thread  │       │ worker thread  │       │ worker thread  │
             │  process()     │       │  process()     │       │  process()     │
             └────────────────┘       └────────────────┘       └────────────────┘
                Consumer A              Consumer B              Consumer C
```

| Component | File | Role |
|-----------|------|------|
| `IFrameProducer` | `include/posest/IFrameProducer.h` | Pure interface: `id()`, `addConsumer()`, `start()`, `stop()`. |
| `IFrameConsumer` | `include/posest/IFrameConsumer.h` | Pure interface: `id()`, `start()`, `stop()`, `deliver(FramePtr)`. |
| `Frame` | `include/posest/Frame.h` | Value type: `steady_clock` capture_time + monotonic sequence + camera_id + `cv::Mat image`. Distributed as `std::shared_ptr<const Frame>`. |
| `ProducerBase` | `include/posest/ProducerBase.h`, `src/core/ProducerBase.cpp` | Owns capture thread, subscriber list, sequence counter, timestamp fallback. Subclasses implement `captureOne()`. |
| `ConsumerBase` | `include/posest/ConsumerBase.h`, `src/core/ConsumerBase.cpp` | Owns consumer worker thread, holds a `LatestFrameSlot`. Subclasses implement `process()`. |
| `LatestFrameSlot` | `include/posest/LatestFrameSlot.h`, `src/core/LatestFrameSlot.cpp` | Single-slot drop-oldest mailbox. Non-blocking `put()`, blocking `take()`. |
| `RuntimeGraph` | `include/posest/runtime/RuntimeGraph.h`, `src/runtime/RuntimeGraph.cpp` | Constructs producers + consumers from config, wires bindings, orders start/stop. |

### 2.2 Threading model

Per active graph, threads in play:

- **N producer threads** — one per enabled camera (`ProducerBase::worker_`). Started by `producer.start()` after every consumer has been attached and started.
- **M consumer threads** — one per consumer (`ConsumerBase::worker_`, `VisionPipelineBase::worker_`, `CalibrationRecorder::worker_`). Each loops on its own `LatestFrameSlot::take()` → `process()`.
- The calling thread (usually the main/daemon thread) is used only for `start()`/`stop()` orchestration and never participates in frame flow.

Producer and consumer threads are fully decoupled: a 60 FPS producer with a 5 Hz consumer still produces at 60 FPS. The producer has no handle on the consumer's worker; it only calls the consumer's non-blocking `deliver()`.

### 2.3 The latency contract

This is the load-bearing rule: **the producer thread is never allowed to stall.** Three mechanisms enforce it.

**(1) `IFrameConsumer::deliver()` MUST be non-blocking.**

`ProducerBase::runLoop()` (`src/core/ProducerBase.cpp:47`) calls `deliver()` **synchronously in a loop** over the subscribers:

```cpp
for (auto& c : subscribers) {
    c->deliver(pub);
}
```

If any consumer's `deliver()` blocks, every other consumer on the same producer is also blocked, and the producer itself stops capturing. The interface declares this contract; `ConsumerBase::deliver` and `VisionPipelineBase::deliver` both satisfy it by forwarding straight into their internal `LatestFrameSlot::put()`:

```cpp
void ConsumerBase::deliver(FramePtr frame) {
    if (!running_.load(std::memory_order_acquire)) return;
    slot_.put(std::move(frame));
}
```

**(2) `LatestFrameSlot::put()` never blocks and drops the old frame.**

```cpp
void LatestFrameSlot::put(FramePtr frame) {
    {
        std::lock_guard<std::mutex> g(mu_);
        if (shut_) return;
        if (pending_) ++dropped_;
        pending_ = std::move(frame);
    }
    cv_.notify_one();
}
```

The mutex is held only for the pointer swap — microseconds, never waits on the consumer. When a consumer is slow, its mailbox's `dropped_` counter bumps; the producer thread is unaware and unaffected.

**(3) `take()` always returns the newest frame.**

The consumer's worker sees exactly the most recent capture when it's ready to run `process()`. It never processes a stale backlog. This is the "minimal latency" guarantee: the consumer's observed latency is bounded by one capture period plus its own processing time — never by a queue depth.

### 2.4 Consequence: sequence gaps are the norm, not a bug

Consumers see a strictly increasing but non-contiguous subsequence of frame sequence numbers. A 120 FPS producer feeding a 20 FPS consumer will produce sequences 0, 1, 2, …, 119; the consumer sees roughly 0, 6, 12, … (or whatever scheduling produces). `LatestFrameSlot::droppedCount()` exposes how many frames were skipped per-consumer. This is covered by `test/test_mock_pipeline.cpp:48` (`SlowConsumerDropsButProducerIsNotBlocked`) which asserts both that producer FPS stays at target *and* that the consumer observes gaps.

### 2.5 Timestamp discipline

All `Frame::capture_time` values live in the `std::chrono::steady_clock` domain, regardless of producer backend. This is required for cross-camera math (multi-camera fusion, VIO time alignment). Contract for `ProducerBase` subclasses:

- **Preferred:** the subclass supplies `out_capture_time` inside `captureOne()`, converted from whatever native timestamp the backend exposes (V4L2 `CLOCK_MONOTONIC` buffer stamp, GenICam/PTP hardware stamp, vendor SDK metadata). For cameras with their own epoch, do a one-shot startup calibration (capture `steady_clock::now()` and the camera tick together, store the offset).
- **Fallback:** if `out_capture_time` is left `std::nullopt`, `ProducerBase::runLoop` stamps `steady_clock::now()` immediately after `captureOne()` returns. This is subject to scheduler jitter and should only be used when the backend genuinely has no better stamp.

`MockProducer` stamps the *scheduled deadline* (`src/mock/MockProducer.cpp:50`), mimicking what a real camera's shutter-time stamp looks like. This is what the `SubclassSuppliedTimestampPassesThrough` and `FallbackStampsWithSteadyClockNowWhenSubclassOmits` tests in `test/test_producer_base.cpp` exercise.

---

## 3. Multi-producer topology

The system is designed for **~6 concurrent producers**. There is no single shared capture thread, no global frame queue, and no central dispatcher — each producer is fully self-contained.

`RuntimeGraph::build()` (`src/runtime/RuntimeGraph.cpp:22`) walks `config_.cameras`, asks the factory for an `IFrameProducer` for each enabled entry, and stores them in a map keyed by `camera_id`. `RuntimeGraph::start()` then calls `start()` on each producer in order. Each producer spawns its own capture thread inside that call:

```cpp
void ProducerBase::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) return;
    worker_ = std::thread(&ProducerBase::runLoop, this);
}
```

There is no coupling between producers. Six cameras at mixed framerates operate as six independent capture threads, six independent sequence counters, six independent mailboxes per consumer. The OS scheduler handles the CPU contention; on any modern multi-core machine this scales linearly as long as decode cost stays in budget.

**What the implementation does NOT need:** there is no thread pool, no work-stealing queue, no shared producer registry, no cross-producer synchronization. The design deliberately pushes all coordination to the OS and keeps the abstraction flat.

**What is shared across producers:** only the `IMeasurementSink` downstream (Teensy fusion output, etc.) — and that sink is responsible for its own thread-safety. `FusionService` and `MeasurementBus` are written with multiple producers in mind.

---

## 4. Fan-out: 1 producer to many consumers

The typical wiring today is 1-to-1 (one camera → one `AprilTagPipeline`), driven by `RuntimeConfig::bindings`. `RuntimeGraph::build()` resolves each binding and calls `camera->addConsumer(pipeline)` (`src/runtime/RuntimeGraph.cpp:74`).

The fan-out mechanics are the same whether there's 1 or 10 consumers: the producer's subscriber list holds `shared_ptr<IFrameConsumer>`, and the capture loop iterates over it per frame. Each consumer gets the *same* `shared_ptr<const Frame>` — the image data is shared, not copied. This is cheap: adding a second consumer to an existing camera costs one mailbox (`LatestFrameSlot`, ~40 bytes), one worker thread, and one `shared_ptr` refcount bump per frame delivered.

### 4.1 Example: parallel fan-out verified

`test/test_mock_pipeline.cpp:80` (`FastAndSlowConsumersAreIndependent`) demonstrates this explicitly:

- Producer at 120 FPS.
- Consumer A: idle (fast).
- Consumer B: 30 ms/frame (slow, ~33 FPS ceiling).
- Assertion: `fast.frames_received > slow.frames_received * 2` — A processes nearly every frame while B drops ~75% of them. Neither affects the other.

### 4.2 The "attach a video preview" use case

The explicit motivating case is a **display consumer** that streams frames to the web UI for live preview. The architecture supports this semantically: the preview is just another `IFrameConsumer` subscribed to the camera. The display consumer would:

- Accept a frame in `deliver()` → put it in its own `LatestFrameSlot`.
- A worker thread takes the latest frame, JPEG-encodes it (or hands it to an MJPEG HTTP endpoint), and streams it.
- If encoding is slow, its mailbox drops stale frames; the primary vision pipeline attached to the same camera is untouched.

**Today this is not wired up.** There is no `DisplayConsumer` / `VideoPreviewConsumer` implementation and no binding in `RuntimeConfig` for non-pipeline consumers. See §6.

---

## 5. Lifecycle and orchestration

`RuntimeGraph` owns startup/shutdown ordering. The canonical sequence in production (`src/runtime/RuntimeGraph.cpp:80–107`):

**Startup (`start()`):**

1. `build()` has already constructed producers, constructed consumers (pipelines), and called `camera->addConsumer(pipeline)` for every binding.
2. `pipeline->start()` for each pipeline — workers begin blocking on their mailboxes.
3. `camera->start()` for each camera — capture threads begin fanning out.

**Shutdown (`stop()`):**

1. `camera->stop()` for each camera in reverse order — each joins its capture thread first. No new frames flow after this.
2. `pipeline->stop()` for each pipeline in reverse order — `slot_.shutdown()` unblocks the waiting `take()` with `nullptr`, the worker exits.

This ordering is what the test suite uses (`test/test_mock_pipeline.cpp`). Base destructors (`ProducerBase::~ProducerBase`, `ConsumerBase::~ConsumerBase`) both call `stop()` as a safety net, but explicit ordering is required for deterministic teardown — destruction order of members in `RuntimeGraph` is not what you want to rely on.

### 5.1 Wiring contract: addConsumer before start

`ProducerBase::runLoop()` snapshots the consumer list once at the top of the loop:

```cpp
std::vector<std::shared_ptr<IFrameConsumer>> subscribers;
{
    std::lock_guard<std::mutex> g(consumers_mu_);
    subscribers = consumers_;
}
while (running_.load(std::memory_order_acquire)) { ... }
```

After `start()` returns, **hot-adding or removing consumers is not supported.** `addConsumer()` still takes the lock, so calling it during capture won't corrupt memory — it just won't have any effect on the live snapshot. This is the design choice referenced in `CLAUDE.md` and documented in `IFrameProducer.h:13`: "Wiring (addConsumer) must happen before start()."

This is the primary gap for the "attach a live preview on demand" use case (see §6.1).

---

## 6. Gap analysis vs. the stated requirements

| Requirement | Status | Notes |
|-------------|--------|-------|
| One thread per producer | ✅ Complete | `ProducerBase::worker_` per producer. |
| One thread per consumer | ✅ Complete | `ConsumerBase::worker_` (and `VisionPipelineBase::worker_`, `CalibrationRecorder::worker_`). |
| Minimal latency — only keep newest frame | ✅ Complete | `LatestFrameSlot` drop-oldest, per-consumer. |
| Producer never blocked by slow consumer | ✅ Complete | Enforced by `put()` non-blocking + `deliver()` contract. Verified in `test_mock_pipeline.cpp`. |
| Scales to ~6 producers | ✅ Complete | Independent, no shared state between producers. No structural limit. |
| 1-to-many fan-out | ✅ Complete for static graphs | Subscriber list is fixed at `start()`. |
| **Dynamic subscribe/unsubscribe (e.g. live preview attach)** | ❌ Missing | Snapshot-at-start is explicit non-goal today. |
| **Video-preview consumer implementation** | ❌ Missing | No `DisplayConsumer` class, no config binding for non-pipeline consumers. |
| Cross-camera timestamp domain | ✅ Complete | `steady_clock` enforced by `ProducerBase`. |
| Shutdown ordering / clean teardown | ✅ Complete | `RuntimeGraph` + base destructors + tests. |

### 6.1 Dynamic subscription for live preview — **MISSING**

The limiting factor for a user-facing video feed is §5.1. A preview consumer that attaches when the frontend opens a page and detaches when it closes cannot be done today without stopping and restarting the camera. There are two reasonable shapes for the fix:

**Option A — mutex-protected live list.** Make `ProducerBase::runLoop` re-read the subscriber list each iteration under the mutex. Cost: one lock per frame per producer (~500 ns). Benefit: simplest API. `addConsumer()` and a new `removeConsumer()` just work, no restart needed.

**Option B — RCU-style snapshot swap.** Keep the snapshot in the hot loop, but publish an atomic `shared_ptr<vector<shared_ptr<IFrameConsumer>>>` that `runLoop` loads once per frame with `std::memory_order_acquire`. `addConsumer` / `removeConsumer` allocate a new vector and store it. No lock on the hot path.

Either is a small change — the harder work is the API question (should `removeConsumer` be by-id or by-pointer? what is the contract for frames already in flight to the removed consumer?).

Once this lands, the "attach a preview" feature becomes:

1. UI opens video feed page.
2. Web service creates a `DisplayConsumer` (new class), starts it, and calls `camera->addConsumer(display)`.
3. UI closes page. Web service calls `camera->removeConsumer(display)`, stops it.
4. Camera keeps running; the primary pipeline is untouched.

### 6.2 Video-preview consumer — **MISSING**

No `DisplayConsumer` / `VideoPreviewConsumer` / MJPEG-over-HTTP streamer exists in the codebase. `posest::runtime::WebService` exists but serves config and telemetry, not frames. Building this is a small implementation once §6.1 lands.

Suggested shape:

```cpp
class PreviewConsumer final : public ConsumerBase {
public:
    explicit PreviewConsumer(std::string id, int jpeg_quality = 70);
    // Called by HTTP handlers; blocks until next frame or timeout.
    std::optional<std::vector<unsigned char>> nextEncodedFrame(
        std::chrono::milliseconds timeout);
protected:
    void process(const Frame& f) override;  // encode to JPEG, stash in mailbox
};
```

This keeps encoding on the consumer's own thread, off the producer thread — consistent with the latency contract.

### 6.3 Non-pipeline bindings in `RuntimeConfig` — **MISSING**

`RuntimeConfig::bindings` only supports camera → vision pipeline. Auxiliary consumers like the calibration recorder are wired up imperatively in `Daemon.cpp:606` (they bypass `RuntimeGraph`). A preview consumer could follow the same pattern, but if we want preview consumers to be declarative (e.g. always-on low-res thumbnail stream per camera), the binding model needs a second consumer category.

Low priority: the imperative attach path from the web service is probably the right primary mode for preview anyway.

### 6.4 Plumbing-level duplication: `ConsumerBase` vs. `VisionPipelineBase`

`VisionPipelineBase` (`src/pipelines/VisionPipelineBase.cpp`) reimplements the exact same `thread + LatestFrameSlot + deliver/put/take/process` pattern that `ConsumerBase` already provides. The only real difference is `VisionPipelineBase` also carries an `IMeasurementSink&` and exposes `type()`. This should probably inherit from `ConsumerBase` and drop the duplicated plumbing.

Not a correctness issue, but a cleanup that would pay off every time we add a new kind of consumer. Low priority.

### 6.5 No producer-side telemetry for "I died"

When a producer's `captureOne()` returns `false` (end-of-stream, disconnect), the capture thread exits silently. Consumers continue to wait on empty mailboxes forever. There is no `IFrameProducer::isAlive()` / `connectionState()` method and no callback to the graph. This is discussed in more depth in [`camera-producer.md`](camera-producer.md) §6.2 — it's a camera-lifecycle concern more than a plumbing one, but the plumbing could help by exposing a "producer stopped on its own" signal.

### 6.6 Decode-on-producer-thread tradeoff (informational, not a gap)

MJPEG decode happens on the producer thread (`V4L2Producer::grabFrame`), so the Mat that goes into the fan-out is already decoded. This is a deliberate choice: decode once, fan out to N consumers, instead of decoding once per consumer. At 6 cameras × 1080p MJPEG × 60 FPS this is well within budget on a modern CPU. At 6 × 4K × 60 FPS it may not be — at that point, per-consumer decode or off-thread decode would need to be introduced. Not a current requirement.

---

## 7. Summary

The core plumbing (interfaces, base classes, drop-oldest mailbox, fan-out, startup/shutdown, timestamp discipline) implements the stated requirements cleanly. Multi-producer scales trivially because nothing is shared between producers; 1-to-many fan-out is a first-class feature and its isolation properties are covered by tests.

The meaningful gap for the planned product is **dynamic subscription** — the subscriber list is frozen at `start()` by design. This blocks the "attach a live preview on demand" use case without a camera restart. Fixing it is a small, localized change to `ProducerBase::runLoop` plus a `removeConsumer()` method on `IFrameProducer`. Once that lands, building the video-preview consumer is a straightforward consumer implementation that encodes frames off the producer thread and streams them over HTTP.

Everything else flagged (plumbing duplication, producer-death telemetry, declarative non-pipeline bindings) is cleanup rather than capability.

---

## 8. File reference

| File | Role |
|------|------|
| `include/posest/IFrameProducer.h` | Producer interface. |
| `include/posest/IFrameConsumer.h` | Consumer interface (`deliver()` must be non-blocking). |
| `include/posest/Frame.h` | `Frame` value type + `FramePtr = shared_ptr<const Frame>`. |
| `include/posest/ProducerBase.h` / `src/core/ProducerBase.cpp` | Capture-thread + fan-out base. |
| `include/posest/ConsumerBase.h` / `src/core/ConsumerBase.cpp` | Mailbox-driven consumer base. |
| `include/posest/LatestFrameSlot.h` / `src/core/LatestFrameSlot.cpp` | Drop-oldest single-slot mailbox. |
| `include/posest/MockProducer.h` / `src/mock/MockProducer.cpp` | Deterministic synthetic producer for tests. |
| `include/posest/MockConsumer.h` / `src/mock/MockConsumer.cpp` | Instrumented consumer for tests (latency stats, drop counts). |
| `include/posest/runtime/RuntimeGraph.h` / `src/runtime/RuntimeGraph.cpp` | Graph builder + start/stop orchestration. |
| `include/posest/pipelines/VisionPipelineBase.h` / `src/pipelines/VisionPipelineBase.cpp` | Pipeline-flavored consumer base (duplicates `ConsumerBase` plumbing, see §6.4). |
| `test/test_latest_frame_slot.cpp` | Drop-oldest semantics, shutdown, 1P/1C concurrency. |
| `test/test_producer_base.cpp` | Fan-out, sequencing, timestamp passthrough, timestamp fallback. |
| `test/test_mock_pipeline.cpp` | End-to-end producer→consumer: slow-consumer isolation, multi-consumer fan-out, clean stop. |
| `test/test_runtime_graph.cpp` | Graph construction + binding resolution. |
