# Producer/Consumer Architecture

**Status:** Core plumbing is feature-complete and now supports **dynamic subscription**: `addConsumer()` / `removeConsumer()` may be called at any time, including while a producer is running. `ProducerBase` also exposes a `state()` enum so the graph can detect a capture loop that exited on its own. The remaining gap is the user-facing **video-preview consumer** (`PreviewConsumer`) and its declarative binding entry — both deferred until an HTTP server is wired into `WebService`.

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

The explicit motivating case is a **display consumer** that streams frames to the web UI for live preview. The architecture now supports this end-to-end: the preview is just another `IFrameConsumer` subscribed to the camera, and §5.1's live subscriber list lets it attach/detach on demand without restarting the camera. The display consumer would:

- Accept a frame in `deliver()` → put it in its own `LatestFrameSlot`.
- A worker thread takes the latest frame, JPEG-encodes it (or hands it to an MJPEG HTTP endpoint), and streams it.
- If encoding is slow, its mailbox drops stale frames; the primary vision pipeline attached to the same camera is untouched.

**The plumbing is ready; the consumer itself isn't.** There is still no `PreviewConsumer` / `DisplayConsumer` implementation, and `WebService` has no HTTP server to host an MJPEG endpoint. Both are deferred together (see §6.2).

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

### 5.1 Subscriber list is live (hot attach/detach supported)

`ProducerBase::runLoop()` refreshes the subscriber snapshot under `consumers_mu_` *each frame*:

```cpp
std::vector<std::shared_ptr<IFrameConsumer>> subscribers;
while (state_.load(std::memory_order_acquire) == ProducerState::Running) {
    // captureOne() / timestamp / sequence ...
    {
        std::lock_guard<std::mutex> g(consumers_mu_);
        subscribers = consumers_;
    }
    for (auto& c : subscribers) c->deliver(pub);
}
```

Cost: one uncontended mutex acquisition + a small `vector<shared_ptr>` copy per captured frame. At 6 cameras × 60 FPS this is ~360 acquires/sec — well below any threshold that affects capture latency.

`addConsumer()` and `removeConsumer()` are safe to call at any time, including from another thread while the producer is running. Consumer identity is by `shared_ptr` equality (`removeConsumer()` returns `false` if the pointer is unknown). Frames already pushed into a removed consumer's mailbox are not recalled — the caller still owns the consumer's `start()` / `stop()` lifecycle. The end-to-end attach/detach behavior is exercised by `MockPipeline.HotPreviewAttachDoesNotDisturbPrimary` and three `ProducerBase.HotAdd…` / `HotRemove…` / `AddRemoveDuringCaptureDoesNotDeadlock` tests.

---

## 6. Gap analysis vs. the stated requirements

| Requirement | Status | Notes |
|-------------|--------|-------|
| One thread per producer | ✅ Complete | `ProducerBase::worker_` per producer. |
| One thread per consumer | ✅ Complete | `ConsumerBase::worker_` (now also serves `VisionPipelineBase`; see §6.4). `CalibrationRecorder` still rolls its own worker. |
| Minimal latency — only keep newest frame | ✅ Complete | `LatestFrameSlot` drop-oldest, per-consumer. |
| Producer never blocked by slow consumer | ✅ Complete | Enforced by `put()` non-blocking + `deliver()` contract. Verified in `test_mock_pipeline.cpp`. |
| Scales to ~6 producers | ✅ Complete | Independent, no shared state between producers. No structural limit. |
| 1-to-many fan-out | ✅ Complete | Static and dynamic. Per-frame mutex-protected snapshot in `runLoop`. |
| Dynamic subscribe/unsubscribe (e.g. live preview attach) | ✅ Complete | `addConsumer()` / `removeConsumer()` are safe at any time. See §6.1. |
| Plumbing dedupe (`VisionPipelineBase` ↔ `ConsumerBase`) | ✅ Complete | `VisionPipelineBase` now inherits `ConsumerBase`. See §6.4. |
| Producer-died telemetry | 🚧 Partial | `ProducerState` enum + `state()` and the `ProducerBase` state machine are wired; `RuntimeGraph::deadProducers()` is declared but its implementation, the warn-log on graph stop, and dedicated tests are still pending. See §6.5. |
| **Video-preview consumer implementation** | ❌ Deferred | No `PreviewConsumer` class; `WebService` has no HTTP server to host MJPEG yet. Co-deferred with non-pipeline binding scaffolding (§6.3). |
| Declarative non-pipeline bindings in `RuntimeConfig` | ❌ Deferred | Co-motivated by preview; design once a concrete consumer exists. |
| Cross-camera timestamp domain | ✅ Complete | `steady_clock` enforced by `ProducerBase`. |
| Shutdown ordering / clean teardown | ✅ Complete | `RuntimeGraph` + base destructors + tests. |

### 6.1 Dynamic subscription for live preview — **DONE**

Resolved with **Option A** (mutex-protected live list). `IFrameProducer` gained `removeConsumer(const std::shared_ptr<IFrameConsumer>&) -> bool`, and `ProducerBase::runLoop` now refreshes its subscriber snapshot under `consumers_mu_` once per captured frame (see §5.1 for the body). Identity is by `shared_ptr` equality; double-removes return `false`.

Test coverage:

- `ProducerBase.HotAddConsumerSeesOnlyFramesAfterAttach` — late-attaching consumer's first observed sequence is ≥ the producer's frame count at the moment of attach.
- `ProducerBase.HotRemoveConsumerStopsReceiving` — after `removeConsumer` returns and a brief drain window, the removed consumer receives no further frames.
- `ProducerBase.RemoveConsumerNotPresentReturnsFalse` — bool contract.
- `ProducerBase.AddRemoveDuringCaptureDoesNotDeadlock` — random attach/detach churn at 240 FPS for 300 ms; primary consumer's FPS stays ≥ 50 % of target, no deadlock.
- `MockPipeline.HotPreviewAttachDoesNotDisturbPrimary` — end-to-end: a slow auxiliary consumer attaches mid-stream, runs 400 ms, detaches; the primary consumer's FPS stays ≥ 70 % of 60 FPS across both windows.

The "attach a preview on demand" sequence the architecture now permits:

1. UI opens video feed page.
2. Web service creates a `PreviewConsumer` (still to be built — see §6.2), starts it, and calls `camera->addConsumer(preview)`.
3. UI closes page. Web service calls `camera->removeConsumer(preview)`, stops it.
4. Camera keeps running; the primary pipeline is untouched.

### 6.2 Video-preview consumer — **DEFERRED**

No `PreviewConsumer` / MJPEG-over-HTTP streamer exists yet. `posest::runtime::WebService` is currently a thin holder for config + telemetry state — it has no HTTP server, no route table, and no streaming endpoint. The implementation is deferred until that HTTP layer lands; without it, a `PreviewConsumer` would have no way to expose its encoded frames.

§6.1 has unblocked the plumbing side: a future `PreviewConsumer` can attach/detach against a running camera at will. The remaining work is:

1. Pick and add an HTTP library (cpp-httplib is the lightest fit) to `conanfile.txt`.
2. Extend `WebService` with route registration and a chunked / `multipart/x-mixed-replace` response path.
3. Implement the `PreviewConsumer` itself. Suggested shape:

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

   Encoding stays on the consumer's own thread, off the producer thread — consistent with the latency contract.

### 6.3 Non-pipeline bindings in `RuntimeConfig` — **DEFERRED (with §6.2)**

`RuntimeConfig::bindings` still only models camera → vision pipeline. Auxiliary consumers like the calibration recorder are wired imperatively in `Daemon.cpp:613–664` (the `RecordKalibrDataset` one-shot CLI), and that pattern is the right home for them — they're not part of the long-running runtime graph.

The only motivation for a declarative non-pipeline binding category is preview ("always-on low-res thumbnail stream per camera"). Without `PreviewConsumer` (§6.2), there is no concrete consumer to bind, so the data-model design is deferred alongside §6.2. Note that the imperative attach path from a future web service is probably the right primary mode for preview anyway.

### 6.4 Plumbing-level duplication: `ConsumerBase` vs. `VisionPipelineBase` — **DONE**

`VisionPipelineBase` now inherits `ConsumerBase` and only carries `type_` and `IMeasurementSink&`. The duplicated `thread + LatestFrameSlot + deliver/put/take/process` plumbing was removed; `process()` is overridden once with `final` to forward to the existing `processFrame()` hook so concrete pipelines (`AprilTagPipeline`, placeholder pipelines) compile unchanged. To resolve the diamond on `IFrameConsumer` between `ConsumerBase` and `runtime::IVisionPipeline`, both bases now use `public virtual IFrameConsumer`. The `posest_pipelines` and `posest_runtime_graph` test suites pass without modification.

### 6.5 Producer-side telemetry for "I died" — **PARTIAL**

`IFrameProducer` now exposes `ProducerState state() const`, with values `Idle | Running | EndOfStream | Failed`. `ProducerBase` owns a `std::atomic<ProducerState>` that:

- starts at `Idle`,
- transitions `Idle → Running` via CAS in `start()` (rejecting restart from terminal states),
- writes `EndOfStream` (via `Running → EndOfStream` CAS) when `captureOne()` returns `false`,
- writes `Failed` (via `Running → Failed` CAS) when `captureOne()` propagates an exception,
- returns to `Idle` in `stop()`, joining the worker thread either way (terminal-state stops still join cleanly).

The CAS guards ensure that a concurrent `stop()` always wins — operators who explicitly stopped don't see a spurious `EndOfStream`/`Failed` afterwards. Consumers' mailboxes still need to be `stop()`'d by the owner; `state()` is what surfaces "the producer left on its own" to the polling caller.

**Still pending (small):**

- `RuntimeGraph::deadProducers()` is declared in the header but the body is not implemented yet; once implemented it will scan `cameras_` and return the ids whose `state()` is `EndOfStream` or `Failed`.
- `RuntimeGraph::stop()` should warn-log when a camera is already in a terminal state at shutdown so silent deaths surface even when nothing was polling.
- Dedicated tests for the new state machine (`StateRunningWhileCapturing`, `StateBecomesEndOfStreamWhenCaptureOneReturnsFalse`, `StateBecomesFailedOnException`, `StopJoinsCleanlyAfterEndOfStream`).

The deeper camera-lifecycle discussion still lives in [`camera-producer.md`](camera-producer.md) §6.2; this section covers only the plumbing-level signal.

### 6.6 Decode-on-producer-thread tradeoff (informational, not a gap)

MJPEG decode happens on the producer thread (`V4L2Producer::grabFrame`), so the Mat that goes into the fan-out is already decoded. This is a deliberate choice: decode once, fan out to N consumers, instead of decoding once per consumer. At 6 cameras × 1080p MJPEG × 60 FPS this is well within budget on a modern CPU. At 6 × 4K × 60 FPS it may not be — at that point, per-consumer decode or off-thread decode would need to be introduced. Not a current requirement.

---

## 7. Summary

The core plumbing (interfaces, base classes, drop-oldest mailbox, fan-out, startup/shutdown, timestamp discipline) implements the stated requirements cleanly. Multi-producer scales trivially because nothing is shared between producers; 1-to-many fan-out is a first-class feature and its isolation properties are covered by tests.

Dynamic subscription (§6.1) and the `VisionPipelineBase` / `ConsumerBase` dedupe (§6.4) have landed. Producer-side state telemetry (§6.5) is partially in: the `ProducerState` enum and the `ProducerBase` state machine are wired and CAS-safe against concurrent `stop()`, but the `RuntimeGraph::deadProducers()` body, the warn-log on graph stop, and the dedicated tests are still pending.

The remaining product-visible gap is the user-facing **video-preview consumer** (§6.2). The plumbing is ready for it, but it can't ship until `WebService` grows an HTTP server to host an MJPEG endpoint. Declarative non-pipeline bindings (§6.3) are co-deferred with §6.2 — without a concrete preview consumer there's nothing to bind.

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
