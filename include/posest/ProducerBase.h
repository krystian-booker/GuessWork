#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "posest/Frame.h"
#include "posest/IFrameConsumer.h"
#include "posest/IFrameProducer.h"

namespace posest {

class CameraTriggerCache;
class ToFSampleCache;

// Reusable base for any frame source.
//
// Owns: the capture thread, the subscriber list, the sequence counter, and the
// timestamping discipline (steady_clock, stamped inside the base right after
// captureOne() returns). Subclasses only implement captureOne() — they write
// an image into the out Mat and return true, or return false to end the stream.
class ProducerBase : public IFrameProducer {
public:
    explicit ProducerBase(std::string id);
    ~ProducerBase() override;

    ProducerBase(const ProducerBase&) = delete;
    ProducerBase& operator=(const ProducerBase&) = delete;

    const std::string& id() const override { return id_; }

    // Safe to call at any time, including while the producer is running.
    // The capture loop refreshes its subscriber snapshot every frame.
    void addConsumer(std::shared_ptr<IFrameConsumer> consumer) override;
    [[nodiscard]] bool removeConsumer(
        const std::shared_ptr<IFrameConsumer>& consumer) override;

    [[nodiscard]] ProducerState start() override;
    void stop() override;

    ProducerState state() const override { return state_.load(std::memory_order_acquire); }

    std::uint64_t producedCount() const { return produced_.load(); }

    // Wire (or unwire) a CameraTriggerCache. When set, the capture loop looks
    // up a Teensy-stamped trigger for each frame's capture_time and stamps
    // the resulting Frame::teensy_time_us / trigger_sequence. Safe to call at
    // any time; the load is atomic so the producer thread observes updates
    // without locking.
    void setTriggerCache(std::shared_ptr<const CameraTriggerCache> cache);

    // Wire (or unwire) a ToFSampleCache. The producer only attempts a lookup
    // when a trigger_sequence was already paired by the trigger cache; the
    // join is keyed on that sequence (firmware tags each ranging with the
    // originating camera trigger's sequence). Lookups for non-VIO cameras
    // miss harmlessly.
    void setToFSampleCache(std::shared_ptr<const ToFSampleCache> cache);

protected:
    // Produce one frame. Return true on success (frame will be fanned out),
    // false to signal end-of-stream (capture loop exits).
    //
    // Timestamping contract:
    //   - If the backend exposes a native capture timestamp (V4L2 buffer
    //     timestamp, GenICam/PTP hardware stamp, vendor SDK metadata, etc.),
    //     the subclass MUST convert it into the std::chrono::steady_clock
    //     domain and assign it to out_capture_time. For V4L2 with
    //     V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC this is a direct construction
    //     from the kernel timespec; for cameras with their own epoch, do a
    //     one-shot startup calibration (capture steady_clock::now() and the
    //     camera tick together, store the offset) and apply it per-frame.
    //   - If the subclass leaves out_capture_time as std::nullopt, the base
    //     will fall back to stamping steady_clock::now() immediately after
    //     captureOne() returns. This is the best userspace approximation
    //     but is subject to scheduling jitter — real producers with access
    //     to a hardware/kernel timestamp should always supply one.
    //
    // All consumers observe Frame::capture_time in the steady_clock domain,
    // regardless of which backend produced it. This keeps cross-camera
    // timestamp math well-defined (required for VIO/multi-camera fusion).
    virtual bool captureOne(
        cv::Mat& out,
        std::optional<std::chrono::steady_clock::time_point>& out_capture_time) = 0;

    // Hook for subclasses that want to check the running flag themselves
    // (e.g. to break out of a blocking read). Base calls this in the loop.
    bool isRunning() const {
        return state_.load(std::memory_order_acquire) == ProducerState::Running;
    }

private:
    void runLoop();

    std::string id_;
    std::vector<std::shared_ptr<IFrameConsumer>> consumers_;
    std::mutex consumers_mu_;  // guards consumers_ across add/remove and the per-frame snapshot in runLoop

    std::thread worker_;
    std::atomic<ProducerState> state_{ProducerState::Idle};
    std::atomic<std::uint64_t> produced_{0};
    std::uint64_t next_sequence_{0};
    std::shared_ptr<const CameraTriggerCache> trigger_cache_;
    std::shared_ptr<const ToFSampleCache> tof_cache_;
};

}  // namespace posest
