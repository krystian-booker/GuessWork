#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "consumer/consumer.hpp"
#include "core/frame_channel.hpp"

namespace gw {

// Subscribes to a FrameChannel and writes Mono8 frames to disk in EuRoC
// dataset layout, suitable for hand-off to `basalt_calibrate --dataset-type
// euroc`:
//
//   <root>/mav0/cam0/data/<host_capture_ns>.png
//   <root>/mav0/cam0/data.csv     (header: "#timestamp [ns],filename")
//
// Pipeline shape:
//   pull thread  →  bounded queue (heap-buffer tasks)  →  N encode workers
//
// The pull thread copies the Mono8 plane out of the Frame into a tight heap
// buffer and releases the Frame back to its (small) pool immediately. Encode
// workers run libspng on those independent buffers in parallel. This keeps
// the producer's frame pool hot regardless of encoder backlog and lets us
// saturate multiple cores when the camera frame rate exceeds single-thread
// PNG encode throughput.
//
// Drop policy: if the queue is full when the pull thread tries to enqueue,
// the new task is dropped and `frames_dropped()` is bumped. This mirrors the
// FrameChannel's "latest-only" behavior — backpressure never leaks back into
// the producer.
//
// CSV rows are appended by workers under a mutex; with parallel encoders the
// rows are not guaranteed to be in monotonic timestamp order. EuRoC consumers
// (including Basalt) index by timestamp / filename, not row order.
class RecordingConsumer final : public IConsumer {
public:
    // root_dir must already exist or be creatable.
    explicit RecordingConsumer(std::filesystem::path root_dir,
                               std::string           name = "recording");
    ~RecordingConsumer() override;

    RecordingConsumer(const RecordingConsumer&)            = delete;
    RecordingConsumer& operator=(const RecordingConsumer&) = delete;

    std::string_view name() const override { return name_; }

    void attach(FrameChannel& ch) override;
    void detach() override;

    // Diagnostic counters. Both monotonic; safe to read concurrently.
    uint64_t frames_written() const { return frames_written_.load(std::memory_order_relaxed); }
    uint64_t frames_dropped() const { return frames_dropped_.load(std::memory_order_relaxed); }

    const std::filesystem::path& cam0_dir() const { return cam0_dir_; }
    const std::filesystem::path& root_dir() const { return root_dir_; }

private:
    struct EncodeTask {
        uint64_t                     timestamp_ns = 0;
        uint32_t                     width        = 0;
        uint32_t                     height       = 0;
        std::unique_ptr<uint8_t[]>   pixels;  // tightly packed, width*height bytes
    };

    void run_pull();
    void run_worker();
    void close_csv();

    // Tunables. Queue capacity × worker count bounds peak resident heap for
    // pixel copies (e.g. 4 × 2048×1536 ≈ 12 MB at full HD-ish Mono8).
    static constexpr size_t kQueueCapacity = 4;
    static constexpr size_t kWorkerCount   = 4;

    std::string                name_;
    std::filesystem::path      root_dir_;
    std::filesystem::path      cam0_dir_;
    std::filesystem::path      data_dir_;

    std::mutex                 csv_mu_;
    std::ofstream              csv_;

    FrameChannel*                      channel_ = nullptr;
    FrameChannel::SubscriberHandle     sub_;

    std::atomic<bool>          running_{false};   // pull thread loop guard
    std::atomic<bool>          stopping_{false};  // wakes workers to drain + exit
    std::thread                pull_thread_;
    std::vector<std::thread>   workers_;

    std::mutex                          queue_mu_;
    std::condition_variable             queue_cv_;
    std::deque<EncodeTask>              queue_;

    std::atomic<uint64_t>      frames_written_{0};
    std::atomic<uint64_t>      frames_dropped_{0};
    uint64_t                   last_seq_ = 0;

    // sensor.yaml is written once the first frame's dimensions are known.
    // Latched to prevent later workers from rewriting it.
    std::atomic<bool>          wrote_sensor_yaml_{false};
};

}  // namespace gw
