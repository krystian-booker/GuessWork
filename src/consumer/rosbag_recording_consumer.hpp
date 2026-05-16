#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>

#include "consumer/consumer.hpp"
#include "consumer/rosbag_writer.hpp"
#include "core/frame_channel.hpp"

namespace gw {

// Subscribes to a FrameChannel and writes Mono8 frames into a single ROS1
// bag at `<session_root>/calibration.bag`, plus a copy of the AprilGrid
// target config at `<session_root>/target.yaml` (so the suggested docker
// command has bag + target under one mount root).
//
// Pipeline shape:
//   pull thread  →  bounded queue (heap-buffer tasks)  →  single writer thread
//
// The pull thread mirrors the old RecordingConsumer: copy the Mono8 plane out
// of the IOSurface-backed CVPixelBuffer into a tight heap buffer, then
// release the Frame immediately so the producer's small (~6-slot) pool
// stays hot regardless of writer backlog. Drop policy matches: a full queue
// drops the new task and bumps `frames_dropped()`.
//
// We use a single writer thread (not a pool like the PNG version did)
// because ROS1 bag offsets are sequential and parallelising would require
// a lock that costs more than it saves. At 20 Hz × 1280×960 Mono8 a single
// ofstream keeps up trivially.
class RosbagRecordingConsumer final : public IConsumer {
public:
    // session_root: directory to populate (must be creatable).
    // target_yaml_source: file copied into session_root/target.yaml at
    //   attach() time. Empty path skips the copy.
    // topic / frame_id: ROS topic + Image.header.frame_id. Defaults match
    //   the suggested Kalibr command in CalibrationSupervisor.
    explicit RosbagRecordingConsumer(std::filesystem::path session_root,
                                     std::filesystem::path target_yaml_source = {},
                                     std::string           topic    = "/cam0/image_raw",
                                     std::string           frame_id = "cam0",
                                     std::string           name     = "calibration");
    ~RosbagRecordingConsumer() override;

    RosbagRecordingConsumer(const RosbagRecordingConsumer&)            = delete;
    RosbagRecordingConsumer& operator=(const RosbagRecordingConsumer&) = delete;

    std::string_view name() const override { return name_; }
    void attach(FrameChannel& ch) override;
    void detach() override;

    uint64_t frames_written() const { return frames_written_.load(std::memory_order_relaxed); }
    uint64_t frames_dropped() const { return frames_dropped_.load(std::memory_order_relaxed); }

    const std::filesystem::path& session_root() const { return session_root_; }
    const std::filesystem::path& bag_path()     const { return bag_path_; }

private:
    struct ImageTask {
        uint64_t                   timestamp_ns = 0;
        uint32_t                   width        = 0;
        uint32_t                   height       = 0;
        std::unique_ptr<uint8_t[]> pixels;  // tightly packed, width*height bytes
    };

    void run_pull();
    void run_writer();

    // Queue capacity sized so we have ~half-a-second of slack at 20 Hz; the
    // single writer thread is the only consumer, and at expected mono8
    // bandwidth it keeps up without backpressure.
    static constexpr size_t kQueueCapacity = 8;

    std::string                       name_;
    std::filesystem::path             session_root_;
    std::filesystem::path             bag_path_;
    std::filesystem::path             target_yaml_source_;
    std::string                       topic_;
    std::string                       frame_id_;

    std::unique_ptr<RosbagWriter>     writer_;

    FrameChannel*                     channel_ = nullptr;
    FrameChannel::SubscriberHandle    sub_;

    std::atomic<bool>                 running_{false};
    std::atomic<bool>                 stopping_{false};
    std::thread                       pull_thread_;
    std::thread                       writer_thread_;

    std::mutex                        queue_mu_;
    std::condition_variable           queue_cv_;
    std::deque<ImageTask>             queue_;

    std::atomic<uint64_t>             frames_written_{0};
    std::atomic<uint64_t>             frames_dropped_{0};
    uint64_t                          last_seq_ = 0;
};

}  // namespace gw
