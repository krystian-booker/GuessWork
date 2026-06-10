#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "consumer/rosbag_writer.hpp"
#include "core/frame_channel.hpp"
#include "core/imu_types.hpp"
#include "core/measurement_bus.hpp"

namespace gw {

// Records N camera image streams plus the IMU stream into a single ROS1 bag
// for Kalibr extrinsic calibration (kalibr_calibrate_imu_camera). The
// sibling of RosbagRecordingConsumer, which stays single-topic for the
// intrinsics workflow; this class is not an IConsumer because that
// interface is structurally one-FrameChannel.
//
// Timestamps: images are stamped with Frame::camera_ts_ns — the Teensy
// pulse clock — which is the same domain as ImuSample::t_ns, so Kalibr sees
// a shared timeline by construction (the whole point of this recorder).
// Frames whose camera_ts_ns is 0 (no Teensy pulse matched — fallback path)
// are dropped and counted: a 1970-epoch stamp would break the bag's
// per-connection time monotonicity.
//
// Threading: one pull thread per camera (copy Mono8 out of the IOSurface,
// release the frame fast), one IMU drain thread, one writer thread (ROS1
// bag offsets are sequential). Bounded queues, never block the producers:
// the image queue drops new entries when full, the IMU queue drops oldest.
struct CameraInputSpec {
    FrameChannel* channel = nullptr;
    std::string   topic;      // e.g. "/cam0/image_raw"
    std::string   frame_id;   // e.g. "cam0"
};

class MultiTopicBagRecorder {
public:
    // imu_bus may be null (no /imu0 connection is registered then) — used by
    // unit tests; production always passes the Teensy bus.
    MultiTopicBagRecorder(std::filesystem::path        session_root,
                          std::filesystem::path        target_yaml_source,
                          std::vector<CameraInputSpec> cameras,
                          MeasurementBus<ImuSample>*   imu_bus,
                          std::string                  imu_topic    = "/imu0",
                          std::string                  imu_frame_id = "imu0");
    ~MultiTopicBagRecorder();

    MultiTopicBagRecorder(const MultiTopicBagRecorder&)            = delete;
    MultiTopicBagRecorder& operator=(const MultiTopicBagRecorder&) = delete;

    // Creates the session dir, copies target.yaml alongside the bag, opens
    // the writer, subscribes everywhere, and spawns the worker threads.
    // Throws std::runtime_error on filesystem failure.
    void start();

    // Unsubscribes, joins all threads (drains queued work), closes the bag.
    // Idempotent.
    void stop();

    struct CameraStats {
        uint64_t written = 0;
        uint64_t dropped = 0;
    };
    CameraStats camera_stats(size_t camera_idx) const;
    uint64_t    imu_written() const { return imu_written_.load(std::memory_order_relaxed); }
    uint64_t    imu_dropped() const { return imu_dropped_.load(std::memory_order_relaxed); }

    const std::filesystem::path& bag_path() const { return bag_path_; }

private:
    struct ImageTask {
        uint32_t                   conn_id      = 0;
        size_t                     camera_idx   = 0;
        uint64_t                   timestamp_ns = 0;
        uint32_t                   width        = 0;
        uint32_t                   height       = 0;
        std::unique_ptr<uint8_t[]> pixels;
    };

    // ~250 ms of slack at 2 cams × 30 fps; ≈ 20 MB at 1280×960 Mono8.
    static constexpr size_t kImageQueueCapacity = 16;
    // ~10 s of IMU at 400 Hz; samples are 32 B so memory is irrelevant.
    static constexpr size_t kImuQueueCapacity = 4096;

    void run_pull(size_t camera_idx);
    void run_imu();
    void run_writer();

    struct CamState {
        CameraInputSpec               spec;
        uint32_t                      conn_id = 0;
        FrameChannel::SubscriberHandle sub;
        std::thread                   thread;
        uint64_t                      last_seq = 0;   // pull-thread only
        std::atomic<uint64_t>         written{0};
        std::atomic<uint64_t>         dropped{0};
    };

    std::filesystem::path session_root_;
    std::filesystem::path bag_path_;
    std::filesystem::path target_yaml_source_;

    std::vector<std::unique_ptr<CamState>> cams_;
    MeasurementBus<ImuSample>*             imu_bus_;
    std::string                            imu_topic_;
    std::string                            imu_frame_id_;
    uint32_t                               imu_conn_id_ = 0;
    MeasurementBus<ImuSample>::SubscriberHandle imu_sub_;
    std::thread                            imu_thread_;
    std::atomic<uint64_t>                  imu_written_{0};
    std::atomic<uint64_t>                  imu_dropped_{0};

    std::unique_ptr<RosbagWriter> writer_;
    std::thread                   writer_thread_;

    std::mutex              queue_mu_;
    std::condition_variable queue_cv_;
    std::deque<ImageTask>   image_queue_;
    std::deque<ImuSample>   imu_queue_;

    std::atomic<bool> running_{false};
    std::atomic<bool> stopping_{false};
};

}  // namespace gw
