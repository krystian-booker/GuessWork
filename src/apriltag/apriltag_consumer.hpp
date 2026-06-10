#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "apriltag/field_layout.hpp"
#include "apriltag/pose_math.hpp"
#include "apriltag/tag_pose_estimator.hpp"
#include "apriltag/tag_pose_types.hpp"
#include "consumer/consumer.hpp"
#include "core/frame_channel.hpp"

// Forward-declare the AprilRobotics C types — the library headers stay out
// of downstream translation units.
struct apriltag_detector;
struct apriltag_family;

namespace gw::apriltag {

// Hot-swappable global configuration shared by every AprilTagConsumer:
// the active field layout, the operator's IMU→robot transform, and the
// estimator tuning. Snapshots are immutable; ApriltagSupervisor publishes a
// fresh shared_ptr on change and consumers pick it up next frame.
struct SharedTagConfig {
    std::optional<PreparedLayout> layout;
    std::optional<Mat4>           T_robot_imu;
    EstimatorConfig               est;
};

struct TagStatusEntry {
    int32_t               id              = 0;
    double                decision_margin = 0.0;
    std::optional<double> range_m;
};

struct ConsumerStatusSnapshot {
    int64_t     camera_id = 0;
    std::string camera_name;
    // "ok" | "resolution_mismatch" | "no_extrinsics_chain"
    std::string reason;
    uint64_t    frames_seen          = 0;
    uint64_t    detections_total     = 0;
    uint64_t    published            = 0;
    uint64_t    skipped_no_tags      = 0;
    uint64_t    skipped_ambiguous    = 0;
    uint64_t    skipped_high_reproj  = 0;
    uint64_t    skipped_no_extrinsics = 0;
    uint64_t    skipped_solve_failed = 0;
    double      det_per_s            = 0.0;
    double      last_latency_ms      = 0.0;
    double      latency_ewma_ms      = 0.0;
    double      mean_reproj_err_px   = 0.0;
    std::vector<TagStatusEntry>           last_tags;
    std::optional<Mat4>                   last_pose;  // T_field_robot
    int64_t                               last_pose_t_ns = 0;
};

// Per-camera AprilTag detection worker. Pulls from the camera's latest-only
// FrameChannel (stale detections are worthless — skipping is correct), runs
// the detector zero-copy on the locked Mono8 IOSurface base address, then
// solves and publishes field-frame robot poses on the TagPoseBus.
//
// Detection settings: tag36h11, quad_decimate=2, refine_edges, nthreads=2;
// detections filtered to hamming==0 && decision_margin > threshold.
//
// One detector instance per consumer (concurrent detect on one detector is
// not safe); it lives and dies with the consumer.
class AprilTagConsumer final : public gw::IConsumer {
public:
    AprilTagConsumer(int64_t                                camera_id,
                     std::string                            camera_name,
                     PinholeCamera                          intrinsics,
                     std::optional<Mat4>                    T_cam_imu,
                     std::shared_ptr<TagPoseBus>            bus,
                     std::shared_ptr<const SharedTagConfig> shared);
    ~AprilTagConsumer() override;

    AprilTagConsumer(const AprilTagConsumer&)            = delete;
    AprilTagConsumer& operator=(const AprilTagConsumer&) = delete;

    std::string_view name() const override { return "apriltag"; }
    void             attach(gw::FrameChannel& ch) override;
    void             detach() override;

    void update_shared(std::shared_ptr<const SharedTagConfig> shared);

    ConsumerStatusSnapshot snapshot() const;

private:
    void run();
    void process_frame(gw::Frame* f);

    const int64_t             camera_id_;
    const std::string         camera_name_;
    const PinholeCamera       intrinsics_;
    const std::optional<Mat4> T_cam_imu_;

    std::shared_ptr<TagPoseBus> bus_;

    mutable std::mutex                     shared_mu_;
    std::shared_ptr<const SharedTagConfig> shared_;

    apriltag_detector* detector_ = nullptr;
    apriltag_family*   family_   = nullptr;

    gw::FrameChannel*               channel_ = nullptr;
    gw::FrameChannel::SubscriberHandle sub_;
    std::thread                     worker_;
    std::atomic<bool>               running_{false};

    // Counters (worker writes, snapshot reads).
    std::atomic<uint64_t> frames_seen_{0};
    std::atomic<uint64_t> detections_total_{0};
    std::atomic<uint64_t> published_{0};
    std::atomic<uint64_t> skipped_no_tags_{0};
    std::atomic<uint64_t> skipped_ambiguous_{0};
    std::atomic<uint64_t> skipped_high_reproj_{0};
    std::atomic<uint64_t> skipped_no_extrinsics_{0};
    std::atomic<uint64_t> skipped_solve_failed_{0};
    std::atomic<bool>     resolution_mismatch_{false};

    // Rolling detection rate (FpsSampler pattern) + latency. Guarded by
    // stats_mu_ together with the last_* fields.
    mutable std::mutex          stats_mu_;
    std::vector<TagStatusEntry> last_tags_;
    std::optional<Mat4>         last_pose_;
    int64_t                     last_pose_t_ns_     = 0;
    double                      last_latency_ms_    = 0.0;
    double                      latency_ewma_ms_    = 0.0;
    double                      mean_reproj_err_px_ = 0.0;
    double                      det_per_s_          = 0.0;
    uint64_t                    rate_last_count_    = 0;
    std::chrono::steady_clock::time_point rate_last_t_{};
};

}  // namespace gw::apriltag
