#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "apriltag/apriltag_consumer.hpp"
#include "apriltag/tag_pose_types.hpp"

namespace gw {
class IConsumer;
}

namespace gw::server {

class CameraRepository;
class FieldLayoutRepository;
class ImuConfigRepository;
struct Camera;

struct ApriltagStatus {
    std::optional<int64_t>     active_layout_id;
    std::optional<std::string> active_layout_name;
    bool                       t_robot_imu_set = false;
    // One entry per camera with role='apriltag'. Cameras whose factory
    // declined (no intrinsics / parse failure) carry the recorded reason and
    // empty counters; the route layer overlays "offline" from
    // CameraSupervisor::snapshot_all separately (no lock nesting here).
    struct CameraEntry {
        int64_t                                 camera_id = 0;
        std::string                             name;
        bool                                    running = false;
        std::string                             reason;  // "ok" | gating reason
        gw::apriltag::ConsumerStatusSnapshot    stats;   // valid when running
    };
    std::vector<CameraEntry> cameras;
};

// Registry + configuration hub for the AprilTag detection pipeline:
//   - owns the TagPoseBus every AprilTagConsumer publishes to,
//   - owns the hot-swappable SharedTagConfig (active field layout +
//     T_robot_imu + estimator tuning) and pushes snapshots to consumers,
//   - acts as the CameraSupervisor consumer factory for role='apriltag'
//     cameras (register make_consumer via register_consumer_factory).
//
// Locking: only its own mu_. make_consumer runs under the camera
// supervisor's lock (factory contract) and must not call back into it;
// status() composes purely from this registry + repositories.
class ApriltagSupervisor {
public:
    ApriltagSupervisor(CameraRepository&      cameras,
                       FieldLayoutRepository& field_layouts,
                       ImuConfigRepository&   imu_config);

    // The ConsumerFactory body. Returns null (and records why) unless the
    // row has role='apriltag' and parseable intrinsics. Prefers
    // imu_extrinsics_json (refined intrinsics + T_cam_imu); falls back to
    // calibration_json (intrinsics-only — detection runs, publish gated).
    std::shared_ptr<gw::IConsumer> make_consumer(const Camera& row);

    // Re-reads the active field layout + imu_config and pushes a fresh
    // SharedTagConfig snapshot into every registered consumer. Called after
    // layout activation and T_robot_imu updates.
    void reload_shared();

    std::shared_ptr<gw::apriltag::TagPoseBus> bus() const { return bus_; }

    ApriltagStatus status();

private:
    std::shared_ptr<const gw::apriltag::SharedTagConfig> build_shared_locked();

    CameraRepository&      cameras_;
    FieldLayoutRepository& field_layouts_;
    ImuConfigRepository&   imu_config_;

    std::shared_ptr<gw::apriltag::TagPoseBus> bus_;

    std::mutex mu_;
    std::shared_ptr<const gw::apriltag::SharedTagConfig> shared_;
    std::optional<int64_t>                               active_layout_id_;
    std::optional<std::string>                           active_layout_name_;
    // weak_ptr: the CameraSlot owns the consumer's lifetime; a dead entry
    // here just means the slot tore it down (camera offline).
    std::map<int64_t, std::weak_ptr<gw::apriltag::AprilTagConsumer>> consumers_;
    std::map<int64_t, std::string> factory_reasons_;  // why make_consumer declined
};

}  // namespace gw::server
