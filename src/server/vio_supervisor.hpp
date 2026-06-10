#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "vio/openvins_runner.hpp"
#include "vio/stereo_sync_consumer.hpp"
#include "vio/vio_types.hpp"

namespace gw {
class IConsumer;
}

namespace gw::server {

class CameraRepository;
class ImuConfigRepository;
class VioConfigRepository;
class TeensyManager;
struct Camera;

struct VioStatus {
    bool        enabled = false;
    std::string reason;  // "ok" when the runner is up, else the gating reason

    bool     running     = false;  // runner exists
    bool     initialized = false;
    std::string phase;
    uint64_t epoch   = 0;
    uint64_t reinits = 0;
    double   freq_hz = 0.0;
    uint32_t tracked_features = 0;
    double   cov_pos_std_m    = 0.0;
    std::optional<gw::vio::VioOdometry> last;

    gw::vio::StereoSyncPairer::Counters pair_counters;
    uint64_t frames_fed      = 0;
    uint64_t imu_fed         = 0;
    uint64_t imu_bus_dropped = 0;
    double   imu_rate_hz     = 0.0;  // from TeensyManager

    struct CameraEntry {
        int64_t                    camera_id = 0;
        std::string                name;
        std::string                role;            // vio_left | vio_right
        bool                       feeder_running = false;
        std::optional<double>      reproj_std_px;   // from guesswork_meta
    };
    std::vector<CameraEntry> cameras;
};

// Registry + lifecycle hub for the OpenVINS pipeline:
//   - owns the VioBus, the StereoSyncPairer, and (when gating passes) the
//     OpenVinsRunner,
//   - acts as a CameraSupervisor consumer factory for the vio_left /
//     vio_right cameras (per-camera VioFeederConsumer into the shared
//     pairer — the runner's lifetime is independent of camera slots),
//   - gates enablement on calibration quality and rebuilds the runner when
//     the configuration fingerprint changes (calibration upload, vio_config
//     PUT, role changes) — calibration is consumed programmatically.
//
// Locking: only its own mu_. make_consumer runs under the camera
// supervisor's lock (factory contract — must not call back into it);
// status() composes from own state + repositories only.
class VioSupervisor {
public:
    VioSupervisor(CameraRepository&    cameras,
                  ImuConfigRepository& imu_config,
                  VioConfigRepository& vio_config,
                  TeensyManager&       teensy);
    ~VioSupervisor();

    // ConsumerFactory body: returns a VioFeederConsumer for vio_left /
    // vio_right rows (null otherwise) and re-evaluates the runner.
    std::shared_ptr<gw::IConsumer> make_consumer(const Camera& row);

    // Re-gates and rebuilds the runner if the fingerprint changed. Called
    // after config/calibration mutations and once at boot.
    void reload();

    // Forces a VIO reinitialization (epoch bump). No-op when not running.
    void restart();

    std::shared_ptr<gw::vio::VioBus> bus() const { return bus_; }

    VioStatus status();

private:
    struct Fingerprint {
        int64_t left_id = 0, right_id = 0;
        int64_t left_extr_at = 0, right_extr_at = 0;
        int64_t vio_config_at = 0, imu_config_at = 0;
        bool    operator==(const Fingerprint&) const = default;
    };

    // Re-evaluates gating + runner under mu_. Sets reason_.
    void ensure_runner_locked();

    CameraRepository&    cameras_;
    ImuConfigRepository& imu_config_;
    VioConfigRepository& vio_config_;
    TeensyManager&       teensy_;

    std::shared_ptr<gw::vio::VioBus>           bus_;
    std::shared_ptr<gw::vio::StereoSyncPairer> pairer_;
    std::shared_ptr<std::atomic<uint64_t>>     epoch_;

    std::mutex                              mu_;
    std::unique_ptr<gw::vio::OpenVinsRunner> runner_;
    std::optional<Fingerprint>              fingerprint_;
    std::string                             reason_ = "not evaluated";
    std::map<int64_t, std::weak_ptr<gw::vio::VioFeederConsumer>> feeders_;
};

}  // namespace gw::server
