#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "core/imu_types.hpp"
#include "core/measurement_bus.hpp"
#include "vio/stereo_sync_consumer.hpp"
#include "vio/vio_config_builder.hpp"
#include "vio/vio_types.hpp"

// Owns one OpenVINS VioManager and the single thread that feeds it.
//
// Threading: ONE runner thread does everything — waits for a stereo pair
// from the pairer, drains the ImuBus until IMU coverage extends past the
// pair's timestamp, calls feed_measurement_camera (synchronous; OpenVINS's
// camera path has no internal locking, so single-threaded feeding is the
// only shape that needs no assumptions about upstream thread-safety), then
// extracts pose + marginal covariance and publishes on the VioBus.
//
// Reinit: reconstructs the VioManager from the same config (cheap). The
// supervisor-owned epoch counter increments so downstream consumers know
// the odom origin moved (see VioOdometry's fusion contract). Triggers:
// tracked-feature collapse, position-covariance explosion, an inter-pair
// gap > 2 s (camera unplug/replug), or request_reinit().
//
// Pimpl: OpenVINS / Eigen / OpenCV headers stay out of this header.

namespace gw::vio {

struct VioReinitPolicy {
    bool   auto_reinit          = true;
    int    min_features         = 15;
    int    window_frames        = 15;
    double max_pos_std_m        = 2.0;
};

class OpenVinsRunner {
public:
    OpenVinsRunner(VioRunnerConfig                    cfg,
                   VioReinitPolicy                    policy,
                   std::shared_ptr<StereoSyncPairer>  pairer,
                   gw::MeasurementBus<gw::ImuSample>& imu_bus,
                   std::shared_ptr<VioBus>            out_bus,
                   std::shared_ptr<std::atomic<uint64_t>> epoch);
    ~OpenVinsRunner();  // shuts the pairer, unsubscribes, joins

    OpenVinsRunner(const OpenVinsRunner&)            = delete;
    OpenVinsRunner& operator=(const OpenVinsRunner&) = delete;

    // Async: the runner loop reinitializes before the next pair.
    void request_reinit();

    struct Snapshot {
        bool        initialized = false;
        std::string phase;  // "initializing" | "tracking"
        uint64_t    epoch       = 0;
        uint64_t    reinits     = 0;
        uint64_t    frames_fed  = 0;
        uint64_t    imu_fed     = 0;
        uint64_t    imu_bus_dropped = 0;
        double      freq_hz     = 0.0;  // update rate, 1 s rolling window
        double      cov_pos_std_m = 0.0;
        uint32_t    tracked_features = 0;
        std::optional<VioOdometry> last;
    };
    Snapshot snapshot() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace gw::vio
