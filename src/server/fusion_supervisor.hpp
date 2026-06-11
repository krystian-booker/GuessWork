#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "fusion/fusion_types.hpp"

namespace gw::server {

class ApriltagSupervisor;
class FusionConfigRepository;
class ImuConfigRepository;
class TeensyManager;
class VioSupervisor;

struct FusionStatus {
    bool        enabled = false;
    std::string reason;  // "ok" | "disabled" | "awaiting first tag pose"

    gw::fusion::FusedState     state;     // initialized=false ⇒ pose invalid
    gw::fusion::FusionCounters counters;  // engine counters (zeroed when disabled)

    struct SourceEntry {
        double                 rate_hz = 0.0;  // measured over ~1 s
        std::optional<int64_t> last_age_ms;
        uint64_t               bus_dropped = 0;
    };
    SourceEntry tag, vio, odom;
    bool        vio_enabled = false;  // T_robot_imu configured + parseable
    std::string vio_reason;           // why not, when disabled

    double lag_s = 0.0;  // active engine lag (config copy)

    bool   teensy_now_healthy = false;
    double teensy_now_offset_ms = 0.0;

    uint64_t output_sent        = 0;
    uint64_t output_send_errors = 0;
    uint64_t queue_dropped      = 0;
};

// Owns the fusion engine and its threads:
//   - 3 bus drainers (TagPoseBus, VioBus, OdomBus) push measurements into a
//     bounded internal queue (the odom drainer also feeds the Teensy-now
//     clock estimator),
//   - 1 engine thread drains the queue into the single-threaded
//     gw::fusion::FusionEngine and publishes a snapshot,
//   - 1 output thread extrapolates the snapshot to Teensy-now at output_hz
//     and ships it via TeensyManager::send_pose.
//
// Construct after ApriltagSupervisor / VioSupervisor / TeensyManager and
// destroy before them (declaration order in main.cpp handles both).
class FusionSupervisor {
public:
    FusionSupervisor(FusionConfigRepository& fusion_config,
                     ImuConfigRepository&    imu_config,
                     ApriltagSupervisor&     apriltag,
                     VioSupervisor&          vio,
                     TeensyManager&          teensy);
    ~FusionSupervisor();

    FusionSupervisor(const FusionSupervisor&)            = delete;
    FusionSupervisor& operator=(const FusionSupervisor&) = delete;

    // Re-reads fusion_config + imu_config (T_robot_imu). Engine-relevant
    // changes rebuild the engine (⇒ reinit); output_hz/max_extrapolation_ms
    // are live-applied. Returns true iff the engine was rebuilt.
    bool reload();

    // Manual engine reinit (does not bump the automatic-reinit counter).
    void reset();

    FusionStatus status();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace gw::server
