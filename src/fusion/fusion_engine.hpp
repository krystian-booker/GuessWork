#pragma once

#include <memory>

#include "apriltag/tag_pose_types.hpp"
#include "core/odom_types.hpp"
#include "fusion/fusion_types.hpp"
#include "vio/vio_types.hpp"

namespace gw::fusion {

// Single-threaded fusion core: one GTSAM IncrementalFixedLagSmoother fusing
// tag poses (absolute priors, Mahalanobis-gated + Huber), VIO (equal-epoch
// relative deltas rotated into the robot frame, Huber) and chassis speeds
// (integrated body twist, Cauchy — slip-robust). GTSAM lives entirely in the
// .cpp Pimpl.
//
// THREADING: not thread-safe by design. The owner (FusionSupervisor's engine
// thread, or a test) provides the single thread; every method — including
// state()/counters() — must be called from it.
//
// Lifecycle: feed_* in measurement-arrival order. The engine initializes
// itself from the first few tag poses (geometric medoid of 5); before that,
// state().initialized is false and VIO/odom samples only prime accumulators.
// reset() drops the graph and returns to the uninitialized state.
class FusionEngine {
public:
    explicit FusionEngine(const FusionParams& params);
    ~FusionEngine();

    FusionEngine(const FusionEngine&)            = delete;
    FusionEngine& operator=(const FusionEngine&) = delete;

    void feed_tag(const gw::apriltag::TagPoseMeasurement& m);
    void feed_vio(const gw::vio::VioOdometry& m);
    void feed_odom(const gw::ChassisSpeeds& m);
    // void feed_imu(const gw::ImuSample&);  // reserved: Phase 7 IMU
    //   preintegration fallback for VIO-unhealthy. Today's degradation path
    //   is tags + chassis speeds.

    // Snapshot of the newest smoothed state; refreshed after every smoother
    // update. Same-thread only.
    const FusedState&     state() const;
    const FusionCounters& counters() const;

    // Full reinit: drops the smoother + accumulators, returns to the
    // uninitialized state. Does NOT bump counters().reinits (that counter
    // tracks automatic reinits only); counters are otherwise preserved.
    void reset();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace gw::fusion
