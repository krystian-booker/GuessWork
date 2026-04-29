#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include <Eigen/Core>
#include <gtsam/geometry/Pose3.h>
#include <opencv2/core/mat.hpp>

namespace posest::vio {

// Output emitted by a VIO backend per processed frame. Kimera emits this
// on its own internal backend thread; the FakeVioBackend emits inline
// from the test driver. Either way, the consumer must treat the callback
// as potentially racy w.r.t. its own threads.
struct VioBackendOutput {
    // Frame timestamp in the Teensy hardware-clock domain. Carried
    // through end-to-end so the consumer can join against
    // Frame::ground_distance_m and convert to host steady_clock at
    // publish time.
    std::uint64_t teensy_time_us{0};

    // Absolute pose in the backend's world frame. Kimera's W frame is
    // bootstrap-defined and drifts independently of FusionService's
    // world — fine, we only consume relative motion downstream.
    gtsam::Pose3 world_T_body;

    // 6×6 marginal covariance over Pose3 tangent. **Convention check
    // load-bearing here**: gtsam Pose3 tangent is [rx, ry, rz, tx, ty, tz].
    // Kimera's BackendOutput::state_covariance_lkf_ is also documented
    // as [rot, pos, ...] for the leading 6 rows/cols, so a direct copy
    // of the upper-left 6×6 block matches gtsam's convention. Verify
    // with the YAML round-trip test before enabling on hardware.
    Eigen::Matrix<double, 6, 6> pose_covariance{
        Eigen::Matrix<double, 6, 6>::Zero()};

    // Set when the backend's tracker reports a valid solution. The
    // consumer drops measurements with tracking_ok=false rather than
    // publishing them with degraded covariance — FusionService already
    // treats this as a no-op (FusionService.cpp:696) but doing the
    // gate locally avoids enqueueing dead measurements.
    bool tracking_ok{false};

    // Number of landmarks Kimera held in its smoother for this output.
    // Phase 2 telemetry signal: see KimeraVioStats::landmark_count_avg
    // and outputs_below_landmark_floor for the operator-facing
    // observability surface. KimeraBackend populates this from
    // BackendOutput::landmark_count_; FakeVioBackend leaves it at 0
    // unless a test wires it explicitly.
    std::int32_t landmark_count{0};

    // Free-form status string for telemetry; mirrors
    // VioMeasurement::backend_status. Optional.
    std::string backend_status;
};

// Backend abstraction. The real implementation (KimeraBackend) lives in
// the Apple-only posest_vio_kimera library; FakeVioBackend (always
// built) implements this interface deterministically for tests so the
// KimeraVioConsumer integration test runs on Linux CI without a Kimera
// install.
//
// Thread model the consumer assumes:
//   - tryPushFrame / tryPushImu are called from the consumer's frame
//     worker thread (process()) and the IMU drainer thread respectively.
//     Both must be non-blocking: a slow Kimera must not stall the
//     producer chain. Returning `false` is the way to signal backpressure
//     so the consumer can record a drop.
//   - The output callback may be invoked on any thread the backend
//     chooses. It must be set before start() is called.
class IVioBackend {
public:
    using OutputCallback = std::function<void(const VioBackendOutput&)>;

    virtual ~IVioBackend() = default;

    virtual void setOutputCallback(OutputCallback cb) = 0;

    // Hand a frame to the backend. The backend takes ownership of the
    // image data (callers should already hold a shared reference if
    // they need it after this call). Returns false if the backend's
    // input queue is full — the consumer increments a drop counter
    // and moves on; do NOT block.
    virtual bool tryPushFrame(std::uint64_t teensy_time_us,
                              const cv::Mat& image) = 0;

    // Hand an IMU sample to the backend. Same non-blocking contract as
    // tryPushFrame.
    virtual bool tryPushImu(std::uint64_t teensy_time_us,
                            const Eigen::Vector3d& accel_mps2,
                            const Eigen::Vector3d& gyro_radps) = 0;

    virtual void start() = 0;
    virtual void stop() = 0;
};

}  // namespace posest::vio
