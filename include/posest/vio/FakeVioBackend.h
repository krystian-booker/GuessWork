#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>

#include "posest/vio/IVioBackend.h"

namespace posest::vio {

// Deterministic synthetic-trajectory backend used by tests and by Linux
// CI builds where Kimera is not available. Each tryPushFrame:
//   - Records the teensy timestamp.
//   - Advances an internal pose by `frame_delta` (default: 0.05 m
//     forward, 0.01 rad yaw — enough to make BetweenFactor non-trivial).
//   - Emits a VioBackendOutput on the configured callback synchronously
//     (caller's thread). Synchronous emit keeps tests free of timing
//     races; the consumer's threading audit is unaffected because the
//     callback's contract is "any thread, anytime".
//
// IMU samples are accepted and counted but not used to alter the
// trajectory. Tests use that count to assert IMU plumbing wired up.
class FakeVioBackend final : public IVioBackend {
public:
    struct Config {
        Config();

        // Per-frame delta applied to the synthetic pose. Default
        // (set in the Config constructor) is 0.05 m forward + 0.01 rad
        // yaw — small enough to stay near the linearization point of
        // Pose3::between and large enough that floating-point noise
        // doesn't dominate.
        gtsam::Pose3 frame_delta;

        // Diagonal covariance baked into every emitted output. Tangent
        // order [rx, ry, rz, tx, ty, tz].
        double sigma_rotation_rad{0.005};
        double sigma_translation_m{0.01};

        // Frames whose 1-based index is <= this value report
        // tracking_ok=false. Tests use this to assert the consumer
        // drops bad-tracking frames.
        std::uint64_t tracking_ok_after_n_frames{0};

        // If non-zero, tryPushFrame returns false on this 1-based
        // frame index to simulate backpressure and exercise the
        // consumer's drop counter.
        std::uint64_t fail_push_on_frame{0};

        // Synthetic landmark count stamped onto every emitted output.
        // Default 20 sits above the consumer's default
        // landmark_count_floor (8) so existing tests don't see the
        // outputs_below_landmark_floor counter tick on every output.
        // Tests targeting the floor counter set this explicitly.
        std::int32_t landmark_count{20};
    };

    explicit FakeVioBackend(Config config = {});
    ~FakeVioBackend() override;

    void setOutputCallback(OutputCallback cb) override;
    bool tryPushFrame(std::uint64_t teensy_time_us,
                      const cv::Mat& image) override;
    bool tryPushImu(std::uint64_t teensy_time_us,
                    const Eigen::Vector3d& accel_mps2,
                    const Eigen::Vector3d& gyro_radps) override;
    void start() override;
    void stop() override;

    // Test introspection.
    std::uint64_t imuPushCount() const { return imu_push_count_.load(); }
    std::uint64_t framePushCount() const { return frame_push_count_.load(); }
    // teensy_time_us of the most recent IMU push (the value the
    // consumer handed in, not the frame's). 0 before any push. Used
    // by the wire-level timestamp regression test.
    std::uint64_t lastImuTeensyTimeUs() const {
        return last_imu_teensy_time_us_.load();
    }

private:
    Config config_;
    OutputCallback callback_;
    std::mutex pose_mu_;
    gtsam::Pose3 current_pose_{};
    std::uint64_t frame_index_{0};
    std::atomic<std::uint64_t> imu_push_count_{0};
    std::atomic<std::uint64_t> frame_push_count_{0};
    std::atomic<std::uint64_t> last_imu_teensy_time_us_{0};
    std::atomic<bool> running_{false};
};

}  // namespace posest::vio
