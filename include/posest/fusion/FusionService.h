#pragma once

#include <atomic>
#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "posest/MeasurementBus.h"
#include "posest/fusion/IFusionOutputSink.h"
#include "posest/runtime/RuntimeConfig.h"

namespace posest::fusion {

struct FusionStats {
    std::uint64_t measurements_processed{0};
    std::uint64_t stale_measurements{0};
    std::optional<Timestamp> last_measurement_time;
};

inline constexpr std::uint32_t kFusionStatusInitializing = 1u << 0u;
inline constexpr std::uint32_t kFusionStatusVisionUnavailable = 1u << 1u;
inline constexpr std::uint32_t kFusionStatusMarginalUnavailable = 1u << 2u;
inline constexpr std::uint32_t kFusionStatusOptimizerError = 1u << 3u;
inline constexpr std::uint32_t kFusionStatusDegradedInput = 1u << 4u;
inline constexpr std::uint32_t kFusionStatusShockInflated = 1u << 5u;
inline constexpr std::uint32_t kFusionStatusChassisGap = 1u << 6u;

struct FusionConfig {
    // Process noise on the per-Δt BetweenFactor built from integrated
    // ChassisSpeeds. Order matches gtsam Pose3 tangent: [rx, ry, rz, tx, ty, tz].
    std::array<double, 6> chassis_sigmas{0.05, 0.05, 0.05, 0.02, 0.02, 0.02};
    std::array<double, 6> origin_prior_sigmas{10.0, 10.0, 3.14, 10.0, 10.0, 10.0};

    // Fires the shock branch when |a - g_local| over imu_window_seconds
    // exceeds this. 50 m/s^2 (~5g) is past normal driving accel for an FRC
    // chassis but well below collision/wheel-slip transients.
    double shock_threshold_mps2{50.0};
    // Fires the free-fall branch when |a| drops below this. The robot in
    // air-time over a bump or ramp reads ~0; normal driving never does.
    double freefall_threshold_mps2{3.0};
    // Multiplier applied to every chassis_sigma when either branch fires.
    double shock_inflation_factor{100.0};
    // IMU samples older than this (relative to the current chassis sample)
    // are pruned from the shock-detection window.
    double imu_window_seconds{0.05};
    // ChassisSpeeds samples spaced more than this apart skip the integration
    // step entirely (kFusionStatusChassisGap is set).
    double max_chassis_dt_seconds{0.5};
    // Local gravity vector in the IMU's robot frame. Subtracted before the
    // shock-magnitude test. Override for tilted IMU mounts.
    Vec3 gravity_local_mps2{0.0, 0.0, 9.80665};
};

FusionConfig buildFusionConfig(const runtime::RuntimeConfig& runtime_config);

struct FusionBackend;

class FusionService final {
public:
    explicit FusionService(MeasurementBus& measurement_bus, FusionConfig config = {});
    ~FusionService();

    FusionService(const FusionService&) = delete;
    FusionService& operator=(const FusionService&) = delete;

    void addOutputSink(std::shared_ptr<IFusionOutputSink> sink);
    void start();
    void stop();

    FusionStats stats() const;
    std::optional<FusedPoseEstimate> latestEstimate() const;

private:
    void runLoop();
    void process(const Measurement& measurement);
    bool isSupportedMeasurement(const Measurement& measurement) const;
    bool acceptTimestamp(Timestamp timestamp);
    void publishEstimate(FusedPoseEstimate estimate);

    MeasurementBus& measurement_bus_;
    std::unique_ptr<FusionBackend> backend_;
    mutable std::mutex mu_;
    std::vector<std::shared_ptr<IFusionOutputSink>> sinks_;
    FusionStats stats_;
    std::optional<FusedPoseEstimate> latest_estimate_;
    std::thread worker_;
    std::atomic<bool> running_{false};
};

}  // namespace posest::fusion
