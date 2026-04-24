#pragma once

#include <atomic>
#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
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

struct FusionConfig {
    std::unordered_map<int, Pose3d> field_to_tags;
    std::unordered_map<std::string, Pose3d> camera_to_robot;
    std::array<double, 6> wheel_sigmas{0.05, 0.05, 0.05, 0.02, 0.02, 0.02};
    std::array<double, 6> vision_sigmas{0.15, 0.15, 0.12, 0.10, 0.10, 0.20};
    std::array<double, 6> origin_prior_sigmas{10.0, 10.0, 3.14, 10.0, 10.0, 10.0};
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
