#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "posest/MeasurementBus.h"
#include "posest/fusion/IFusionOutputSink.h"

namespace posest::fusion {

struct FusionStats {
    std::uint64_t measurements_processed{0};
    std::uint64_t stale_measurements{0};
    std::optional<Timestamp> last_measurement_time;
};

class FusionService final {
public:
    explicit FusionService(MeasurementBus& measurement_bus);
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
    bool acceptTimestamp(Timestamp timestamp);
    void publishEstimate(FusedPoseEstimate estimate);

    MeasurementBus& measurement_bus_;
    mutable std::mutex mu_;
    std::vector<std::shared_ptr<IFusionOutputSink>> sinks_;
    FusionStats stats_;
    std::optional<FusedPoseEstimate> latest_estimate_;
    std::thread worker_;
    std::atomic<bool> running_{false};
};

}  // namespace posest::fusion
