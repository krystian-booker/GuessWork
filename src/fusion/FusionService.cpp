#include "posest/fusion/FusionService.h"

#include <type_traits>
#include <utility>

namespace posest::fusion {

namespace {

Timestamp timestampOf(const Measurement& measurement) {
    return std::visit(
        [](const auto& value) {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, AprilTagObservation>) {
                return value.capture_time;
            } else {
                return value.timestamp;
            }
        },
        measurement);
}

}  // namespace

FusionService::FusionService(MeasurementBus& measurement_bus)
    : measurement_bus_(measurement_bus) {}

FusionService::~FusionService() {
    stop();
}

void FusionService::addOutputSink(std::shared_ptr<IFusionOutputSink> sink) {
    std::lock_guard<std::mutex> g(mu_);
    sinks_.push_back(std::move(sink));
}

void FusionService::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return;
    }
    worker_ = std::thread(&FusionService::runLoop, this);
}

void FusionService::stop() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) {
        return;
    }
    measurement_bus_.shutdown();
    if (worker_.joinable()) {
        worker_.join();
    }
}

FusionStats FusionService::stats() const {
    std::lock_guard<std::mutex> g(mu_);
    return stats_;
}

std::optional<FusedPoseEstimate> FusionService::latestEstimate() const {
    std::lock_guard<std::mutex> g(mu_);
    return latest_estimate_;
}

void FusionService::runLoop() {
    while (running_.load(std::memory_order_acquire)) {
        auto measurement = measurement_bus_.take();
        if (!measurement) {
            return;
        }
        process(*measurement);
    }
}

void FusionService::process(const Measurement& measurement) {
    const Timestamp measurement_time = timestampOf(measurement);
    if (!acceptTimestamp(measurement_time)) {
        return;
    }

    FusedPoseEstimate estimate;
    estimate.timestamp = measurement_time;

    // This is a deterministic placeholder for the GTSAM-owned state estimate.
    // It keeps the service boundary testable until real factors are integrated.
    std::visit(
        [&estimate](const auto& value) {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, WheelOdometrySample>) {
                estimate.field_to_robot = value.chassis_delta;
            } else if constexpr (std::is_same_v<T, RobotOdometrySample>) {
                estimate.field_to_robot = value.field_to_robot;
                estimate.status_flags = value.status_flags;
            } else if constexpr (std::is_same_v<T, AprilTagObservation>) {
                estimate.status_flags = value.detections.empty() ? 1u : 0u;
            } else if constexpr (std::is_same_v<T, VioMeasurement>) {
                estimate.status_flags = value.tracking_ok ? 0u : 2u;
            } else if constexpr (std::is_same_v<T, ImuSample>) {
                estimate.status_flags = value.status_flags;
            }
        },
        measurement);

    publishEstimate(estimate);
}

bool FusionService::acceptTimestamp(Timestamp timestamp) {
    std::lock_guard<std::mutex> g(mu_);
    if (stats_.last_measurement_time && timestamp < *stats_.last_measurement_time) {
        ++stats_.stale_measurements;
        return false;
    }
    stats_.last_measurement_time = timestamp;
    ++stats_.measurements_processed;
    return true;
}

void FusionService::publishEstimate(FusedPoseEstimate estimate) {
    std::vector<std::shared_ptr<IFusionOutputSink>> sinks;
    {
        std::lock_guard<std::mutex> g(mu_);
        latest_estimate_ = estimate;
        sinks = sinks_;
    }

    for (auto& sink : sinks) {
        sink->publish(estimate);
    }
}

}  // namespace posest::fusion
