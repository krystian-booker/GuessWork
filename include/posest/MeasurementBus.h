#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <variant>

#include "posest/MeasurementTypes.h"

namespace posest {

using Measurement = std::variant<
    AprilTagObservation,
    VioMeasurement,
    ImuSample,
    WheelOdometrySample,
    RobotOdometrySample>;

class IMeasurementSink {
public:
    virtual ~IMeasurementSink() = default;

    virtual bool publish(AprilTagObservation observation) = 0;
    virtual bool publish(VioMeasurement measurement) = 0;
    virtual bool publish(ImuSample sample) = 0;
    virtual bool publish(WheelOdometrySample sample) = 0;
    virtual bool publish(RobotOdometrySample sample) = 0;
};

class MeasurementBus final : public IMeasurementSink {
public:
    explicit MeasurementBus(std::size_t capacity);

    MeasurementBus(const MeasurementBus&) = delete;
    MeasurementBus& operator=(const MeasurementBus&) = delete;

    bool publish(AprilTagObservation observation) override;
    bool publish(VioMeasurement measurement) override;
    bool publish(ImuSample sample) override;
    bool publish(WheelOdometrySample sample) override;
    bool publish(RobotOdometrySample sample) override;

    std::optional<Measurement> take();
    void shutdown();

    std::size_t size() const;
    std::uint64_t droppedNewestCount() const;

private:
    bool push(Measurement measurement);

    const std::size_t capacity_;
    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::deque<Measurement> queue_;
    std::uint64_t dropped_newest_{0};
    bool shut_{false};
};

}  // namespace posest
