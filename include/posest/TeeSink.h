#pragma once

#include <utility>
#include <vector>

#include "posest/MeasurementBus.h"

namespace posest {

// Per-overload publisher-side fan-out for IMeasurementSink.
//
// The MeasurementBus is contractually single-consumer: only one thread is
// expected to call take(). That contract makes adding a second consumer
// (e.g. KimeraVioConsumer alongside FusionService) impossible without
// either splitting the queue or refactoring the bus. TeeSink takes the
// other route — it is itself an IMeasurementSink, configured per type
// with a list of downstream sinks. Publishers (TeensyService) keep their
// existing single IMeasurementSink& reference and send each measurement
// once; TeeSink forwards it to N independent buses.
//
// Routing is per-overload because the VIO bus only needs IMU samples —
// fanning out chassis/AprilTag/etc. to it would just stuff a queue that
// nothing drains. The default-constructed TeeSink routes nothing; call
// addRoute<T>(sink) for each (type, destination) pair.
//
// publish() returns true only if every targeted sink accepted the
// measurement, so existing drop-counters in TeensyService stay accurate.
class TeeSink final : public IMeasurementSink {
public:
    TeeSink() = default;

    TeeSink(const TeeSink&) = delete;
    TeeSink& operator=(const TeeSink&) = delete;

    // Add `sink` as a destination for measurements of type T. Order of
    // calls determines order of delivery (first added → first called).
    // Sinks must outlive the TeeSink.
    template <class T>
    void addRoute(IMeasurementSink* sink);

    bool publish(AprilTagObservation observation) override;
    bool publish(VioMeasurement measurement) override;
    bool publish(ImuSample sample) override;
    bool publish(ChassisSpeedsSample sample) override;
    bool publish(CameraTriggerEvent event) override;
    bool publish(ToFSample sample) override;

private:
    template <class T>
    bool fanOut(const std::vector<IMeasurementSink*>& sinks, T value);

    std::vector<IMeasurementSink*> apriltag_sinks_;
    std::vector<IMeasurementSink*> vio_sinks_;
    std::vector<IMeasurementSink*> imu_sinks_;
    std::vector<IMeasurementSink*> chassis_sinks_;
    std::vector<IMeasurementSink*> trigger_sinks_;
    std::vector<IMeasurementSink*> tof_sinks_;
};

template <>
inline void TeeSink::addRoute<AprilTagObservation>(IMeasurementSink* sink) {
    apriltag_sinks_.push_back(sink);
}
template <>
inline void TeeSink::addRoute<VioMeasurement>(IMeasurementSink* sink) {
    vio_sinks_.push_back(sink);
}
template <>
inline void TeeSink::addRoute<ImuSample>(IMeasurementSink* sink) {
    imu_sinks_.push_back(sink);
}
template <>
inline void TeeSink::addRoute<ChassisSpeedsSample>(IMeasurementSink* sink) {
    chassis_sinks_.push_back(sink);
}
template <>
inline void TeeSink::addRoute<CameraTriggerEvent>(IMeasurementSink* sink) {
    trigger_sinks_.push_back(sink);
}
template <>
inline void TeeSink::addRoute<ToFSample>(IMeasurementSink* sink) {
    tof_sinks_.push_back(sink);
}

template <class T>
inline bool TeeSink::fanOut(const std::vector<IMeasurementSink*>& sinks,
                            T value) {
    if (sinks.empty()) {
        // No subscribers: treat as accepted. The publisher has no useful
        // way to react to "nobody is listening" and we don't want such
        // routing gaps to count as drops.
        return true;
    }
    bool all_ok = true;
    // Move into the last sink to avoid one redundant copy. Each earlier
    // sink takes a copy; the bus's publish() consumes by value anyway.
    for (std::size_t i = 0; i + 1 < sinks.size(); ++i) {
        all_ok = sinks[i]->publish(value) && all_ok;
    }
    all_ok = sinks.back()->publish(std::move(value)) && all_ok;
    return all_ok;
}

inline bool TeeSink::publish(AprilTagObservation observation) {
    return fanOut(apriltag_sinks_, std::move(observation));
}
inline bool TeeSink::publish(VioMeasurement measurement) {
    return fanOut(vio_sinks_, std::move(measurement));
}
inline bool TeeSink::publish(ImuSample sample) {
    return fanOut(imu_sinks_, std::move(sample));
}
inline bool TeeSink::publish(ChassisSpeedsSample sample) {
    return fanOut(chassis_sinks_, std::move(sample));
}
inline bool TeeSink::publish(CameraTriggerEvent event) {
    return fanOut(trigger_sinks_, std::move(event));
}
inline bool TeeSink::publish(ToFSample sample) {
    return fanOut(tof_sinks_, std::move(sample));
}

}  // namespace posest
