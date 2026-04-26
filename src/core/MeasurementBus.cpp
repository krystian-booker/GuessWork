#include "posest/MeasurementBus.h"

#include <stdexcept>
#include <utility>

namespace posest {

MeasurementBus::MeasurementBus(std::size_t capacity) : capacity_(capacity) {
    if (capacity_ == 0) {
        throw std::invalid_argument("MeasurementBus capacity must be greater than zero");
    }
}

bool MeasurementBus::publish(AprilTagObservation observation) {
    return push(std::move(observation));
}

bool MeasurementBus::publish(VioMeasurement measurement) {
    return push(std::move(measurement));
}

bool MeasurementBus::publish(ImuSample sample) {
    return push(std::move(sample));
}

bool MeasurementBus::publish(ChassisSpeedsSample sample) {
    return push(std::move(sample));
}

bool MeasurementBus::publish(CameraTriggerEvent event) {
    return push(std::move(event));
}

bool MeasurementBus::push(Measurement measurement) {
    {
        std::lock_guard<std::mutex> g(mu_);
        if (shut_) {
            return false;
        }
        if (queue_.size() >= capacity_) {
            ++dropped_newest_;
            return false;
        }
        queue_.push_back(std::move(measurement));
    }
    cv_.notify_one();
    return true;
}

std::optional<Measurement> MeasurementBus::take() {
    std::unique_lock<std::mutex> g(mu_);
    cv_.wait(g, [this] { return !queue_.empty() || shut_; });
    if (queue_.empty()) {
        return std::nullopt;
    }
    Measurement out = std::move(queue_.front());
    queue_.pop_front();
    return out;
}

void MeasurementBus::shutdown() {
    {
        std::lock_guard<std::mutex> g(mu_);
        shut_ = true;
    }
    cv_.notify_all();
}

std::size_t MeasurementBus::size() const {
    std::lock_guard<std::mutex> g(mu_);
    return queue_.size();
}

std::uint64_t MeasurementBus::droppedNewestCount() const {
    std::lock_guard<std::mutex> g(mu_);
    return dropped_newest_;
}

}  // namespace posest
