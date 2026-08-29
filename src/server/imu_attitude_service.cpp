#include "server/imu_attitude_service.hpp"

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <thread>

#include "server/sync_controller_manager.hpp"

namespace gw::server {

struct ImuAttitudeService::Impl {
    SyncControllerManager& controller;

    gw::MeasurementBus<gw::ImuSample>::SubscriberHandle sub;
    std::thread       worker;
    std::atomic<bool> stop{false};

    mutable std::mutex mu;
    gw::AttitudeFilter filter;
    gw::ImuSample      last{};
    bool               seen = false;
    std::chrono::steady_clock::time_point       last_at{};
    std::deque<std::chrono::steady_clock::time_point> window;  // ~1 s

    explicit Impl(SyncControllerManager& t) : controller(t) {}

    void run() {
        gw::ImuSample s;
        while (controller.imu_bus().wait_pop(sub, s)) {
            if (stop.load(std::memory_order_acquire)) break;
            const auto now = std::chrono::steady_clock::now();
            std::lock_guard lk(mu);
            filter.feed(s);
            last    = s;
            seen    = true;
            last_at = now;
            window.push_back(now);
            while (!window.empty() &&
                   now - window.front() > std::chrono::seconds(1)) {
                window.pop_front();
            }
        }
    }
};

ImuAttitudeService::ImuAttitudeService(SyncControllerManager& controller)
    : impl_(std::make_unique<Impl>(controller)) {
    impl_->sub    = controller.imu_bus().subscribe(4096);
    impl_->worker = std::thread([this] { impl_->run(); });
}

ImuAttitudeService::~ImuAttitudeService() {
    impl_->stop.store(true, std::memory_order_release);
    impl_->controller.imu_bus().unsubscribe(impl_->sub);  // wakes wait_pop
    if (impl_->worker.joinable()) impl_->worker.join();
}

ImuAttitudeService::Status ImuAttitudeService::status() const {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard lk(impl_->mu);
    Status st;
    st.attitude = impl_->filter.snapshot();
    for (int i = 0; i < 3; ++i) {
        st.accel[i] = impl_->last.accel[i];
        st.gyro[i]  = impl_->last.gyro[i];
    }
    if (impl_->seen) {
        st.last_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                             now - impl_->last_at)
                             .count();
    }
    // Count only window entries still inside 1 s (idle decays to 0).
    std::size_t in_window = 0;
    for (const auto& t : impl_->window) {
        if (now - t <= std::chrono::seconds(1)) ++in_window;
    }
    st.rate_hz = static_cast<double>(in_window);
    return st;
}

void ImuAttitudeService::zero_yaw() {
    std::lock_guard lk(impl_->mu);
    impl_->filter.zero_yaw();
}

void ImuAttitudeService::reset() {
    std::lock_guard lk(impl_->mu);
    impl_->filter.reset();
}

}  // namespace gw::server
