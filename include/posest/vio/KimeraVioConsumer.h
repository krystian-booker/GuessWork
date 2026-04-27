#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "posest/ConsumerBase.h"
#include "posest/MeasurementBus.h"
#include "posest/MeasurementTypes.h"
#include "posest/Timestamp.h"
#include "posest/vio/AirborneCovariance.h"
#include "posest/vio/IVioBackend.h"
#include "posest/vio/KimeraVioConfig.h"

namespace posest::vio {

// Callable that maps a Teensy hardware-clock microsecond stamp onto a
// host steady_clock Timestamp. In production this is bound to
// TeensyService::timestampFromTeensyTime; in tests it's a 1:1 stub.
// The fallback Timestamp is returned when the time-sync filter has not
// yet established a sync — same contract as the underlying TeensyService
// method.
using TeensyTimeConverter =
    std::function<Timestamp(std::uint64_t teensy_time_us, Timestamp fallback)>;

struct KimeraVioStats {
    std::uint64_t frames_pushed{0};
    std::uint64_t frames_dropped_backpressure{0};
    std::uint64_t imu_pushed{0};
    std::uint64_t imu_dropped_backpressure{0};
    std::uint64_t imu_buffer_overflow{0};
    std::uint64_t outputs_received{0};
    std::uint64_t outputs_published{0};
    std::uint64_t outputs_skipped_first{0};
    std::uint64_t outputs_skipped_no_tracking{0};
    std::uint64_t outputs_inflated_airborne{0};
    std::uint64_t ground_distance_missing{0};
};

// Consumes Frame objects (via ConsumerBase) and ImuSample measurements
// (via a dedicated, single-consumer MeasurementBus instance), feeds
// them into an IVioBackend, and republishes the backend's pose deltas
// onto an output IMeasurementSink as VioMeasurement. FusionService
// then turns those into BetweenFactor<Pose3> on its existing pose
// chain — the consumer is the *only* place the Kimera world frame
// touches our system.
//
// Threads owned by this class:
//   A. ConsumerBase worker — drives process(Frame), pushes to backend.
//   B. IMU drainer — blocks on imu_input_bus_.take() and buffers samples.
// Plus the backend's output callback, which may run on Kimera's
// internal thread. See class-internal comments for the locking story.
class KimeraVioConsumer final : public ConsumerBase {
public:
    KimeraVioConsumer(std::string id,
                      MeasurementBus& imu_input_bus,
                      IMeasurementSink& output_sink,
                      TeensyTimeConverter time_converter,
                      std::unique_ptr<IVioBackend> backend,
                      KimeraVioConfig config = {});
    ~KimeraVioConsumer() override;

    KimeraVioConsumer(const KimeraVioConsumer&) = delete;
    KimeraVioConsumer& operator=(const KimeraVioConsumer&) = delete;

    void start() override;
    void stop() override;

    KimeraVioStats stats() const;

protected:
    void process(const Frame& frame) override;

private:
    void imuDrainerLoop();
    void onBackendOutput(const VioBackendOutput& out);

    // Drain `imu_buffer_` of samples whose teensy time ≤ frame_teensy_us
    // and push them into the backend in order. Called from thread A
    // under no lock — acquires `imu_buffer_mu_` internally.
    void drainImuUpTo(std::uint64_t frame_teensy_us);

    // Record the airborne state for `teensy_time_us` so the (potentially
    // late-arriving) backend callback can recover it. Trims the buffer
    // to `airborne_lookup_capacity_`.
    void recordAirborneState(std::uint64_t teensy_time_us,
                             AirborneState state);
    AirborneState lookupAirborneState(std::uint64_t teensy_time_us) const;

    KimeraVioConfig config_;
    MeasurementBus& imu_bus_;
    IMeasurementSink& output_sink_;
    TeensyTimeConverter time_converter_;
    std::unique_ptr<IVioBackend> backend_;

    // (A↔B) IMU staging buffer keyed by teensy_time_us. Insert order
    // is monotonic when the bus is operating normally, but we don't
    // enforce that — drainImuUpTo just compares timestamps.
    mutable std::mutex imu_buffer_mu_;
    std::deque<ImuSample> imu_buffer_;

    // Airborne state at frame-push time, keyed by teensy_time_us. Read
    // from the backend output callback (any thread); written from the
    // frame worker (thread A). Mutex is fine — both sides do trivial
    // work under it.
    mutable std::mutex airborne_lookup_mu_;
    std::deque<std::pair<std::uint64_t, AirborneState>> airborne_lookup_;

    // Owned by the backend callback; never observed elsewhere. No lock.
    std::optional<gtsam::Pose3> last_kimera_pose_;
    std::optional<Eigen::Matrix<double, 6, 6>> last_kimera_pose_cov_;

    AirborneTracker airborne_tracker_;

    std::thread imu_drainer_;
    std::atomic<bool> imu_drainer_running_{false};

    mutable std::mutex stats_mu_;
    KimeraVioStats stats_;
};

}  // namespace posest::vio
