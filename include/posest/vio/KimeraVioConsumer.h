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
#include "posest/runtime/IVisionPipeline.h"
#include "posest/vio/AirborneCovariance.h"
#include "posest/vio/IVioBackend.h"
#include "posest/vio/KimeraVioConfig.h"
#include "posest/vio/KimeraVioStats.h"

namespace posest::vio {

// Callable that maps a Teensy hardware-clock microsecond stamp onto a
// host steady_clock Timestamp. In production this is bound to
// TeensyService::timestampFromTeensyTime; in tests it's a 1:1 stub.
// The fallback Timestamp is returned when the time-sync filter has not
// yet established a sync — same contract as the underlying TeensyService
// method.
using TeensyTimeConverter =
    std::function<Timestamp(std::uint64_t teensy_time_us, Timestamp fallback)>;

// KimeraVioStats now lives in posest/vio/KimeraVioStats.h so that
// posest/pipelines/PipelineStats.h can name it as a variant alternative
// without transitively pulling in the consumer's GTSAM-heavy headers.

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
class KimeraVioConsumer final : public ConsumerBase,
                                public runtime::IVisionPipeline {
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

    // IVisionPipeline. The consumer plugs into the per-camera pipeline
    // factory slot keyed by `type() == "vio"`; see ProductionFactories.
    const std::string& type() const override { return type_; }

    // IVisionPipeline polymorphic telemetry hook. Wraps stats() in a
    // PipelineStatsValue so DaemonController::refreshHealth can fan
    // KimeraVioStats onto DaemonHealth alongside AprilTagPipelineStats
    // without dynamic_cast'ing per concrete pipeline type. Stamps
    // pipeline_id from ConsumerBase::id() so the JSON consumer can
    // disambiguate when multiple VIO pipelines are wired (today only one).
    pipelines::PipelineStatsValue pipelineStats() const override;

    // Stage a new config for the next process() iteration. Safe to call
    // from any thread; the swap happens at the top of process() before
    // any backend interaction. Live fields take effect immediately;
    // structural fields (param_dir, imu_buffer_capacity, camera_id) are
    // reverted to the running config and bump
    // KimeraVioStats::config_reloads_structural_skipped — they require
    // a backend restart to apply. Mirrors FusionService::applyConfig.
    void applyConfig(KimeraVioConfig new_config);

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

    // Drain any staged config (set by applyConfig) and merge live fields
    // into config_. Called from the frame worker (thread A) before
    // touching the backend or the airborne tracker. Returns true if a
    // config was applied (touched live fields), false if no config was
    // pending. Independent of the structural-skipped counter.
    bool drainPendingConfig();

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

    // Staged config from applyConfig. Producer is the web thread (any
    // thread); consumer is the frame worker (thread A) at the top of
    // process(). The mutex guards only the optional swap — the
    // KimeraVioConfig payload is copied out under the lock.
    mutable std::mutex pending_config_mu_;
    std::optional<KimeraVioConfig> pending_config_;

    std::thread imu_drainer_;
    std::atomic<bool> imu_drainer_running_{false};

    mutable std::mutex stats_mu_;
    KimeraVioStats stats_;
    // Wall-clock of the most recent successfully published VioMeasurement.
    // Stamped from steady_clock at the bottom of onBackendOutput; consumed
    // by stats() to derive KimeraVioStats::last_output_age_ms. Held here
    // rather than on KimeraVioStats so that struct stays chrono-free for
    // PipelineStats.h consumption.
    std::optional<std::chrono::steady_clock::time_point> last_output_at_;

    // Pipeline type tag, returned by reference from type(). Constant
    // for the lifetime of the consumer.
    std::string type_{"vio"};
};

}  // namespace posest::vio
