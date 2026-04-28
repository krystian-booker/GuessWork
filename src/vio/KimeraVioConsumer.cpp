#include "posest/vio/KimeraVioConsumer.h"

#include <algorithm>
#include <chrono>
#include <utility>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include "posest/MeasurementBus.h"

namespace posest::vio {

namespace {

Pose3d toPose3d(const gtsam::Pose3& p) {
    // Mirrors FusionService::toGtsamPose (FusionService.cpp:40): RzRyRx
    // composed from (roll, pitch, yaw) constructs the rotation, so
    // gtsam::Rot3::rpy() recovers the same triple.
    const auto rpy = p.rotation().rpy();
    Pose3d out;
    out.rotation_rpy_rad = {rpy(0), rpy(1), rpy(2)};
    out.translation_m = {p.x(), p.y(), p.z()};
    return out;
}

std::array<double, 36> toRowMajor(const Eigen::Matrix<double, 6, 6>& m) {
    std::array<double, 36> out{};
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            out[static_cast<std::size_t>(r * 6 + c)] = m(r, c);
        }
    }
    return out;
}

}  // namespace

KimeraVioConsumer::KimeraVioConsumer(
    std::string id,
    MeasurementBus& imu_input_bus,
    IMeasurementSink& output_sink,
    TeensyTimeConverter time_converter,
    std::unique_ptr<IVioBackend> backend,
    KimeraVioConfig config)
    : ConsumerBase(std::move(id)),
      config_(std::move(config)),
      imu_bus_(imu_input_bus),
      output_sink_(output_sink),
      time_converter_(std::move(time_converter)),
      backend_(std::move(backend)),
      airborne_tracker_(config_.airborne) {
    backend_->setOutputCallback(
        [this](const VioBackendOutput& out) { onBackendOutput(out); });
}

KimeraVioConsumer::~KimeraVioConsumer() {
    // Leaf destructor: the ConsumerBase destructor will call stop() but
    // its check expects the worker to already be stopped, so we
    // explicitly stop here while our vtable is still intact.
    stop();
}

void KimeraVioConsumer::start() {
    backend_->start();

    bool expected = false;
    if (imu_drainer_running_.compare_exchange_strong(expected, true)) {
        imu_drainer_ = std::thread(&KimeraVioConsumer::imuDrainerLoop, this);
    }

    ConsumerBase::start();
}

void KimeraVioConsumer::stop() {
    // Stop the frame worker first so process() can no longer call into
    // backend_->tryPushFrame.
    ConsumerBase::stop();

    // Then unblock the IMU drainer. shutdown() of the bus is permanent,
    // matching the start-once-stop-once contract on ConsumerBase. The
    // bus is dedicated to this consumer (the daemon constructs it for
    // the sole purpose of feeding the VIO module) so shutting it down
    // here is the right ownership call.
    if (imu_drainer_running_.exchange(false)) {
        imu_bus_.shutdown();
        if (imu_drainer_.joinable()) {
            imu_drainer_.join();
        }
    }

    // Stop the backend last so any in-flight callback completes against
    // an output_sink_ that is still alive (lifetime is the daemon's
    // responsibility — see class comment).
    if (backend_) {
        backend_->stop();
    }
}

KimeraVioStats KimeraVioConsumer::stats() const {
    std::lock_guard<std::mutex> g(stats_mu_);
    KimeraVioStats s = stats_;
    s.ground_distance_missing = airborne_tracker_.missingCount();
    return s;
}

void KimeraVioConsumer::applyConfig(KimeraVioConfig new_config) {
    // Stage-and-swap mirroring FusionService::applyConfig. The web
    // thread returns immediately; the frame worker picks up the new
    // config at the top of its next process() iteration. We do *not*
    // touch config_ here — that would race with the frame worker, which
    // reads it without locking on every frame.
    std::lock_guard<std::mutex> g(pending_config_mu_);
    pending_config_ = std::move(new_config);
}

bool KimeraVioConsumer::drainPendingConfig() {
    std::optional<KimeraVioConfig> staged;
    {
        std::lock_guard<std::mutex> g(pending_config_mu_);
        staged.swap(pending_config_);
    }
    if (!staged.has_value()) {
        return false;
    }

    // Detect structural drift (fields the live path can't honor) and
    // count + revert before applying. param_dir is read by the backend
    // at start(); imu_buffer_capacity sizes the deque consulted by the
    // IMU drainer thread (B); camera_id is stamped on every emitted
    // measurement and changing it would scramble FusionService's
    // routing. Each requires a backend restart and is silently rolled
    // back here for parity with FusionService's structural reversion
    // (FusionService.cpp:762-806).
    const bool structural_diff =
        staged->param_dir != config_.param_dir ||
        staged->imu_buffer_capacity != config_.imu_buffer_capacity ||
        staged->camera_id != config_.camera_id;
    if (structural_diff) {
        staged->param_dir = config_.param_dir;
        staged->imu_buffer_capacity = config_.imu_buffer_capacity;
        staged->camera_id = config_.camera_id;
        std::lock_guard<std::mutex> g(stats_mu_);
        ++stats_.config_reloads_structural_skipped;
    }

    // Live fields. airborne_tracker_ is touched only from this thread
    // (process()) so its setter is safe to call without locking.
    config_ = std::move(*staged);
    airborne_tracker_.setThresholds(config_.airborne);

    {
        std::lock_guard<std::mutex> g(stats_mu_);
        ++stats_.config_reloads_applied;
    }
    return true;
}

void KimeraVioConsumer::process(const Frame& frame) {
    // Drain any web-saved config before touching anything that depends
    // on it. The swap is at the top of the loop so the frame and its
    // recorded airborne state are evaluated against a single config
    // version.
    drainPendingConfig();

    // Update the airborne state machine on the same thread that drives
    // frames into Kimera, so the (teensy_time → state) snapshot we
    // record below corresponds exactly to the frame the backend will
    // emit a delta for.
    const auto state = airborne_tracker_.update(
        frame.ground_distance_m, std::chrono::steady_clock::now());

    // Frames without a Teensy timestamp can't be aligned to IMU samples
    // (which only carry teensy_time_us in their wire form, see
    // ImuPayload at Protocol.h:76). Drop with a counter — the producer
    // chain should always pair frames with trigger pulses on this
    // platform, so this branch is a "you broke the wiring" alarm.
    if (!frame.teensy_time_us.has_value()) {
        std::lock_guard<std::mutex> g(stats_mu_);
        ++stats_.frames_dropped_backpressure;
        return;
    }
    const std::uint64_t frame_t_us = *frame.teensy_time_us;

    recordAirborneState(frame_t_us, state);
    drainImuUpTo(frame_t_us);

    const bool ok = backend_->tryPushFrame(frame_t_us, frame.image);
    std::lock_guard<std::mutex> g(stats_mu_);
    if (ok) {
        ++stats_.frames_pushed;
    } else {
        ++stats_.frames_dropped_backpressure;
    }
}

void KimeraVioConsumer::imuDrainerLoop() {
    while (imu_drainer_running_.load(std::memory_order_acquire)) {
        auto m = imu_bus_.take();
        if (!m.has_value()) {
            // shutdown() was called.
            return;
        }
        // The TeeSink wired into TeensyService should only forward
        // ImuSample to this bus, but the bus itself accepts the full
        // variant — defend with a visit and ignore non-IMU types.
        if (!std::holds_alternative<ImuSample>(*m)) {
            continue;
        }
        const auto& sample = std::get<ImuSample>(*m);

        std::lock_guard<std::mutex> g(imu_buffer_mu_);
        if (imu_buffer_.size() >= config_.imu_buffer_capacity) {
            imu_buffer_.pop_front();
            std::lock_guard<std::mutex> sg(stats_mu_);
            ++stats_.imu_buffer_overflow;
        }
        imu_buffer_.push_back(sample);
    }
}

void KimeraVioConsumer::drainImuUpTo(std::uint64_t frame_teensy_us) {
    // Pop samples whose timestamp <= frame_teensy_us under the buffer
    // mutex, then push them to the backend without holding the mutex
    // (Kimera's tryPushImu may take its own internal lock and we must
    // not invert that ordering).
    std::deque<ImuSample> to_push;
    {
        std::lock_guard<std::mutex> g(imu_buffer_mu_);
        while (!imu_buffer_.empty()) {
            // ImuSample carries a host steady_clock timestamp; we need
            // to compare on the Teensy domain. The TimeSyncFilter is
            // monotonic on average, so converting the steady_clock
            // value back is a round-trip we want to avoid. The wire
            // ImuPayload has teensy_time_us but TeensyService strips
            // it before publishing onto the bus (MeasurementTypes.h:72
            // does not carry it). We therefore convert frame_teensy_us
            // to host time via the TeensyTimeConverter and compare in
            // the steady_clock domain.
            //
            // Note: this is slightly weaker than wire-level matching
            // because the time-sync filter has small skew. For VIO
            // preintegration that's fine — Kimera tolerates IMU
            // samples a few milliseconds out of order around the frame
            // boundary.
            const Timestamp frame_host_time = time_converter_(
                frame_teensy_us, std::chrono::steady_clock::now());
            if (imu_buffer_.front().timestamp > frame_host_time) {
                break;
            }
            to_push.push_back(std::move(imu_buffer_.front()));
            imu_buffer_.pop_front();
        }
    }

    for (const auto& s : to_push) {
        const Eigen::Vector3d accel(
            s.accel_mps2.x, s.accel_mps2.y, s.accel_mps2.z);
        const Eigen::Vector3d gyro(
            s.gyro_radps.x, s.gyro_radps.y, s.gyro_radps.z);
        // We use frame_teensy_us as the IMU timestamp here because the
        // host-side ImuSample dropped its teensy_time_us. The backend
        // receives a stream of IMU samples bracketing each frame; for
        // the FakeVioBackend this is irrelevant and for KimeraBackend
        // a follow-up wire-level fix (carry teensy_time_us on
        // ImuSample) will replace this approximation. TODO(IMU-T).
        const bool ok = backend_->tryPushImu(frame_teensy_us, accel, gyro);
        std::lock_guard<std::mutex> g(stats_mu_);
        if (ok) {
            ++stats_.imu_pushed;
        } else {
            ++stats_.imu_dropped_backpressure;
        }
    }
}

void KimeraVioConsumer::recordAirborneState(std::uint64_t teensy_time_us,
                                            AirborneState state) {
    std::lock_guard<std::mutex> g(airborne_lookup_mu_);
    airborne_lookup_.emplace_back(teensy_time_us, state);
    while (airborne_lookup_.size() > config_.airborne_lookup_capacity) {
        airborne_lookup_.pop_front();
    }
}

AirborneState KimeraVioConsumer::lookupAirborneState(
    std::uint64_t teensy_time_us) const {
    std::lock_guard<std::mutex> g(airborne_lookup_mu_);
    for (const auto& [t, s] : airborne_lookup_) {
        if (t == teensy_time_us) {
            return s;
        }
    }
    // Fallback: if the entry was evicted (consumer fell badly behind)
    // assume grounded. The alternative — assume airborne — would
    // permanently mute VIO under heavy backlog and is the worse
    // failure mode.
    return AirborneState::kGrounded;
}

void KimeraVioConsumer::onBackendOutput(const VioBackendOutput& out) {
    {
        std::lock_guard<std::mutex> g(stats_mu_);
        ++stats_.outputs_received;
    }

    if (!out.tracking_ok) {
        std::lock_guard<std::mutex> g(stats_mu_);
        ++stats_.outputs_skipped_no_tracking;
        return;
    }

    if (!last_kimera_pose_.has_value()) {
        last_kimera_pose_ = out.world_T_body;
        last_kimera_pose_cov_ = out.pose_covariance;
        std::lock_guard<std::mutex> g(stats_mu_);
        ++stats_.outputs_skipped_first;
        return;
    }

    const gtsam::Pose3 relative =
        last_kimera_pose_->between(out.world_T_body);

    // Absolute → relative covariance. See the architecture plan for
    // why kScaled is the default; kDelta is more correct but brittle.
    Eigen::Matrix<double, 6, 6> rel_cov;
    switch (config_.covariance_strategy) {
        case CovarianceStrategy::kAbsolute:
            rel_cov = out.pose_covariance;
            break;
        case CovarianceStrategy::kDelta: {
            rel_cov = out.pose_covariance - *last_kimera_pose_cov_;
            // Floor the diagonal so the matrix stays PSD even when
            // Kimera reports a temporary drop in absolute uncertainty
            // (e.g. after a relinearization).
            for (int i = 0; i < 6; ++i) {
                if (rel_cov(i, i) < 1.0e-9) rel_cov(i, i) = 1.0e-9;
            }
            break;
        }
        case CovarianceStrategy::kScaled:
        default:
            rel_cov =
                config_.covariance_scale_alpha * out.pose_covariance;
            break;
    }

    auto cov_array = toRowMajor(rel_cov);

    const AirborneState state = lookupAirborneState(out.teensy_time_us);
    if (state != AirborneState::kGrounded) {
        cov_array =
            inflate(cov_array, state, config_.inflation_factor,
                    config_.inflation_cap);
        std::lock_guard<std::mutex> g(stats_mu_);
        ++stats_.outputs_inflated_airborne;
    }

    VioMeasurement m;
    m.camera_id = config_.camera_id;
    m.timestamp = time_converter_(
        out.teensy_time_us, std::chrono::steady_clock::now());
    m.relative_motion = toPose3d(relative);
    m.covariance = cov_array;
    m.tracking_ok = true;
    m.backend_status = out.backend_status;

    output_sink_.publish(std::move(m));

    last_kimera_pose_ = out.world_T_body;
    last_kimera_pose_cov_ = out.pose_covariance;

    std::lock_guard<std::mutex> g(stats_mu_);
    ++stats_.outputs_published;
}

}  // namespace posest::vio
