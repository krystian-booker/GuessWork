#include "posest/fusion/FusionService.h"

#include <chrono>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

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

double secondsBetween(Timestamp earlier, Timestamp later) {
    return std::chrono::duration<double>(later - earlier).count();
}

gtsam::Pose3 toGtsamPose(const Pose3d& pose) {
    return gtsam::Pose3(
        gtsam::Rot3::RzRyRx(
            pose.rotation_rpy_rad.x,
            pose.rotation_rpy_rad.y,
            pose.rotation_rpy_rad.z),
        gtsam::Point3(
            pose.translation_m.x,
            pose.translation_m.y,
            pose.translation_m.z));
}

Pose2d toPose2d(const gtsam::Pose3& pose) {
    return {
        pose.x(),
        pose.y(),
        pose.rotation().yaw(),
    };
}

gtsam::SharedNoiseModel diagonalNoise(const std::array<double, 6>& sigmas) {
    gtsam::Vector noise(6);
    for (int i = 0; i < 6; ++i) {
        noise(i) = sigmas[static_cast<std::size_t>(i)];
    }
    return gtsam::noiseModel::Diagonal::Sigmas(noise);
}

// Wraps `inner` in a Huber M-estimator. The wrapped factor downweights
// residuals beyond `huber_k` standardized sigmas without rejecting them
// outright — chosen for short-duration wheel-slip outliers where Cauchy
// would over-suppress legitimate hard accelerations.
gtsam::SharedNoiseModel robustHuberNoise(
    const gtsam::SharedNoiseModel& inner, double huber_k) {
    return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(huber_k),
        inner);
}

gtsam::Key poseKey(std::uint64_t index) {
    return gtsam::Symbol('x', index).key();
}

// Velocity variable key, used only when IMU preintegration is enabled.
// 'v' deliberately distinct from 'x' so the existing pose-only graph never
// collides with a velocity key after the feature flag flips on mid-build.
gtsam::Key velocityKey(std::uint64_t index) {
    return gtsam::Symbol('v', index).key();
}

}  // namespace

struct ImuShockSample {
    Timestamp timestamp{};
    double abs_accel{0.0};
    double accel_minus_g{0.0};
};

// Phase C state machine. Stays in kBoot until the first IMU sample arrives;
// transitions kCalibratingBias → kAwaitingFieldFix once a quiet stationary
// window has accumulated; transitions kAwaitingFieldFix → kRunning on the
// first AprilTag observation that snaps origin to the field. ImuFactors are
// suppressed before kRunning because PreintegrationParams::MakeSharedU
// depends on a defined nav frame.
enum class InitState {
    kBoot,
    kCalibratingBias,
    kAwaitingFieldFix,
    kRunning,
};

// Boot stationary detector. Accumulates IMU mean over a window during which
// chassis speed reads ≈ 0. Stops when bias_calibration_seconds elapses.
struct BiasCalibrator {
    Vec3 accel_sum{};
    Vec3 gyro_sum{};
    std::size_t count{0};
    std::optional<Timestamp> window_start;
    bool window_quiet{true};  // true ↔ no chassis-disqualifying sample seen
};

// Per-keyframe accumulator. chassis_twist_accum holds the composed Pose3
// of every chassis sample's Δt-twist since the last commit; chassis_dt_accum
// tracks the integration window length so the BetweenFactor noise model
// matches. has_imu reflects whether the active preintegrator carried at
// least one IMU sample for this interval.
struct PendingKeyframe {
    gtsam::Pose3 chassis_twist_accum;  // identity initially
    double chassis_dt_accum{0.0};
    bool has_imu{false};
    std::optional<Timestamp> window_start;
};

struct FusionBackend {
    explicit FusionBackend(FusionConfig config)
        : config_(std::move(config)),
          chassis_noise_(diagonalNoise(config_.chassis_sigmas)),
          chassis_robust_noise_(robustHuberNoise(chassis_noise_, config_.huber_k)),
          origin_prior_noise_(diagonalNoise(config_.origin_prior_sigmas)) {
        gtsam::ISAM2Params params;
        params.relinearizeThreshold = 0.01;
        params.relinearizeSkip = 1;
        isam_ = gtsam::ISAM2(params);

        // Seed bias from persisted [ax, ay, az, gx, gy, gz]. Remains the
        // active bias until the boot stationary calibration overwrites it
        // (or the calibration window times out, leaving the seed in place
        // and kFusionStatusBiasUnverified set).
        const auto& pb = config_.persisted_bias;
        current_bias_ = gtsam::imuBias::ConstantBias(
            gtsam::Vector3(pb[0], pb[1], pb[2]),
            gtsam::Vector3(pb[3], pb[4], pb[5]));

        if (config_.enable_imu_preintegration) {
            initImuParams();
            state_ = InitState::kCalibratingBias;
        }
    }

    void addImu(const ImuSample& sample) {
        const double dx = sample.accel_mps2.x - config_.gravity_local_mps2.x;
        const double dy = sample.accel_mps2.y - config_.gravity_local_mps2.y;
        const double dz = sample.accel_mps2.z - config_.gravity_local_mps2.z;
        ImuShockSample entry;
        entry.timestamp = sample.timestamp;
        entry.abs_accel = std::sqrt(
            sample.accel_mps2.x * sample.accel_mps2.x +
            sample.accel_mps2.y * sample.accel_mps2.y +
            sample.accel_mps2.z * sample.accel_mps2.z);
        entry.accel_minus_g = std::sqrt(dx * dx + dy * dy + dz * dz);
        imu_window_.push_back(entry);
        pruneImuWindow(sample.timestamp);
    }

    // Phase C: routed from FusionService::process when
    // enable_imu_preintegration is true. Returns the current state so the
    // caller can decide whether to drive the keyframe-commit path.
    InitState state() const { return state_; }
    bool imuPreintegrationActive() const {
        return config_.enable_imu_preintegration;
    }

    // Bias-calibration step. The chassis context is optional because IMU
    // arrives at higher rate; absent chassis ≈ "trust whatever quiet flag
    // we saw on the most recent chassis sample". Returns true if calibration
    // completed on this call (so the caller can update stats counters).
    bool processBiasCalibration(const ImuSample& sample) {
        if (state_ != InitState::kCalibratingBias) {
            return false;
        }
        if (!calibrator_.window_start) {
            calibrator_.window_start = sample.timestamp;
        }
        // Disqualify the window if the chassis cursor became non-quiet.
        if (calibrator_.window_quiet) {
            calibrator_.accel_sum.x += sample.accel_mps2.x;
            calibrator_.accel_sum.y += sample.accel_mps2.y;
            calibrator_.accel_sum.z += sample.accel_mps2.z;
            calibrator_.gyro_sum.x += sample.gyro_radps.x;
            calibrator_.gyro_sum.y += sample.gyro_radps.y;
            calibrator_.gyro_sum.z += sample.gyro_radps.z;
            ++calibrator_.count;
        }
        const double elapsed =
            secondsBetween(*calibrator_.window_start, sample.timestamp);
        if (elapsed < config_.bias_calibration_seconds) {
            return false;
        }
        // Window over. If we got at least one quiet sample, accept the mean
        // as the new bias seed. Otherwise leave persisted bias in place and
        // mark the result unverified.
        if (calibrator_.window_quiet && calibrator_.count > 0) {
            const double n = static_cast<double>(calibrator_.count);
            // Subtract local gravity from accel mean — the bias is the
            // residual of "what the IMU reports while stationary minus the
            // gravity we know is acting on it".
            const gtsam::Vector3 accel_bias(
                calibrator_.accel_sum.x / n - config_.gravity_local_mps2.x,
                calibrator_.accel_sum.y / n - config_.gravity_local_mps2.y,
                calibrator_.accel_sum.z / n - config_.gravity_local_mps2.z);
            const gtsam::Vector3 gyro_bias(
                calibrator_.gyro_sum.x / n,
                calibrator_.gyro_sum.y / n,
                calibrator_.gyro_sum.z / n);
            current_bias_ = gtsam::imuBias::ConstantBias(accel_bias, gyro_bias);
            bias_unverified_ = false;
        } else {
            bias_unverified_ = true;
        }
        // Reset the accumulator so a future re-entry starts clean.
        calibrator_ = BiasCalibrator{};
        // Re-build the active preintegrator using the freshly chosen bias.
        resetPreintegration();
        state_ = InitState::kAwaitingFieldFix;
        return true;
    }

    // Marks the calibration window non-quiet if the chassis cursor reports
    // motion above the threshold. Called whenever a chassis sample arrives
    // while we're still in kCalibratingBias.
    void noteChassisDuringCalibration(const ChassisSpeedsSample& sample) {
        if (state_ != InitState::kCalibratingBias) {
            return;
        }
        const double mag = std::sqrt(
            sample.vx_mps * sample.vx_mps +
            sample.vy_mps * sample.vy_mps);
        if (mag > config_.bias_calibration_chassis_threshold ||
            std::fabs(sample.omega_radps) > config_.bias_calibration_chassis_threshold) {
            calibrator_.window_quiet = false;
        }
    }

    // IMU preintegration accumulator. Drops out-of-order samples and resets
    // the preintegrator on a forward gap larger than max_imu_gap_seconds
    // (mirroring §6 of the design memo). Returns the integration outcome so
    // the caller can update stats counters and status flags.
    enum class ImuIntegrationOutcome {
        kAccepted,          // sample integrated successfully
        kSkippedNotRunning, // backend isn't in kRunning yet
        kOutOfOrder,        // monotonicity check failed; sample dropped
        kReset,             // forward gap too large; preintegrator cleared
    };
    ImuIntegrationOutcome addImuToPreintegration(const ImuSample& sample) {
        if (state_ != InitState::kRunning) {
            return ImuIntegrationOutcome::kSkippedNotRunning;
        }
        if (!last_imu_time_) {
            last_imu_time_ = sample.timestamp;
            return ImuIntegrationOutcome::kAccepted;
        }
        const double dt = secondsBetween(*last_imu_time_, sample.timestamp);
        if (dt <= 0.0) {
            return ImuIntegrationOutcome::kOutOfOrder;
        }
        if (dt > config_.max_imu_gap_seconds) {
            // Forward gap too large for safe preintegration. Reset and skip
            // the IMU factor for the in-progress keyframe; chassis-only
            // fallback covers the gap.
            resetPreintegration();
            pending_.has_imu = false;
            last_imu_time_ = sample.timestamp;
            return ImuIntegrationOutcome::kReset;
        }
        if (active_pim_) {
            const gtsam::Vector3 accel(
                sample.accel_mps2.x, sample.accel_mps2.y, sample.accel_mps2.z);
            const gtsam::Vector3 gyro(
                sample.gyro_radps.x, sample.gyro_radps.y, sample.gyro_radps.z);
            active_pim_->integrateMeasurement(accel, gyro, dt);
            pending_.has_imu = true;
        }
        last_imu_time_ = sample.timestamp;
        return ImuIntegrationOutcome::kAccepted;
    }

    // Adds a chassis sample to the pending keyframe instead of committing
    // immediately. The caller checks elapsed time post-call and triggers
    // commitKeyframe when max_keyframe_dt_seconds has passed (or when a
    // vision observation closes the keyframe).
    void accumulatePendingChassis(
        const ChassisSpeedsSample& sample, Timestamp timestamp) {
        if (!pending_.window_start) {
            pending_.window_start = timestamp;
            last_chassis_time_ = timestamp;
            return;
        }
        const double dt = secondsBetween(*last_chassis_time_, timestamp);
        if (dt <= 0.0 || dt > config_.max_chassis_dt_seconds) {
            // Resync without integrating; the chassis-gap path of
            // commitKeyframe will surface kFusionStatusChassisGap.
            last_chassis_time_ = timestamp;
            chassis_gap_seen_ = true;
            return;
        }
        const gtsam::Pose3 step(
            gtsam::Rot3::Rz(sample.omega_radps * dt),
            gtsam::Point3(sample.vx_mps * dt, sample.vy_mps * dt, 0.0));
        pending_.chassis_twist_accum =
            pending_.chassis_twist_accum.compose(step);
        pending_.chassis_dt_accum += dt;
        last_chassis_time_ = timestamp;
        if ((sample.status_flags & kChassisStatusSlip) != 0u) {
            pending_slip_reported_ = true;
        }
    }

    bool keyframeIntervalElapsed(Timestamp now) const {
        if (!pending_.window_start) {
            return false;
        }
        return secondsBetween(*pending_.window_start, now) >=
               config_.max_keyframe_dt_seconds;
    }

    // Closes the pending keyframe: builds the BetweenFactor (chassis) and
    // the ImuFactor (if has_imu), inserts new pose+velocity keys, calls
    // ISAM2.update(), resets the active preintegrator. Returns the next
    // estimate or nullopt if nothing was committed (e.g. empty pending).
    std::optional<FusedPoseEstimate> commitKeyframe(
        Timestamp timestamp,
        std::uint32_t extra_status = 0u) {
        if (state_ != InitState::kRunning) {
            return std::nullopt;
        }
        if (pending_.chassis_dt_accum <= 0.0 && !pending_.has_imu) {
            // Nothing to commit yet (no chassis dt accumulated, no IMU).
            return std::nullopt;
        }

        std::uint32_t status_flags = extra_status;
        if (chassis_gap_seen_) {
            status_flags |= kFusionStatusChassisGap;
            chassis_gap_seen_ = false;
        }
        if (pending_slip_reported_) {
            status_flags |= kFusionStatusSlipReported | kFusionStatusDegradedInput;
            pending_slip_reported_ = false;
        }

        const std::uint64_t next_index = current_index_ + 1u;
        const gtsam::Key next_pose = poseKey(next_index);
        const gtsam::Key next_vel = velocityKey(next_index);

        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial_values;

        const gtsam::Pose3 next_pose_init =
            current_pose_.compose(pending_.chassis_twist_accum);
        initial_values.insert(next_pose, next_pose_init);

        // Chassis BetweenFactor (Huber-wrapped). Rebuild noise on every step
        // so a non-trivial dt accumulation widens process noise proportionally.
        if (pending_.chassis_dt_accum > 0.0) {
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                current_key_, next_pose,
                pending_.chassis_twist_accum, chassis_robust_noise_));
        }

        // IMU factor + initial velocity guess. Use a forward-Euler velocity
        // estimate from the chassis twist as the initial guess; ImuFactor
        // refines it.
        gtsam::Vector3 next_velocity_init = current_velocity_;
        if (pending_.has_imu && active_pim_) {
            const gtsam::NavState predicted = active_pim_->predict(
                gtsam::NavState(current_pose_, current_velocity_),
                current_bias_);
            next_velocity_init = predicted.velocity();
            graph.add(gtsam::ImuFactor(
                current_key_, velocityKey(current_index_),
                next_pose, next_vel,
                gtsam::Symbol('b', 0).key(),  // bias placeholder; see init prior
                *active_pim_));
        }
        initial_values.insert(next_vel, next_velocity_init);

        if (!update(graph, initial_values, next_pose, next_index)) {
            // ISAM2 rejected the update; preserve current pose and surface
            // the optimizer error. Reset preintegration anyway — keeping a
            // half-built pim across a failure invites compounding garbage.
            resetPreintegration();
            pending_ = PendingKeyframe{};
            return estimateFromCurrent(
                timestamp, status_flags | kFusionStatusOptimizerError);
        }
        // Success: refresh velocity from the optimizer, cache new live key,
        // reset pending+pim, and return the new estimate.
        try {
            current_velocity_ = estimate_.at<gtsam::Vector3>(next_vel);
        } catch (const std::exception&) {
            // Velocity key absent — fall back to integrator output.
            current_velocity_ = next_velocity_init;
        }
        ++keyframes_committed_;
        if (pending_.has_imu) {
            ++imu_factors_committed_;
        }
        live_keys_.push_back(next_pose);
        pending_ = PendingKeyframe{};
        resetPreintegration();
        marginalizeOldKeys();
        return makeEstimate(next_pose, timestamp, status_flags);
    }

    // Sliding-window marginalization. Hands the oldest leaf keys off to
    // ISAM2.marginalizeLeaves() once live_keys_ exceeds the configured
    // window. Bounded ISAM2 update cost; keeps long matches O(window).
    void marginalizeOldKeys() {
        const std::size_t window = config_.marginalize_keyframe_window;
        if (live_keys_.size() <= window) {
            return;
        }
        gtsam::FastList<gtsam::Key> to_marginalize;
        while (live_keys_.size() > window) {
            to_marginalize.push_back(live_keys_.front());
            live_keys_.pop_front();
        }
        try {
            isam_.marginalizeLeaves(to_marginalize);
        } catch (const std::exception&) {
            // Marginalization failure is benign — the keys stay live and
            // we'll try again on the next commit. No state corruption.
        }
    }

    // Build a fresh PreintegratedImuMeasurements off the active params with
    // the current bias. Called at startup, after a long-gap reset, and after
    // every successful commit (so the next interval starts at zero).
    void resetPreintegration() {
        if (!imu_params_) {
            return;
        }
        active_pim_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(
            imu_params_, current_bias_);
        ++imu_resets_;
    }

    // Counters for FusionStats. Pulled by FusionService::stats() under mu_.
    std::uint64_t keyframesCommitted() const { return keyframes_committed_; }
    std::uint64_t imuFactorsCommitted() const { return imu_factors_committed_; }
    std::uint64_t imuOutOfOrder() const { return imu_out_of_order_; }
    std::uint64_t imuResets() const { return imu_resets_; }
    std::uint64_t biasCalibrationsCompleted() const {
        return bias_calibrations_completed_;
    }
    void incImuOutOfOrder() { ++imu_out_of_order_; }
    void incBiasCalibrationsCompleted() { ++bias_calibrations_completed_; }
    bool biasUnverified() const { return bias_unverified_; }

    // Extra status bits to OR into makeEstimate output when the IMU/init
    // state machine wants the consumer to see them.
    std::uint32_t initStateStatusBits() const {
        std::uint32_t bits = 0u;
        if (state_ == InitState::kAwaitingFieldFix) {
            bits |= kFusionStatusAwaitingFieldFix;
        }
        if (bias_unverified_) {
            bits |= kFusionStatusBiasUnverified;
        }
        return bits;
    }

    // Body-frame velocity for output. Empty until kRunning has populated it
    // through ImuFactor / chassis projection.
    std::optional<Vec3> currentVelocity() const {
        if (state_ != InitState::kRunning) {
            return std::nullopt;
        }
        return Vec3{
            current_velocity_(0),
            current_velocity_(1),
            current_velocity_(2),
        };
    }

    std::optional<FusedPoseEstimate> addChassisSpeeds(
        const ChassisSpeedsSample& sample,
        Timestamp timestamp) {
        std::uint32_t status_flags = kFusionStatusVisionUnavailable;
        if (sample.status_flags != 0u) {
            status_flags |= kFusionStatusDegradedInput;
        }
        // Phase C: surface init-state bits so consumers can see we're in
        // pre-running dead-reckoning even on the legacy chassis path.
        status_flags |= initStateStatusBits();
        const bool slip_reported =
            (sample.status_flags & kChassisStatusSlip) != 0u;

        if (!initialized_) {
            // Bootstrap: anchor x0 at identity; no relative motion factor yet.
            gtsam::NonlinearFactorGraph graph;
            gtsam::Values initial_values;
            const gtsam::Pose3 origin;
            const gtsam::Key origin_key = poseKey(0);
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                origin_key, origin, origin_prior_noise_));
            initial_values.insert(origin_key, origin);
            if (!update(graph, initial_values, origin_key, 0)) {
                return std::nullopt;
            }
            last_chassis_time_ = timestamp;
            return makeEstimate(origin_key, timestamp, status_flags);
        }

        const double dt = secondsBetween(*last_chassis_time_, timestamp);
        if (dt <= 0.0 || dt > config_.max_chassis_dt_seconds) {
            // Skip the BetweenFactor; resync the cursor so the next sample
            // integrates a sensible Δt instead of compounding the gap.
            last_chassis_time_ = timestamp;
            return estimateFromCurrent(timestamp, status_flags | kFusionStatusChassisGap);
        }

        // Inflation fires on either the IMU shock/free-fall detector or an
        // explicit RIO slip flag. The two are independent: slip can happen
        // with the IMU quiet (wheels spinning on a low-µ surface), and a
        // hard impact can happen with no slip. Both bits surface on output.
        const bool shock_inflated = detectShockOrFreefall(timestamp);
        gtsam::SharedNoiseModel step_noise = chassis_robust_noise_;
        if (shock_inflated || slip_reported) {
            std::array<double, 6> sigmas = config_.chassis_sigmas;
            for (auto& sigma : sigmas) {
                sigma *= config_.shock_inflation_factor;
            }
            step_noise = robustHuberNoise(diagonalNoise(sigmas), config_.huber_k);
            if (shock_inflated) {
                status_flags |= kFusionStatusShockInflated;
            }
            if (slip_reported) {
                status_flags |= kFusionStatusSlipReported;
            }
        }

        const gtsam::Pose3 delta(
            gtsam::Rot3::Rz(sample.omega_radps * dt),
            gtsam::Point3(sample.vx_mps * dt, sample.vy_mps * dt, 0.0));

        const std::uint64_t next_index = current_index_ + 1u;
        const gtsam::Key next_key = poseKey(next_index);

        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial_values;
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            current_key_, next_key, delta, step_noise));
        initial_values.insert(next_key, current_pose_.compose(delta));

        if (!update(graph, initial_values, next_key, next_index)) {
            return estimateFromCurrent(timestamp, status_flags | kFusionStatusOptimizerError);
        }
        last_chassis_time_ = timestamp;
        return makeEstimate(next_key, timestamp, status_flags);
    }

    // VioMeasurement → BetweenFactor<Pose3> on the existing key chain. The
    // key-advance pattern mirrors addChassisSpeeds: VIO and chassis share one
    // chain so their factors compose. The placeholder VIO pipeline publishes
    // tracking_ok=false; that branch is a no-op until a real frontend lands.
    enum class VioOutcome { kNoTracking, kSkipped, kProcessed };

    std::pair<std::optional<FusedPoseEstimate>, VioOutcome> addVio(
        const VioMeasurement& measurement,
        Timestamp timestamp) {
        if (!config_.enable_vio) {
            return {std::nullopt, VioOutcome::kSkipped};
        }
        if (!measurement.tracking_ok) {
            return {std::nullopt, VioOutcome::kNoTracking};
        }
        if (!initialized_) {
            // Need a chassis or AprilTag bootstrap before we can attach
            // relative-motion factors.
            return {std::nullopt, VioOutcome::kSkipped};
        }

        gtsam::SharedNoiseModel noise = diagonalNoise(config_.vio_default_sigmas);
        bool covariance_pd = false;
        gtsam::Matrix6 covariance_matrix;
        for (int row = 0; row < 6; ++row) {
            for (int col = 0; col < 6; ++col) {
                covariance_matrix(row, col) =
                    measurement.covariance[static_cast<std::size_t>(row * 6 + col)];
            }
        }
        // Cheap PD check: at least one diagonal must be > 0. The full
        // Gaussian::Covariance constructor will throw on non-PD, in which case
        // we fall back to the configured default sigmas.
        for (int i = 0; i < 6; ++i) {
            if (covariance_matrix(i, i) > 0.0) {
                covariance_pd = true;
                break;
            }
        }
        if (covariance_pd) {
            try {
                noise = gtsam::noiseModel::Gaussian::Covariance(covariance_matrix);
            } catch (const std::exception&) {
                noise = diagonalNoise(config_.vio_default_sigmas);
            }
        }

        const gtsam::Pose3 delta = toGtsamPose(measurement.relative_motion);
        const std::uint64_t next_index = current_index_ + 1u;
        const gtsam::Key next_key = poseKey(next_index);

        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial_values;
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            current_key_, next_key, delta, noise));
        initial_values.insert(next_key, current_pose_.compose(delta));

        if (!update(graph, initial_values, next_key, next_index)) {
            return {estimateFromCurrent(timestamp, kFusionStatusOptimizerError),
                    VioOutcome::kProcessed};
        }
        return {makeEstimate(next_key, timestamp, 0u), VioOutcome::kProcessed};
    }

    // Live graph-size accessors used by the per-stage telemetry pulled from
    // FusionService::stats(). Access is single-threaded under the service's
    // mutex (the same worker thread that calls update() is the only writer).
    std::size_t factorCount() const { return isam_.getFactorsUnsafe().size(); }
    std::size_t variableCount() const { return estimate_.size(); }

    std::optional<FusedPoseEstimate> addAprilTags(
        const AprilTagObservation& observation,
        Timestamp timestamp) {
        std::uint32_t status_flags = 0u;
        if (!observation.field_to_robot.has_value()) {
            status_flags |= kFusionStatusVisionUnavailable;
            return estimateFromCurrent(timestamp, status_flags);
        }

        gtsam::Matrix6 covariance_matrix;
        for (int row = 0; row < 6; ++row) {
            for (int col = 0; col < 6; ++col) {
                covariance_matrix(row, col) =
                    observation.covariance[static_cast<std::size_t>(row * 6 + col)];
            }
        }

        gtsam::SharedNoiseModel vision_noise;
        try {
            vision_noise = gtsam::noiseModel::Gaussian::Covariance(covariance_matrix);
        } catch (const std::exception&) {
            return estimateFromCurrent(timestamp, status_flags | kFusionStatusOptimizerError);
        }

        const gtsam::Pose3 field_to_robot = toGtsamPose(*observation.field_to_robot);

        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial_values;
        gtsam::Key key = current_key_;
        std::uint64_t index = current_index_;
        if (!initialized_) {
            key = poseKey(0);
            index = 0;
            initial_values.insert(key, field_to_robot);
        }
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(key, field_to_robot, vision_noise));

        if (!update(graph, initial_values, key, index)) {
            return estimateFromCurrent(timestamp, status_flags | kFusionStatusOptimizerError);
        }
        // Phase C: vision is the field-fix gate. The first successful vision
        // update transitions kAwaitingFieldFix → kRunning so IMU factors can
        // start contributing on subsequent keyframes.
        onFieldFixed(timestamp);
        return makeEstimate(key, timestamp, status_flags | initStateStatusBits());
    }

    // Promote kAwaitingFieldFix → kRunning. Called by the addAprilTags path
    // immediately after the first vision update succeeds while preintegration
    // is enabled. Inserts an initial velocity variable so future ImuFactors
    // have a key to link to.
    void onFieldFixed(Timestamp timestamp) {
        if (state_ != InitState::kAwaitingFieldFix || !config_.enable_imu_preintegration) {
            return;
        }
        // Insert v_<current_index> = 0 into the graph so the next keyframe's
        // ImuFactor has a tail key to reference. Wrap in a tiny prior on the
        // velocity so ISAM2 has constraint to relinearize against — this is
        // cheap and disappears as IMU integration takes over.
        try {
            gtsam::NonlinearFactorGraph graph;
            gtsam::Values initial_values;
            const gtsam::Key v_key = velocityKey(current_index_);
            const gtsam::Vector3 zero_vel = gtsam::Vector3::Zero();
            initial_values.insert(v_key, zero_vel);
            // Velocity prior σ = 5 m/s per axis; loose enough to converge to
            // truth on the first ImuFactor but tight enough to avoid divergence.
            const gtsam::Vector3 vel_sigmas(5.0, 5.0, 5.0);
            graph.add(gtsam::PriorFactor<gtsam::Vector3>(
                v_key, zero_vel,
                gtsam::noiseModel::Diagonal::Sigmas(vel_sigmas)));
            isam_.update(graph, initial_values);
            current_velocity_ = zero_vel;
            live_keys_.push_back(current_key_);
            last_imu_time_.reset();
            resetPreintegration();
            state_ = InitState::kRunning;
            (void)timestamp;
        } catch (const std::exception&) {
            // If the velocity insert fails, stay in kAwaitingFieldFix and
            // try again on the next vision update. No state corruption.
        }
    }

private:
    void initImuParams() {
        // Nav frame is "Z up" — the field plane is horizontal so this is the
        // canonical FRC choice. PreintegrationParams::MakeSharedU bakes in
        // the gravity vector internally; the host need not subtract it
        // again from accel samples. body_P_sensor handles any IMU-mount tilt.
        imu_params_ = gtsam::PreintegrationParams::MakeSharedU(9.80665);
        // We use gtsam::ImuFactor (not CombinedImuFactor), so bias is a
        // constant injected per factor — bias-rw covariance lives on
        // CombinedImuFactor's params and isn't applicable here. The host
        // re-estimates bias only at boot (stationary calibration), so the
        // configured bias_rw values are surfaced via stats but unused
        // until/unless we migrate to CombinedImuFactor.
        const double a_n = config_.accel_noise_sigma;
        const double g_n = config_.gyro_noise_sigma;
        const double i_c = config_.integration_cov_sigma;
        imu_params_->accelerometerCovariance =
            gtsam::Matrix3::Identity() * (a_n * a_n);
        imu_params_->gyroscopeCovariance =
            gtsam::Matrix3::Identity() * (g_n * g_n);
        imu_params_->integrationCovariance =
            gtsam::Matrix3::Identity() * (i_c * i_c);
        imu_params_->body_P_sensor =
            toGtsamPose(config_.imu_extrinsic_body_to_imu);
    }

    bool update(
        const gtsam::NonlinearFactorGraph& graph,
        const gtsam::Values& initial_values,
        gtsam::Key current_key,
        std::uint64_t current_index) {
        try {
            isam_.update(graph, initial_values);
            isam_.update();
            estimate_ = isam_.calculateEstimate();
            current_pose_ = estimate_.at<gtsam::Pose3>(current_key);
            current_key_ = current_key;
            current_index_ = current_index;
            initialized_ = true;
            return true;
        } catch (const std::exception&) {
            return false;
        }
    }

    std::optional<FusedPoseEstimate> estimateFromCurrent(
        Timestamp timestamp,
        std::uint32_t status_flags) {
        if (!initialized_) {
            return std::nullopt;
        }
        return makeEstimate(current_key_, timestamp, status_flags);
    }

    std::optional<FusedPoseEstimate> makeEstimate(
        gtsam::Key key,
        Timestamp timestamp,
        std::uint32_t status_flags) {
        if (!initialized_) {
            return std::nullopt;
        }

        FusedPoseEstimate estimate;
        estimate.timestamp = timestamp;
        estimate.field_to_robot = toPose2d(current_pose_);
        // Phase C: populate velocity once the state machine is kRunning.
        // currentVelocity() returns nullopt before then so the wire format
        // signals "unavailable" rather than emitting a stale zero.
        estimate.velocity = currentVelocity();
        estimate.status_flags = status_flags;

        try {
            const gtsam::Matrix covariance = isam_.marginalCovariance(key);
            for (int row = 0; row < 6; ++row) {
                for (int col = 0; col < 6; ++col) {
                    estimate.covariance[static_cast<std::size_t>(row * 6 + col)] =
                        covariance(row, col);
                }
            }
        } catch (const std::exception&) {
            estimate.status_flags |= kFusionStatusMarginalUnavailable;
        }

        return estimate;
    }

    void pruneImuWindow(Timestamp now) {
        const auto window =
            std::chrono::duration_cast<Timestamp::duration>(
                std::chrono::duration<double>(config_.imu_window_seconds));
        while (!imu_window_.empty() && imu_window_.front().timestamp + window < now) {
            imu_window_.pop_front();
        }
    }

    bool detectShockOrFreefall(Timestamp window_end) {
        pruneImuWindow(window_end);
        for (const auto& entry : imu_window_) {
            if (entry.accel_minus_g > config_.shock_threshold_mps2 ||
                entry.abs_accel < config_.freefall_threshold_mps2) {
                return true;
            }
        }
        return false;
    }

    FusionConfig config_;
    gtsam::SharedNoiseModel chassis_noise_;
    // Huber-wrapped chassis_noise_ used on every non-inflated step. The raw
    // chassis_noise_ stays around so the inflated branch can rebuild a
    // multiplied-sigma noise without wrapping a wrap.
    gtsam::SharedNoiseModel chassis_robust_noise_;
    gtsam::SharedNoiseModel origin_prior_noise_;
    gtsam::ISAM2 isam_;
    gtsam::Values estimate_;
    gtsam::Pose3 current_pose_;
    gtsam::Key current_key_{poseKey(0)};
    std::uint64_t current_index_{0};
    bool initialized_{false};
    std::optional<Timestamp> last_chassis_time_;
    std::deque<ImuShockSample> imu_window_;

    // Phase C state. All inert when config_.enable_imu_preintegration is
    // false; kept as members so the (hot) chassis path can read them
    // without conditional branching on the flag.
    InitState state_{InitState::kBoot};
    boost::shared_ptr<gtsam::PreintegrationParams> imu_params_;
    gtsam::imuBias::ConstantBias current_bias_;
    std::unique_ptr<gtsam::PreintegratedImuMeasurements> active_pim_;
    gtsam::Vector3 current_velocity_{gtsam::Vector3::Zero()};
    std::optional<Timestamp> last_imu_time_;
    BiasCalibrator calibrator_;
    PendingKeyframe pending_;
    std::deque<gtsam::Key> live_keys_;
    bool bias_unverified_{false};
    bool chassis_gap_seen_{false};
    bool pending_slip_reported_{false};
    std::uint64_t keyframes_committed_{0};
    std::uint64_t imu_factors_committed_{0};
    std::uint64_t imu_out_of_order_{0};
    std::uint64_t imu_resets_{0};
    std::uint64_t bias_calibrations_completed_{0};
};

FusionConfig buildFusionConfig(const runtime::RuntimeConfig& runtime_config) {
    const auto& src = runtime_config.fusion;
    FusionConfig out;
    out.chassis_sigmas = src.chassis_sigmas;
    out.origin_prior_sigmas = src.origin_prior_sigmas;
    out.shock_threshold_mps2 = src.shock_threshold_mps2;
    out.freefall_threshold_mps2 = src.freefall_threshold_mps2;
    out.shock_inflation_factor = src.shock_inflation_factor;
    out.imu_window_seconds = src.imu_window_seconds;
    out.max_chassis_dt_seconds = src.max_chassis_dt_seconds;
    out.gravity_local_mps2 = src.gravity_local_mps2;
    out.enable_vio = src.enable_vio;
    out.vio_default_sigmas = src.vio_default_sigmas;
    out.huber_k = src.huber_k;
    out.enable_imu_preintegration = src.enable_imu_preintegration;
    out.imu_extrinsic_body_to_imu = src.imu_extrinsic_body_to_imu;
    out.accel_noise_sigma = src.accel_noise_sigma;
    out.gyro_noise_sigma = src.gyro_noise_sigma;
    out.accel_bias_rw_sigma = src.accel_bias_rw_sigma;
    out.gyro_bias_rw_sigma = src.gyro_bias_rw_sigma;
    out.integration_cov_sigma = src.integration_cov_sigma;
    out.persisted_bias = src.persisted_bias;
    out.bias_calibration_seconds = src.bias_calibration_seconds;
    out.bias_calibration_chassis_threshold = src.bias_calibration_chassis_threshold;
    out.max_keyframe_dt_seconds = src.max_keyframe_dt_seconds;
    out.max_imu_gap_seconds = src.max_imu_gap_seconds;
    out.marginalize_keyframe_window = src.marginalize_keyframe_window;
    out.slip_disagreement_mps = src.slip_disagreement_mps;
    return out;
}

FusionService::FusionService(MeasurementBus& measurement_bus, FusionConfig config)
    : measurement_bus_(measurement_bus),
      backend_(std::make_unique<FusionBackend>(std::move(config))) {}

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
    FusionStats s = stats_;
    s.graph_update_us = graph_update_hist_.snapshot();
    s.publish_us = publish_hist_.snapshot();
    s.graph_factor_count = graph_factor_count_;
    s.graph_variable_count = graph_variable_count_;
    s.last_update_wall_clock_us = last_update_wall_clock_us_;
    return s;
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
    // Stage 0: capture the moment the worker takes a measurement off the
    // bus. graph_update_us measures stage-0 → ISAM2.update() return.
    const auto bus_pop_time = std::chrono::steady_clock::now();

    if (const auto* imu = std::get_if<ImuSample>(&measurement)) {
        // IMU samples feed the shock-detection window in every mode; they
        // bypass the per-type cursor since the 1 kHz IMU and ~100 Hz
        // chassis-speeds streams interleave naturally.
        backend_->addImu(*imu);
        if (backend_->imuPreintegrationActive()) {
            // Bias calibration → preintegration accumulator → counters.
            const bool calibrated = backend_->processBiasCalibration(*imu);
            if (calibrated) {
                std::lock_guard<std::mutex> g(mu_);
                ++stats_.bias_calibrations_completed;
                backend_->incBiasCalibrationsCompleted();
            }
            const auto outcome = backend_->addImuToPreintegration(*imu);
            if (outcome == FusionBackend::ImuIntegrationOutcome::kOutOfOrder) {
                std::lock_guard<std::mutex> g(mu_);
                ++stats_.imu_out_of_order;
                backend_->incImuOutOfOrder();
            } else if (outcome == FusionBackend::ImuIntegrationOutcome::kReset) {
                std::lock_guard<std::mutex> g(mu_);
                ++stats_.imu_resets;
            }
        }
        std::lock_guard<std::mutex> g(mu_);
        ++stats_.measurements_processed;
        return;
    }

    if (!isSupportedMeasurement(measurement)) {
        return;
    }

    const Timestamp measurement_time = timestampOf(measurement);
    if (!acceptTimestamp(measurement, measurement_time)) {
        return;
    }

    std::optional<FusedPoseEstimate> estimate;
    if (const auto* chassis = std::get_if<ChassisSpeedsSample>(&measurement)) {
        // Phase C dispatch:
        //   kCalibratingBias / kAwaitingFieldFix → use the legacy chassis
        //     BetweenFactor path (no IMU factor) so the graph still moves
        //     during pre-vision dead-reckoning.
        //   kRunning → accumulate into the pending keyframe and commit when
        //     the keyframe interval elapses, attaching an ImuFactor.
        if (backend_->imuPreintegrationActive()) {
            backend_->noteChassisDuringCalibration(*chassis);
        }
        if (backend_->imuPreintegrationActive() &&
            backend_->state() == InitState::kRunning) {
            backend_->accumulatePendingChassis(*chassis, measurement_time);
            if (backend_->keyframeIntervalElapsed(measurement_time)) {
                estimate = backend_->commitKeyframe(measurement_time);
                if (estimate) {
                    std::lock_guard<std::mutex> g(mu_);
                    ++stats_.keyframes_committed;
                    stats_.imu_factors_committed = backend_->imuFactorsCommitted();
                }
            }
        } else {
            estimate = backend_->addChassisSpeeds(*chassis, measurement_time);
        }
    } else if (const auto* tags = std::get_if<AprilTagObservation>(&measurement)) {
        estimate = backend_->addAprilTags(*tags, measurement_time);
        // Force a keyframe close after every vision update once running, so
        // the ImuFactor for the in-flight interval gets committed against the
        // freshly-anchored pose. addAprilTags transitions kAwaitingFieldFix
        // → kRunning internally, so the first vision update doesn't trigger
        // commit (no pending keyframe yet) but subsequent ones do.
        if (backend_->imuPreintegrationActive() &&
            backend_->state() == InitState::kRunning) {
            auto kf = backend_->commitKeyframe(measurement_time);
            if (kf) {
                estimate = std::move(kf);
                std::lock_guard<std::mutex> g(mu_);
                ++stats_.keyframes_committed;
                stats_.imu_factors_committed = backend_->imuFactorsCommitted();
            }
        }
    } else if (const auto* vio = std::get_if<VioMeasurement>(&measurement)) {
        auto [vio_estimate, outcome] = backend_->addVio(*vio, measurement_time);
        estimate = std::move(vio_estimate);
        std::lock_guard<std::mutex> g(mu_);
        if (outcome == FusionBackend::VioOutcome::kProcessed) {
            ++stats_.measurements_vio_processed;
        } else if (outcome == FusionBackend::VioOutcome::kNoTracking) {
            ++stats_.measurements_vio_skipped_no_tracking;
        }
    }

    // Stage 1: graph mutation finished. Record bus-pop → here regardless of
    // estimate emission so a measurement that failed inside the optimizer
    // (no estimate returned) still shows up in the histogram.
    const auto update_done_time = std::chrono::steady_clock::now();
    const auto graph_update_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            update_done_time - bus_pop_time).count();
    {
        std::lock_guard<std::mutex> g(mu_);
        graph_update_hist_.recordSample(graph_update_us);
        graph_factor_count_ = backend_->factorCount();
        graph_variable_count_ = backend_->variableCount();
        last_update_wall_clock_us_ =
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
    }

    if (estimate) {
        publishEstimate(*estimate, update_done_time);
    }
}

bool FusionService::isSupportedMeasurement(const Measurement& measurement) const {
    return std::holds_alternative<ChassisSpeedsSample>(measurement) ||
           std::holds_alternative<AprilTagObservation>(measurement) ||
           std::holds_alternative<VioMeasurement>(measurement);
}

bool FusionService::acceptTimestamp(
    const Measurement& measurement, Timestamp timestamp) {
    static_assert(std::variant_size_v<Measurement> == kMeasurementTypeCount,
                  "FusionService per-type cursor array must match the Measurement variant size");
    std::lock_guard<std::mutex> g(mu_);
    const std::size_t idx = measurement.index();
    auto& cursor = per_type_cursors_[idx];
    if (cursor && timestamp < *cursor) {
        ++stats_.stale_measurements;
        return false;
    }
    cursor = timestamp;
    if (!stats_.last_measurement_time ||
        timestamp > *stats_.last_measurement_time) {
        stats_.last_measurement_time = timestamp;
    }
    ++stats_.measurements_processed;
    return true;
}

void FusionService::publishEstimate(
    FusedPoseEstimate estimate,
    std::chrono::steady_clock::time_point publish_started) {
    std::vector<std::shared_ptr<IFusionOutputSink>> sinks;
    {
        std::lock_guard<std::mutex> g(mu_);
        latest_estimate_ = estimate;
        sinks = sinks_;
    }

    for (auto& sink : sinks) {
        sink->publish(estimate);
    }

    // Stage 2: sink fan-out done. The histogram measures the cumulative cost
    // of every sink's publish() — sinks promise to return quickly (they only
    // enqueue, never block on I/O), so this should stay in single-digit µs.
    const auto publish_done = std::chrono::steady_clock::now();
    const auto publish_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            publish_done - publish_started).count();
    std::lock_guard<std::mutex> g(mu_);
    publish_hist_.recordSample(publish_us);
}

}  // namespace posest::fusion
