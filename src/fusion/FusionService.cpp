#include "posest/fusion/FusionService.h"

#include <chrono>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
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

gtsam::Key poseKey(std::uint64_t index) {
    return gtsam::Symbol('x', index).key();
}

}  // namespace

struct ImuShockSample {
    Timestamp timestamp{};
    double abs_accel{0.0};
    double accel_minus_g{0.0};
};

struct FusionBackend {
    explicit FusionBackend(FusionConfig config)
        : config_(std::move(config)),
          chassis_noise_(diagonalNoise(config_.chassis_sigmas)),
          origin_prior_noise_(diagonalNoise(config_.origin_prior_sigmas)) {
        gtsam::ISAM2Params params;
        params.relinearizeThreshold = 0.01;
        params.relinearizeSkip = 1;
        isam_ = gtsam::ISAM2(params);
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

    std::optional<FusedPoseEstimate> addChassisSpeeds(
        const ChassisSpeedsSample& sample,
        Timestamp timestamp) {
        std::uint32_t status_flags = kFusionStatusVisionUnavailable;
        if (sample.status_flags != 0u) {
            status_flags |= kFusionStatusDegradedInput;
        }

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

        const bool inflated = detectShockOrFreefall(timestamp);
        gtsam::SharedNoiseModel step_noise = chassis_noise_;
        if (inflated) {
            std::array<double, 6> sigmas = config_.chassis_sigmas;
            for (auto& sigma : sigmas) {
                sigma *= config_.shock_inflation_factor;
            }
            step_noise = diagonalNoise(sigmas);
            status_flags |= kFusionStatusShockInflated;
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
        return makeEstimate(key, timestamp, status_flags);
    }

private:
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
    gtsam::SharedNoiseModel origin_prior_noise_;
    gtsam::ISAM2 isam_;
    gtsam::Values estimate_;
    gtsam::Pose3 current_pose_;
    gtsam::Key current_key_{poseKey(0)};
    std::uint64_t current_index_{0};
    bool initialized_{false};
    std::optional<Timestamp> last_chassis_time_;
    std::deque<ImuShockSample> imu_window_;
};

FusionConfig buildFusionConfig(const runtime::RuntimeConfig& /*runtime_config*/) {
    return FusionConfig{};
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
    if (const auto* imu = std::get_if<ImuSample>(&measurement)) {
        // IMU samples feed the shock-detection window only; they don't push
        // the per-type cursor, since the 1 kHz IMU and ~100 Hz chassis-speeds
        // streams interleave naturally and the shock detector windows them
        // internally.
        backend_->addImu(*imu);
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
        estimate = backend_->addChassisSpeeds(*chassis, measurement_time);
    } else if (const auto* tags = std::get_if<AprilTagObservation>(&measurement)) {
        estimate = backend_->addAprilTags(*tags, measurement_time);
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

    if (estimate) {
        publishEstimate(*estimate);
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
