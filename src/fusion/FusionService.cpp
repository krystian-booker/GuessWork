#include "posest/fusion/FusionService.h"

#include <cmath>
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

gtsam::Pose3 toPlanarPose(const Pose2d& pose) {
    return gtsam::Pose3(
        gtsam::Rot3::RzRyRx(0.0, 0.0, pose.theta_rad),
        gtsam::Point3(pose.x_m, pose.y_m, 0.0));
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

std::optional<gtsam::Pose3> fieldToRobotFromTag(
    const FusionConfig& config,
    const AprilTagObservation& observation,
    const AprilTagDetection& detection,
    std::uint32_t& status_flags) {
    if (!detection.camera_to_tag) {
        status_flags |= kFusionStatusDegradedInput;
        return std::nullopt;
    }

    const auto field_tag_it = config.field_to_tags.find(detection.tag_id);
    if (field_tag_it == config.field_to_tags.end()) {
        status_flags |= kFusionStatusDegradedInput;
        return std::nullopt;
    }

    const auto camera_robot_it = config.camera_to_robot.find(observation.camera_id);
    if (camera_robot_it == config.camera_to_robot.end()) {
        status_flags |= kFusionStatusDegradedInput;
        return std::nullopt;
    }

    const gtsam::Pose3 field_to_tag = toGtsamPose(field_tag_it->second);
    const gtsam::Pose3 camera_to_tag = toGtsamPose(*detection.camera_to_tag);
    const gtsam::Pose3 camera_to_robot = toGtsamPose(camera_robot_it->second);
    return field_to_tag.compose(camera_to_tag.inverse()).compose(camera_to_robot);
}

}  // namespace

struct FusionBackend {
    explicit FusionBackend(FusionConfig config)
        : config_(std::move(config)),
          wheel_noise_(diagonalNoise(config_.wheel_sigmas)),
          vision_noise_(diagonalNoise(config_.vision_sigmas)),
          origin_prior_noise_(diagonalNoise(config_.origin_prior_sigmas)) {
        gtsam::ISAM2Params params;
        params.relinearizeThreshold = 0.01;
        params.relinearizeSkip = 1;
        isam_ = gtsam::ISAM2(params);
    }

    std::optional<FusedPoseEstimate> addWheel(
        const WheelOdometrySample& sample,
        Timestamp timestamp) {
        std::uint32_t status_flags = kFusionStatusVisionUnavailable;
        if (sample.status_flags != 0u) {
            status_flags |= kFusionStatusDegradedInput;
        }

        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial_values;
        const gtsam::Pose3 delta = toPlanarPose(sample.chassis_delta);

        std::uint64_t next_index = 0;
        gtsam::Key next_key = poseKey(0);
        if (!initialized_) {
            const gtsam::Pose3 origin;
            const gtsam::Key origin_key = poseKey(0);
            next_index = 1;
            next_key = poseKey(next_index);
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                origin_key, origin, origin_prior_noise_));
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                origin_key, next_key, delta, wheel_noise_));
            initial_values.insert(origin_key, origin);
            initial_values.insert(next_key, origin.compose(delta));
        } else {
            next_index = current_index_ + 1u;
            next_key = poseKey(next_index);
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                current_key_, next_key, delta, wheel_noise_));
            initial_values.insert(next_key, current_pose_.compose(delta));
        }

        if (!update(graph, initial_values, next_key, next_index)) {
            return estimateFromCurrent(timestamp, status_flags | kFusionStatusOptimizerError);
        }
        return makeEstimate(next_key, timestamp, status_flags);
    }

    std::optional<FusedPoseEstimate> addAprilTags(
        const AprilTagObservation& observation,
        Timestamp timestamp) {
        std::uint32_t status_flags = 0u;
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial_values;
        std::optional<gtsam::Pose3> first_pose;

        for (const auto& detection : observation.detections) {
            auto field_to_robot =
                fieldToRobotFromTag(config_, observation, detection, status_flags);
            if (!field_to_robot) {
                continue;
            }
            if (!first_pose) {
                first_pose = *field_to_robot;
            }
        }

        if (!first_pose) {
            status_flags |= kFusionStatusVisionUnavailable;
            return estimateFromCurrent(timestamp, status_flags);
        }

        gtsam::Key key = current_key_;
        std::uint64_t index = current_index_;
        if (!initialized_) {
            key = poseKey(0);
            index = 0;
            initial_values.insert(key, *first_pose);
        }

        for (const auto& detection : observation.detections) {
            auto field_to_robot =
                fieldToRobotFromTag(config_, observation, detection, status_flags);
            if (field_to_robot) {
                graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                    key, *field_to_robot, vision_noise_));
            }
        }

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

    FusionConfig config_;
    gtsam::SharedNoiseModel wheel_noise_;
    gtsam::SharedNoiseModel vision_noise_;
    gtsam::SharedNoiseModel origin_prior_noise_;
    gtsam::ISAM2 isam_;
    gtsam::Values estimate_;
    gtsam::Pose3 current_pose_;
    gtsam::Key current_key_{poseKey(0)};
    std::uint64_t current_index_{0};
    bool initialized_{false};
};

FusionConfig buildFusionConfig(const runtime::RuntimeConfig& runtime_config) {
    FusionConfig config;

    for (const auto& layout : runtime_config.field_layouts) {
        if (layout.id != runtime_config.active_field_layout_id) {
            continue;
        }
        for (const auto& tag : layout.tags) {
            config.field_to_tags[tag.tag_id] = tag.field_to_tag;
        }
        break;
    }

    std::unordered_map<std::string, std::string> active_versions;
    for (const auto& calibration : runtime_config.calibrations) {
        if (calibration.active) {
            active_versions[calibration.camera_id] = calibration.version;
        }
    }
    for (const auto& extrinsics : runtime_config.camera_extrinsics) {
        const auto version_it = active_versions.find(extrinsics.camera_id);
        if (version_it != active_versions.end() &&
            version_it->second == extrinsics.version) {
            config.camera_to_robot[extrinsics.camera_id] = extrinsics.camera_to_robot;
        }
    }

    return config;
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
    if (!isSupportedMeasurement(measurement)) {
        return;
    }

    const Timestamp measurement_time = timestampOf(measurement);
    if (!acceptTimestamp(measurement_time)) {
        return;
    }

    std::optional<FusedPoseEstimate> estimate;
    if (const auto* odom = std::get_if<WheelOdometrySample>(&measurement)) {
        estimate = backend_->addWheel(*odom, measurement_time);
    } else if (const auto* tags = std::get_if<AprilTagObservation>(&measurement)) {
        estimate = backend_->addAprilTags(*tags, measurement_time);
    }

    if (estimate) {
        publishEstimate(*estimate);
    }
}

bool FusionService::isSupportedMeasurement(const Measurement& measurement) const {
    return std::holds_alternative<WheelOdometrySample>(measurement) ||
           std::holds_alternative<AprilTagObservation>(measurement);
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
