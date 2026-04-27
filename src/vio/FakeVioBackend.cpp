#include "posest/vio/FakeVioBackend.h"

#include <utility>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

namespace posest::vio {

FakeVioBackend::Config::Config()
    : frame_delta(gtsam::Rot3::Yaw(0.01),
                  gtsam::Point3(0.05, 0.0, 0.0)) {}

FakeVioBackend::FakeVioBackend(Config config) : config_(std::move(config)) {}

FakeVioBackend::~FakeVioBackend() = default;

void FakeVioBackend::setOutputCallback(OutputCallback cb) {
    callback_ = std::move(cb);
}

void FakeVioBackend::start() {
    running_.store(true);
}

void FakeVioBackend::stop() {
    running_.store(false);
}

bool FakeVioBackend::tryPushFrame(std::uint64_t teensy_time_us,
                                  const cv::Mat& /*image*/) {
    if (!running_.load()) {
        return false;
    }

    const std::uint64_t this_index = ++frame_index_;
    if (config_.fail_push_on_frame != 0 &&
        this_index == config_.fail_push_on_frame) {
        return false;
    }
    frame_push_count_.fetch_add(1);

    VioBackendOutput out;
    out.teensy_time_us = teensy_time_us;
    {
        std::lock_guard<std::mutex> g(pose_mu_);
        current_pose_ = current_pose_.compose(config_.frame_delta);
        out.world_T_body = current_pose_;
    }

    Eigen::Matrix<double, 6, 6> cov =
        Eigen::Matrix<double, 6, 6>::Zero();
    const double r2 = config_.sigma_rotation_rad * config_.sigma_rotation_rad;
    const double t2 =
        config_.sigma_translation_m * config_.sigma_translation_m;
    cov(0, 0) = cov(1, 1) = cov(2, 2) = r2;
    cov(3, 3) = cov(4, 4) = cov(5, 5) = t2;
    out.pose_covariance = cov;

    out.tracking_ok = this_index > config_.tracking_ok_after_n_frames;
    out.backend_status = out.tracking_ok ? "fake_ok" : "fake_warmup";

    if (callback_) {
        callback_(out);
    }
    return true;
}

bool FakeVioBackend::tryPushImu(std::uint64_t /*teensy_time_us*/,
                                const Eigen::Vector3d& /*accel_mps2*/,
                                const Eigen::Vector3d& /*gyro_radps*/) {
    if (!running_.load()) {
        return false;
    }
    imu_push_count_.fetch_add(1);
    return true;
}

}  // namespace posest::vio
