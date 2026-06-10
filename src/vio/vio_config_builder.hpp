#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>

#include "apriltag/pose_math.hpp"
#include "calibration/calibration_store.hpp"

// Builds the OpenVINS runner configuration from stored calibration — the
// "calibration is consumed programmatically" boundary. Two-stage by design:
// this module produces a plain inspectable VioRunnerConfig (golden-testable
// with no OpenVINS dependency); openvins_runner.cpp's Pimpl maps it onto
// ov_msckf::VioManagerOptions.

namespace gw::vio {

class VioConfigError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct VioCameraConfig {
    bool                    equidistant = false;  // else radtan
    std::array<double, 4>   fxfycxcy{};
    std::array<double, 4>   dist{};
    std::array<uint32_t, 2> wh{};
    gw::apriltag::Mat4      T_cam_imu = gw::apriltag::mat4_identity();
};

// IMU noise in Kalibr continuous-time units (mirrors imu_config columns —
// passed as plain values so gw_vio doesn't depend on gw_server).
struct VioImuNoise {
    double gyro_noise_density  = 0;  // rad/s/√Hz
    double gyro_random_walk    = 0;  // rad/s²/√Hz
    double accel_noise_density = 0;  // m/s²/√Hz
    double accel_random_walk   = 0;  // m/s³/√Hz
};

// Tunables (mirrors the vio_config row; plain values for the same reason).
struct VioTuning {
    int  num_pts        = 150;
    int  fast_threshold = 20;
    bool downsample     = true;
};

struct VioRunnerConfig {
    VioCameraConfig left;   // sensor_id 0
    VioCameraConfig right;  // sensor_id 1

    double sigma_w  = 0;  // gyro noise density
    double sigma_wb = 0;  // gyro random walk
    double sigma_a  = 0;  // accel noise density
    double sigma_ab = 0;  // accel random walk

    double calib_camimu_dt = 0;  // left camera's timeshift_cam_imu (s)

    int  num_pts            = 150;
    int  fast_threshold     = 20;
    bool downsample         = true;
    int  max_clone_size     = 11;
    int  num_opencv_threads = 2;

    double init_window_time = 1.0;
    double init_imu_thresh  = 0.5;  // FRC robots vibrate; tune on the rig
    double gravity_mag      = 9.81;
    double sigma_pix        = 1.0;
};

// Throws VioConfigError when either entry lacks the camera-IMU block or the
// noise terms are unset. Does NOT halve intrinsics for downsampling — that
// happens at the VioManagerOptions mapping (matching upstream's loader,
// which halves before constructing the camera models).
VioRunnerConfig build_runner_config(const gw::calib::CamchainEntry& left,
                                    const gw::calib::CamchainEntry& right,
                                    const VioImuNoise&              noise,
                                    const VioTuning&                tuning);

// Maps OpenVINS's marginal pose covariance (order [δθ, δp], left-JPL error:
// R_GtoI = (I − ⌊δθ⌋)·R̂, p = p̂ + δp with δp in the global frame) into our
// body-tangent convention [ω, t] with right perturbation T_true = T_est·Exp(ξ)
// (= TagPoseMeasurement::cov): ω = δθ identically (first order), t = R_GtoI·δp
// ⇒ Σ_body = J·P·Jᵀ with J = blkdiag(I₃, R_GtoI). Pure — unit-tested.
std::array<double, 36> cov_ov_to_body_tangent(const gw::apriltag::Mat3&    R_GtoI,
                                              const std::array<double, 36>& P_theta_p);

}  // namespace gw::vio
