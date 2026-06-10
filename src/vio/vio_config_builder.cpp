#include "vio/vio_config_builder.hpp"

namespace gw::vio {

namespace {

VioCameraConfig to_camera_config(const gw::calib::CamchainEntry& entry,
                                 const char*                     which) {
    if (!entry.imu) {
        throw VioConfigError(std::string(which) +
                             " camera has no camera-IMU extrinsics block");
    }
    VioCameraConfig out;
    out.equidistant = entry.intrinsics.distortion_model == "equidistant";
    out.fxfycxcy    = entry.intrinsics.intrinsics;
    out.dist        = entry.intrinsics.distortion_coeffs;
    out.wh          = entry.intrinsics.resolution;
    out.T_cam_imu   = entry.imu->T_cam_imu;
    return out;
}

}  // namespace

VioRunnerConfig build_runner_config(const gw::calib::CamchainEntry& left,
                                    const gw::calib::CamchainEntry& right,
                                    const VioImuNoise&              noise,
                                    const VioTuning&                tuning) {
    if (noise.gyro_noise_density <= 0 || noise.gyro_random_walk <= 0 ||
        noise.accel_noise_density <= 0 || noise.accel_random_walk <= 0) {
        throw VioConfigError("IMU noise model is not configured");
    }

    VioRunnerConfig cfg;
    cfg.left  = to_camera_config(left, "left");
    cfg.right = to_camera_config(right, "right");

    cfg.sigma_w  = noise.gyro_noise_density;
    cfg.sigma_wb = noise.gyro_random_walk;
    cfg.sigma_a  = noise.accel_noise_density;
    cfg.sigma_ab = noise.accel_random_walk;

    // Both cameras share the Teensy pulse clock, so the two Kalibr
    // timeshifts are ≈ equal; OpenVINS takes one offset — use the left's.
    cfg.calib_camimu_dt = left.imu->timeshift_cam_imu;

    cfg.num_pts        = tuning.num_pts;
    cfg.fast_threshold = tuning.fast_threshold;
    cfg.downsample     = tuning.downsample;
    return cfg;
}

std::array<double, 36> cov_ov_to_body_tangent(
    const gw::apriltag::Mat3&     R_GtoI,
    const std::array<double, 36>& P) {
    // J = blkdiag(I3, R_GtoI);  Σ = J·P·Jᵀ.
    // Block structure: Σθθ = Pθθ, Σθt = Pθp·Rᵀ, Σtθ = R·Ppθ, Σtt = R·Ppp·Rᵀ.
    const auto p = [&](int r, int c) { return P[r * 6 + c]; };

    std::array<double, 36> out{};
    // Rotation block unchanged.
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) out[r * 6 + c] = p(r, c);

    // Σθt = Pθp · Rᵀ   (out[r][3+c] = Σ_k Pθp[r][k] · R[c][k])
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double v = 0;
            for (int k = 0; k < 3; ++k) v += p(r, 3 + k) * R_GtoI[c][k];
            out[r * 6 + (3 + c)] = v;
        }
    }
    // Σtθ = R · Ppθ
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double v = 0;
            for (int k = 0; k < 3; ++k) v += R_GtoI[r][k] * p(3 + k, c);
            out[(3 + r) * 6 + c] = v;
        }
    }
    // Σtt = R · Ppp · Rᵀ
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double v = 0;
            for (int k = 0; k < 3; ++k)
                for (int m = 0; m < 3; ++m)
                    v += R_GtoI[r][k] * p(3 + k, 3 + m) * R_GtoI[c][m];
            out[(3 + r) * 6 + (3 + c)] = v;
        }
    }
    return out;
}

}  // namespace gw::vio
