#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "apriltag/pose_math.hpp"

// Public (std-only) types for the Phase 6 fusion engine. GTSAM never appears
// here — it lives behind FusionEngine's Pimpl (fusion_engine.cpp).
//
// All poses are T_field_robot in the WPILib field frame; all covariances use
// the project-wide convention: row-major 6×6 in tangent order
// [ωx ωy ωz, tx ty tz], RIGHT perturbation T_true = T_est·Exp(ξ) — exactly
// GTSAM Pose3's retract, so they flow into the graph unchanged.

namespace gw::fusion {

// Engine construction parameters (decoded from the fusion_config DB row by
// FusionSupervisor; unit comments match the column comments in database.cpp).
struct FusionParams {
    double  lag_s           = 2.0;
    int64_t min_state_dt_ns = 25'000'000;  // new graph state every ≥25 ms

    double tag_gate_chi2 = 22.46;  // χ²₆ @ 99.9%
    double tag_huber_k   = 1.345;
    double vio_huber_k   = 1.345;
    double odom_cauchy_k = 0.5;

    // Chassis-speeds noise: 1σ velocity error (m/s, rad/s); interval σ scales
    // linearly with dt (slip/scale error is correlated, not white).
    double odom_sigma_vx    = 0.05;
    double odom_sigma_vy    = 0.05;
    double odom_sigma_omega = 0.05;

    // VIO per-sample delta noise: random-walk units (rad/√s, m/√s). The
    // published VioOdometry.cov is NOT differenced (no cross-covariance
    // between consecutive poses is published, so differencing isn't
    // PSD-safe); it only health-gates samples.
    double vio_sigma_rot   = 0.01;
    double vio_sigma_trans = 0.01;

    double collision_inflation = 10.0;
    int    collision_window    = 20;
    double reinit_pos_std_m    = 1.0;

    // Unset ⇒ VIO ingestion disabled (tags already incorporate the transform
    // and chassis speeds are robot-frame, so fusion still runs).
    std::optional<gw::apriltag::Mat4> T_robot_imu;

    // Reserved Phase 7 seam: IMU preintegration as the VIO-unhealthy
    // fallback. Today's degradation path is tags + chassis speeds.
    // bool use_imu_preintegration = false;
};

// Engine → supervisor snapshot, refreshed after every smoother update.
struct FusedState {
    bool    initialized = false;
    int64_t t_ns        = 0;  // newest state time, sync controller clock

    gw::apriltag::Mat4     T_field_robot = gw::apriltag::mat4_identity();
    std::array<double, 36> cov{};  // newest-key marginal

    // Latest body twist (for output extrapolation).
    double vx_mps = 0.0, vy_mps = 0.0, omega_radps = 0.0;

    // 0 = uninitialized; else 255·clamp(1 − σ_pos/reinit_pos_std_m), halved
    // in collision mode, floor 1.
    uint8_t quality        = 0;
    bool    collision_mode = false;
};

struct FusionCounters {
    uint64_t tag_accepted       = 0;
    uint64_t tag_rejected_gate  = 0;
    uint64_t tag_rejected_clock = 0;  // clock_source != kSyncController
    uint64_t tag_rejected_stale = 0;  // older than the in-lag state deque

    uint64_t vio_fused_intervals  = 0;
    uint64_t vio_skipped_epoch    = 0;
    uint64_t vio_skipped_unhealthy = 0;

    uint64_t odom_fused_intervals = 0;
    uint64_t odom_stale           = 0;  // status_flags bit0 sub-intervals
    uint64_t odom_slip            = 0;  // status_flags bit1 sub-intervals

    uint64_t bridge_factors    = 0;  // constant-velocity gap fillers
    uint64_t states_created    = 0;
    uint64_t reinits           = 0;  // automatic only (manual reset excluded)
    uint64_t gate_reopens      = 0;  // collision-mode entries
    uint64_t update_exceptions = 0;

    double solve_ms_last = 0.0;
    double solve_ms_p95  = 0.0;  // over a 256-sample ring

    uint64_t lag_states         = 0;
    double   oldest_state_age_s = 0.0;
};

// Planar constant-twist extrapolation: T ⊕ Exp([0, 0, ω·dt, vx·dt, vy·dt, 0])
// (right perturbation — the twist is body-frame). Pure; used by the output
// thread and unit-tested directly.
gw::apriltag::Mat4 extrapolate_planar(const gw::apriltag::Mat4& T, double vx_mps,
                                      double vy_mps, double omega_radps,
                                      double dt_s);

}  // namespace gw::fusion
