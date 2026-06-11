#include "fusion/fusion_engine.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <limits>
#include <vector>

// Implementation notes (the design rules live in the Phase 6 plan and the
// header; the non-obvious ones):
//
//  - VIO between-factors are emitted LAZILY: OpenVINS publishes ~50–100 ms
//    after the pair's pulse stamp, so at state-creation time the covering
//    deltas haven't arrived yet. Deltas are buffered and an interval's
//    BetweenFactor is added once a delta endpoint reaches past the interval
//    end — adding factors to in-lag historical keys is precisely what the
//    fixed-lag smoother supports (tag priors attach to old keys the same
//    way).
//  - Per-sample VIO delta noise comes from config sigmas, NOT from
//    differencing published covariances: the consecutive-pose
//    cross-covariance is not published, so Σ_k − Σ_{k−1} is not PSD-safe and
//    Σ_k + Σ_{k−1} double-counts the shared (dominant) history. The
//    published covariance still health-gates samples.
//  - The in-lag state deque is trimmed at lag − 0.1 s — strictly tighter
//    than the smoother's marginalization horizon — so we never attach a
//    factor to (or query the marginal of) a key the smoother has dropped.

namespace gw::fusion {

namespace {

namespace ga = gw::apriltag;
using gtsam::symbol_shorthand::X;

gtsam::Pose3 to_pose3(const ga::Mat4& T) {
    gtsam::Matrix3 R;
    R << T[0][0], T[0][1], T[0][2],
         T[1][0], T[1][1], T[1][2],
         T[2][0], T[2][1], T[2][2];
    return gtsam::Pose3(gtsam::Rot3(R),
                        gtsam::Point3(T[0][3], T[1][3], T[2][3]));
}

ga::Mat4 from_pose3(const gtsam::Pose3& P) {
    const gtsam::Matrix4 m = P.matrix();
    ga::Mat4 T;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) T[r][c] = m(r, c);
    return T;
}

gtsam::Matrix6 to_matrix6(const std::array<double, 36>& a) {
    gtsam::Matrix6 m;
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) m(r, c) = a[r * 6 + c];
    return m;
}

std::array<double, 36> from_matrix6(const gtsam::Matrix6& m) {
    std::array<double, 36> a{};
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) a[r * 6 + c] = m(r, c);
    return a;
}

constexpr double kNsToS = 1e-9;

// Floors used when the current twist is unknown — generous on purpose (a
// 5.2 m/s robot with aggressive rotation).
constexpr double kTwistFloorRot   = 1.0;  // rad/s
constexpr double kTwistFloorTrans = 4.0;  // m/s

// Soft planarity sigmas for the chassis-speeds between (z / roll / pitch are
// unobserved by the drivetrain): tight enough to condition the 3D problem,
// loose enough that tags/VIO can bend it on ramps.
constexpr double kPlanarSigmaZ   = 0.02;  // m per interval
constexpr double kPlanarSigmaRot = 0.02;  // rad per interval

constexpr double kVioMaxPosStd  = 2.0;  // published-cov health gate, m
constexpr int    kSolveRingSize = 256;

}  // namespace

struct FusionEngine::Impl {
    explicit Impl(const FusionParams& p) : p_(p) {}

    FusionParams   p_;
    FusedState     state_;
    FusionCounters c_;

    std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;

    // --- graph bookkeeping ---------------------------------------------------
    struct StateRec {
        int64_t  t_ns = 0;
        uint64_t key  = 0;
    };
    std::deque<StateRec> states_;  // in-lag, oldest → newest
    bool     initialized_ = false;
    int64_t  t0_ns_       = 0;
    uint64_t next_key_    = 0;

    // --- init collection -------------------------------------------------------
    std::vector<gw::apriltag::TagPoseMeasurement> init_tags_;

    // --- chassis-speeds buffer --------------------------------------------------
    struct OdomRec {
        int64_t  t_ns = 0;
        double   vx = 0, vy = 0, omega = 0;
        uint16_t flags = 0;
    };
    std::deque<OdomRec> odom_buf_;
    bool    have_twist_ = false;
    OdomRec last_twist_;  // newest sample (bridge + extrapolation fallback)

    // --- VIO delta buffer ----------------------------------------------------------
    struct VioDelta {
        int64_t  t_begin_ns = 0, t_end_ns = 0;
        ga::Mat4 delta_robot = ga::mat4_identity();
        ga::Mat6 cov{};
    };
    std::deque<VioDelta> vio_deltas_;
    bool                 have_vio_prev_ = false;
    gw::vio::VioOdometry vio_prev_;
    uint64_t             vio_done_key_ = 0;  // intervals ending ≤ this key handled

    // --- collision monitor --------------------------------------------------------
    std::deque<bool> gate_window_;  // true = would-pass
    bool             collision_mode_     = false;
    int              consecutive_passes_ = 0;
    int64_t          collision_since_ns_ = 0;

    // --- solve timing ----------------------------------------------------------------
    std::array<double, kSolveRingSize> solve_ring_{};
    size_t solve_n_ = 0;

    // ---------------------------------------------------------------------------
    void feed_tag(const gw::apriltag::TagPoseMeasurement& m);
    void feed_vio(const gw::vio::VioOdometry& m);
    void feed_odom(const gw::ChassisSpeeds& m);
    void reinit(bool automatic);

    void try_init();
    void create_state(int64_t t_ns);
    bool update(const gtsam::NonlinearFactorGraph& nfg, const gtsam::Values& vals,
                const gtsam::FixedLagSmoother::KeyTimestampMap& stamps);
    void flush_vio_intervals();
    void attach_tag(const gw::apriltag::TagPoseMeasurement& m, uint64_t key,
                    int64_t t_key_ns);
    void note_gate_result(bool would_pass, int64_t t_ns);
    void refresh_state();
    void trim_states(int64_t newest_t_ns);

    ga::Mat6 motion_inflation(double dt_s) const;
    gtsam::SharedNoiseModel odom_noise(double dt_s, double inflate) const;

    struct OdomDelta {
        gtsam::Pose3 delta;
        double       inflate      = 1.0;  // stale/slip multiplier
        bool         from_samples = false;
    };
    OdomDelta integrate_odom(int64_t t_from_ns, int64_t t_to_ns);
};

// ---------------------------------------------------------------------------
// helpers

ga::Mat6 FusionEngine::Impl::motion_inflation(double dt_s) const {
    const double w = std::max(std::abs(state_.omega_radps), kTwistFloorRot);
    const double v = std::max({std::abs(state_.vx_mps), std::abs(state_.vy_mps),
                               kTwistFloorTrans});
    ga::Mat6 out{};
    for (int i = 0; i < 3; ++i) out[i * 6 + i] = (dt_s * w) * (dt_s * w);
    for (int i = 3; i < 6; ++i) out[i * 6 + i] = (dt_s * v) * (dt_s * v);
    return out;
}

gtsam::SharedNoiseModel FusionEngine::Impl::odom_noise(double dt_s,
                                                       double inflate) const {
    const double infl =
        inflate * (collision_mode_ ? p_.collision_inflation : 1.0);
    gtsam::Vector6 sig;
    // Tangent order [ω, v]; z / roll / pitch get the soft planarity sigmas.
    sig << kPlanarSigmaRot * infl, kPlanarSigmaRot * infl,
        std::max(p_.odom_sigma_omega * dt_s, 1e-6) * infl,
        std::max(p_.odom_sigma_vx * dt_s, 1e-6) * infl,
        std::max(p_.odom_sigma_vy * dt_s, 1e-6) * infl, kPlanarSigmaZ * infl;
    return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(p_.odom_cauchy_k),
        gtsam::noiseModel::Diagonal::Sigmas(sig));
}

FusionEngine::Impl::OdomDelta FusionEngine::Impl::integrate_odom(
        int64_t t_from_ns, int64_t t_to_ns) {
    OdomDelta out;
    gtsam::Pose3 acc;
    double  inflate = 1.0;
    int64_t cursor  = t_from_ns;
    double  vx = have_twist_ ? last_twist_.vx : 0.0;
    double  vy = have_twist_ ? last_twist_.vy : 0.0;
    double  om = have_twist_ ? last_twist_.omega : 0.0;

    const auto step = [&](int64_t until_ns) {
        const double dt = static_cast<double>(until_ns - cursor) * kNsToS;
        if (dt <= 0) return;
        gtsam::Vector6 xi;
        xi << 0, 0, om * dt, vx * dt, vy * dt, 0;
        acc    = acc * gtsam::Pose3::Expmap(xi);
        cursor = until_ns;
    };

    bool used_sample = false;
    while (!odom_buf_.empty() && odom_buf_.front().t_ns <= t_to_ns) {
        const OdomRec s = odom_buf_.front();
        odom_buf_.pop_front();
        if (s.t_ns > t_from_ns) {
            step(s.t_ns);
            used_sample = true;
        }
        if (s.flags & 0x01) { inflate = std::max(inflate, 2.0); ++c_.odom_stale; }
        if (s.flags & 0x02) { inflate = std::max(inflate, 4.0); ++c_.odom_slip; }
        vx = s.vx; vy = s.vy; om = s.omega;
        used_sample = true;  // even a boundary sample seeds the closing twist
    }
    step(t_to_ns);

    out.delta        = acc;
    out.inflate      = inflate;
    out.from_samples = used_sample;
    return out;
}

void FusionEngine::Impl::trim_states(int64_t newest_t_ns) {
    // Strictly inside the smoother's horizon — see file comment.
    const int64_t horizon =
        newest_t_ns - static_cast<int64_t>((p_.lag_s - 0.1) * 1e9);
    while (states_.size() > 1 && states_.front().t_ns < horizon) {
        states_.pop_front();
    }
}

bool FusionEngine::Impl::update(
        const gtsam::NonlinearFactorGraph& nfg, const gtsam::Values& vals,
        const gtsam::FixedLagSmoother::KeyTimestampMap& stamps) {
    const auto start = std::chrono::steady_clock::now();
    bool ok = true;
    try {
        smoother_->update(nfg, vals, stamps);
    } catch (const std::exception& e) {
        ++c_.update_exceptions;
        std::cerr << "FusionEngine: smoother update failed (" << e.what()
                  << ") — reinitializing\n";
        reinit(true);
        ok = false;
    }
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - start)
                          .count();
    solve_ring_[solve_n_ % kSolveRingSize] = ms;
    ++solve_n_;
    c_.solve_ms_last = ms;
    const size_t n = std::min(solve_n_, static_cast<size_t>(kSolveRingSize));
    std::array<double, kSolveRingSize> sorted = solve_ring_;
    std::sort(sorted.begin(), sorted.begin() + static_cast<ptrdiff_t>(n));
    c_.solve_ms_p95 = sorted[static_cast<size_t>(0.95 * static_cast<double>(n - 1))];
    if (ok) refresh_state();
    return ok;
}

void FusionEngine::Impl::refresh_state() {
    if (!initialized_ || states_.empty()) return;
    const StateRec newest = states_.back();
    try {
        const auto est =
            smoother_->calculateEstimate<gtsam::Pose3>(X(newest.key));
        const gtsam::Matrix6 S = smoother_->marginalCovariance(X(newest.key));

        state_.t_ns          = newest.t_ns;
        state_.T_field_robot = from_pose3(est);
        state_.cov           = from_matrix6(S);

        // Twist from the last two smoothed states (captures every source);
        // odometry fallback when only one state exists.
        if (states_.size() >= 2) {
            const StateRec prev = states_[states_.size() - 2];
            const double   dt =
                static_cast<double>(newest.t_ns - prev.t_ns) * kNsToS;
            if (dt > 1e-4) {
                const auto pe =
                    smoother_->calculateEstimate<gtsam::Pose3>(X(prev.key));
                const gtsam::Vector6 xi = gtsam::Pose3::Logmap(pe.between(est));
                state_.omega_radps = xi(2) / dt;
                state_.vx_mps      = xi(3) / dt;
                state_.vy_mps      = xi(4) / dt;
            }
        } else if (have_twist_) {
            state_.vx_mps      = last_twist_.vx;
            state_.vy_mps      = last_twist_.vy;
            state_.omega_radps = last_twist_.omega;
        }

        const double pos_var = std::max({S(3, 3), S(4, 4), S(5, 5)});
        const double pos_std = std::sqrt(std::max(pos_var, 0.0));
        if (!std::isfinite(pos_std) ||
            !std::isfinite(state_.T_field_robot[0][3])) {
            std::cerr << "FusionEngine: NaN in estimate — reinitializing\n";
            reinit(true);
            return;
        }
        if (pos_std > p_.reinit_pos_std_m) {
            std::cerr << "FusionEngine: position std " << pos_std
                      << " m exceeds the reinit threshold — reinitializing\n";
            reinit(true);
            return;
        }

        double q = std::clamp(1.0 - pos_std / p_.reinit_pos_std_m, 0.0, 1.0);
        if (collision_mode_) q *= 0.5;
        state_.quality        = static_cast<uint8_t>(std::max(1.0, q * 255.0));
        state_.collision_mode = collision_mode_;
        state_.initialized    = true;

        c_.lag_states = states_.size();
        c_.oldest_state_age_s =
            static_cast<double>(newest.t_ns - states_.front().t_ns) * kNsToS;
    } catch (const std::exception& e) {
        ++c_.update_exceptions;
        std::cerr << "FusionEngine: estimate query failed (" << e.what()
                  << ") — reinitializing\n";
        reinit(true);
    }
}

// ---------------------------------------------------------------------------
// state creation + odom

void FusionEngine::Impl::create_state(int64_t t_ns) {
    const StateRec prev = states_.back();
    const uint64_t k    = next_key_++;
    const double   dt_s = static_cast<double>(t_ns - prev.t_ns) * kNsToS;

    gtsam::NonlinearFactorGraph nfg;

    const OdomDelta od = integrate_odom(prev.t_ns, t_ns);
    double bridge_inflate = 1.0;
    if (!od.from_samples) {
        // No odom covering the interval, and VIO can't help a brand-new key
        // (it arrives lazily): a constant-velocity bridge keeps the chain
        // connected — the invariant that prevents indeterminate systems.
        bridge_inflate = 10.0;
        ++c_.bridge_factors;
    } else {
        ++c_.odom_fused_intervals;
    }
    nfg.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        X(prev.key), X(k), od.delta,
        odom_noise(dt_s, od.inflate * bridge_inflate));

    gtsam::Pose3 prev_est;
    try {
        prev_est = smoother_->calculateEstimate<gtsam::Pose3>(X(prev.key));
    } catch (const std::exception&) {
        prev_est = to_pose3(state_.T_field_robot);
    }

    gtsam::Values vals;
    vals.insert(X(k), prev_est * od.delta);
    gtsam::FixedLagSmoother::KeyTimestampMap stamps;
    stamps[X(k)] = static_cast<double>(t_ns - t0_ns_) * kNsToS;

    states_.push_back({t_ns, k});
    ++c_.states_created;
    trim_states(t_ns);

    if (update(nfg, vals, stamps)) flush_vio_intervals();
}

void FusionEngine::Impl::feed_odom(const gw::ChassisSpeeds& m) {
    const int64_t t = static_cast<int64_t>(m.t_ns);
    const OdomRec rec{t, m.vx_mps, m.vy_mps, m.omega_radps, m.status_flags};
    have_twist_ = true;
    last_twist_ = rec;
    if (!initialized_) return;  // accumulators prime only after init

    odom_buf_.push_back(rec);
    // Bound the buffer against state-creation droughts: keep ~4 s.
    while (!odom_buf_.empty() &&
           odom_buf_.front().t_ns < t - 4'000'000'000ll) {
        odom_buf_.pop_front();
    }
    if (t >= states_.back().t_ns + p_.min_state_dt_ns) create_state(t);
}

// ---------------------------------------------------------------------------
// VIO

void FusionEngine::Impl::feed_vio(const gw::vio::VioOdometry& m) {
    if (!p_.T_robot_imu) return;  // ingestion disabled (supervisor reports why)
    if (!m.initialized) {
        have_vio_prev_ = false;
        return;
    }
    // Published-cov health gate (the one legitimate use of the absolute cov).
    const double pos_var =
        std::max({m.cov[3 * 6 + 3], m.cov[4 * 6 + 4], m.cov[5 * 6 + 5]});
    if (pos_var > kVioMaxPosStd * kVioMaxPosStd) {
        ++c_.vio_skipped_unhealthy;
        have_vio_prev_ = false;
        return;
    }
    if (have_vio_prev_ && m.epoch != vio_prev_.epoch) {
        ++c_.vio_skipped_epoch;
        have_vio_prev_ = false;
    }
    if (!have_vio_prev_) {
        vio_prev_      = m;
        have_vio_prev_ = true;
        return;
    }

    const ga::Mat4& T_ri = *p_.T_robot_imu;
    const ga::Mat4 d_imu =
        ga::mat4_mul(ga::mat4_inverse_se3(vio_prev_.T_odom_imu), m.T_odom_imu);
    const ga::Mat4 d_robot =
        ga::mat4_mul(ga::mat4_mul(T_ri, d_imu), ga::mat4_inverse_se3(T_ri));

    const double dt_s = static_cast<double>(m.t_ns - vio_prev_.t_ns) * kNsToS;
    VioDelta d;
    d.t_begin_ns  = vio_prev_.t_ns;
    d.t_end_ns    = m.t_ns;
    d.delta_robot = d_robot;
    // Random-walk per-sample noise in the robot frame (see file comment).
    const double rv = std::max(dt_s, 1e-4);
    for (int i = 0; i < 3; ++i)
        d.cov[i * 6 + i] = p_.vio_sigma_rot * p_.vio_sigma_rot * rv;
    for (int i = 3; i < 6; ++i)
        d.cov[i * 6 + i] = p_.vio_sigma_trans * p_.vio_sigma_trans * rv;

    vio_prev_ = m;
    if (!initialized_) return;

    vio_deltas_.push_back(std::move(d));
    while (vio_deltas_.size() > 512) vio_deltas_.pop_front();
    flush_vio_intervals();
}

void FusionEngine::Impl::flush_vio_intervals() {
    if (!initialized_ || vio_deltas_.empty() || states_.size() < 2) return;
    const int64_t vio_horizon = vio_deltas_.back().t_end_ns;

    // Walk consecutive in-lag state pairs whose interval is fully covered (a
    // delta endpoint exists at/after the interval end) and which haven't
    // been handled yet.
    for (size_t i = 1; i < states_.size(); ++i) {
        const StateRec a = states_[i - 1];
        const StateRec b = states_[i];
        if (b.key <= vio_done_key_) continue;
        if (vio_horizon < b.t_ns) break;  // not covered yet — wait

        // Compose deltas with endpoint in (t_a, t_b].
        ga::Mat4 acc = ga::mat4_identity();
        ga::Mat6 cov{};
        int64_t  first_begin = 0, last_end = 0;
        bool     any = false;
        for (const auto& d : vio_deltas_) {
            if (d.t_end_ns <= a.t_ns || d.t_end_ns > b.t_ns) continue;
            if (!any) first_begin = d.t_begin_ns;
            last_end = d.t_end_ns;
            // Right-perturbation composition:
            //   cov_C = Ad(B⁻¹)·Σ_A·Ad(B⁻¹)ᵀ + Σ_B for C = A·B.
            const ga::Mat6 ad_binv =
                ga::adjoint_se3(ga::mat4_inverse_se3(d.delta_robot));
            cov = ga::congruence(ad_binv, cov);
            for (int j = 0; j < 36; ++j) cov[j] += d.cov[j];
            acc = ga::mat4_mul(acc, d.delta_robot);
            any = true;
        }
        vio_done_key_ = b.key;
        if (!any) continue;  // odom covered the interval

        // Boundary mismatch (≤ one VIO period at each end) → honest motion
        // inflation.
        const double uncovered =
            static_cast<double>(std::max<int64_t>(first_begin - a.t_ns, 0) +
                                std::max<int64_t>(b.t_ns - last_end, 0)) *
            kNsToS;
        if (uncovered > 0) {
            const ga::Mat6 infl = motion_inflation(uncovered);
            for (int j = 0; j < 36; ++j) cov[j] += infl[j];
        }

        const double infl = collision_mode_ ? p_.collision_inflation : 1.0;
        const gtsam::Matrix6 cov6 = to_matrix6(cov) * (infl * infl);
        auto noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(p_.vio_huber_k),
            gtsam::noiseModel::Gaussian::Covariance(cov6));

        gtsam::NonlinearFactorGraph nfg;
        nfg.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(a.key), X(b.key), to_pose3(acc), noise);
        ++c_.vio_fused_intervals;
        if (!update(nfg, {}, {})) return;  // reinit happened — stop
    }

    // Deltas are consumed once the newest handled interval passes them.
    int64_t consumed_until = 0;
    for (size_t i = 1; i < states_.size(); ++i) {
        if (states_[i].key <= vio_done_key_) consumed_until = states_[i].t_ns;
    }
    while (!vio_deltas_.empty() &&
           vio_deltas_.front().t_end_ns <= consumed_until) {
        vio_deltas_.pop_front();
    }
}

// ---------------------------------------------------------------------------
// tags

void FusionEngine::Impl::note_gate_result(bool would_pass, int64_t t_ns) {
    gate_window_.push_back(would_pass);
    while (gate_window_.size() >
           static_cast<size_t>(std::max(p_.collision_window, 1))) {
        gate_window_.pop_front();
    }
    consecutive_passes_ = would_pass ? consecutive_passes_ + 1 : 0;

    const size_t seen = gate_window_.size();
    const size_t rejected = static_cast<size_t>(
        std::count(gate_window_.begin(), gate_window_.end(), false));

    if (!collision_mode_) {
        if (seen >= static_cast<size_t>(p_.collision_window) / 2 &&
            rejected * 2 > seen) {
            collision_mode_     = true;
            collision_since_ns_ = t_ns;
            ++c_.gate_reopens;
            std::cerr << "FusionEngine: sustained tag disagreement — entering "
                         "collision mode (gate open, VIO/odom inflated)\n";
        }
    } else {
        const bool calm = consecutive_passes_ >= 10 || rejected * 5 < seen;
        if (calm) {
            collision_mode_ = false;
            std::cerr << "FusionEngine: collision mode cleared\n";
        } else if (t_ns - collision_since_ns_ > 5'000'000'000ll) {
            std::cerr << "FusionEngine: collision mode unresolved for >5 s — "
                         "reinitializing\n";
            reinit(true);
        }
    }
}

void FusionEngine::Impl::attach_tag(const gw::apriltag::TagPoseMeasurement& m,
                                    uint64_t key, int64_t t_key_ns) {
    // Timing-mismatch inflation (honest: motion during the offset is exactly
    // the attachment error).
    const double dt_s =
        std::abs(static_cast<double>(m.t_ns - t_key_ns)) * kNsToS;
    ga::Mat6 cov{};
    std::copy(m.cov.begin(), m.cov.end(), cov.begin());
    if (dt_s > 0) {
        const ga::Mat6 infl = motion_inflation(dt_s);
        for (int j = 0; j < 36; ++j) cov[j] += infl[j];
    }
    const gtsam::Matrix6 cov6 = to_matrix6(cov);
    const gtsam::Pose3   meas = to_pose3(m.T_field_robot);

    // Gate — always evaluated (it drives the collision monitor); enforced
    // only outside collision mode.
    bool would_pass = true;
    try {
        const auto pred = smoother_->calculateEstimate<gtsam::Pose3>(X(key));
        const gtsam::Matrix6 S = smoother_->marginalCovariance(X(key)) + cov6;
        const gtsam::Vector6 r = gtsam::Pose3::Logmap(pred.between(meas));
        const double d2 = r.transpose() * S.ldlt().solve(r);
        would_pass      = d2 < p_.tag_gate_chi2;
    } catch (const std::exception&) {
        ++c_.tag_rejected_stale;
        return;
    }
    note_gate_result(would_pass, m.t_ns);
    if (!initialized_) return;  // note_gate_result may have reinit'd
    if (!would_pass && !collision_mode_) {
        ++c_.tag_rejected_gate;
        return;
    }

    auto noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(p_.tag_huber_k),
        gtsam::noiseModel::Gaussian::Covariance(cov6));
    gtsam::NonlinearFactorGraph nfg;
    nfg.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(key), meas, noise);
    ++c_.tag_accepted;
    update(nfg, {}, {});
}

void FusionEngine::Impl::try_init() {
    if (init_tags_.size() < 5) return;
    while (init_tags_.size() > 5) init_tags_.erase(init_tags_.begin());
    if (init_tags_.back().t_ns - init_tags_.front().t_ns > 1'000'000'000ll) {
        init_tags_.erase(init_tags_.begin());
        return;
    }

    // Geometric medoid: the sample minimizing summed log-distance to the
    // others — robust to one bad measurement in the opening burst.
    size_t best     = 0;
    double best_sum = std::numeric_limits<double>::max();
    for (size_t i = 0; i < init_tags_.size(); ++i) {
        double sum = 0;
        for (size_t j = 0; j < init_tags_.size(); ++j) {
            if (i == j) continue;
            const ga::Vec6 xi = ga::log_se3(ga::mat4_mul(
                ga::mat4_inverse_se3(init_tags_[i].T_field_robot),
                init_tags_[j].T_field_robot));
            double n = 0;
            for (double v : xi) n += v * v;
            sum += std::sqrt(n);
        }
        if (sum < best_sum) {
            best_sum = sum;
            best     = i;
        }
    }
    const auto seed = init_tags_[best];

    gtsam::ISAM2Params ip;
    ip.factorization         = gtsam::ISAM2Params::QR;  // robustness > speed
    ip.findUnusedFactorSlots = true;  // required with fixed-lag removals
    ip.relinearizeThreshold  = 0.01;
    ip.relinearizeSkip       = 1;
    smoother_ =
        std::make_unique<gtsam::IncrementalFixedLagSmoother>(p_.lag_s, ip);

    t0_ns_    = seed.t_ns;
    next_key_ = 0;
    const uint64_t k = next_key_++;
    states_.clear();
    states_.push_back({seed.t_ns, k});
    vio_done_key_ = k;

    std::array<double, 36> seed_cov = seed.cov;
    for (auto& v : seed_cov) v *= 2.0;  // the medoid is one sample, not a mean
    gtsam::NonlinearFactorGraph nfg;
    nfg.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        X(k), to_pose3(seed.T_field_robot),
        gtsam::noiseModel::Gaussian::Covariance(to_matrix6(seed_cov)));
    gtsam::Values vals;
    vals.insert(X(k), to_pose3(seed.T_field_robot));
    gtsam::FixedLagSmoother::KeyTimestampMap stamps;
    stamps[X(k)] = 0.0;

    initialized_ = true;
    ++c_.states_created;
    ++c_.tag_accepted;
    if (!update(nfg, vals, stamps)) return;
    init_tags_.clear();
    std::cerr << "FusionEngine: initialized from tag medoid\n";
}

void FusionEngine::Impl::feed_tag(const gw::apriltag::TagPoseMeasurement& m) {
    if (m.clock_source != gw::apriltag::TagPoseMeasurement::Clock::kTeensy) {
        ++c_.tag_rejected_clock;
        return;
    }
    if (!initialized_) {
        init_tags_.push_back(m);
        try_init();
        return;
    }

    const int64_t newest_t = states_.back().t_ns;
    if (m.t_ns >= newest_t + p_.min_state_dt_ns) {
        // Newer than every state — the tag creates one and attaches exactly.
        create_state(m.t_ns);
        if (!initialized_) return;  // create_state may have reinit'd
        attach_tag(m, states_.back().key, states_.back().t_ns);
        return;
    }

    // Nearest in-lag state within the attach window.
    const StateRec* best    = nullptr;
    int64_t         best_dt = p_.min_state_dt_ns;
    for (const auto& s : states_) {
        const int64_t dt = std::abs(s.t_ns - m.t_ns);
        if (dt <= best_dt) {
            best_dt = dt;
            best    = &s;
        }
    }
    if (!best) {
        ++c_.tag_rejected_stale;
        return;
    }
    attach_tag(m, best->key, best->t_ns);
}

// ---------------------------------------------------------------------------

void FusionEngine::Impl::reinit(bool automatic) {
    smoother_.reset();
    states_.clear();
    odom_buf_.clear();
    vio_deltas_.clear();
    have_vio_prev_ = false;
    init_tags_.clear();
    gate_window_.clear();
    collision_mode_     = false;
    consecutive_passes_ = 0;
    initialized_        = false;
    vio_done_key_       = 0;

    state_.initialized    = false;
    state_.quality        = 0;
    state_.collision_mode = false;
    if (automatic) ++c_.reinits;
}

// ---------------------------------------------------------------------------
// public surface

FusionEngine::FusionEngine(const FusionParams& params)
    : impl_(std::make_unique<Impl>(params)) {}
FusionEngine::~FusionEngine() = default;

void FusionEngine::feed_tag(const gw::apriltag::TagPoseMeasurement& m) {
    impl_->feed_tag(m);
}
void FusionEngine::feed_vio(const gw::vio::VioOdometry& m) {
    impl_->feed_vio(m);
}
void FusionEngine::feed_odom(const gw::ChassisSpeeds& m) {
    impl_->feed_odom(m);
}

const FusedState&     FusionEngine::state() const { return impl_->state_; }
const FusionCounters& FusionEngine::counters() const { return impl_->c_; }

void FusionEngine::reset() { impl_->reinit(false); }

// ---------------------------------------------------------------------------

gw::apriltag::Mat4 extrapolate_planar(const gw::apriltag::Mat4& T,
                                      double vx_mps, double vy_mps,
                                      double omega_radps, double dt_s) {
    return gw::apriltag::retract(
        T, gw::apriltag::Vec6{0, 0, omega_radps * dt_s, vx_mps * dt_s,
                              vy_mps * dt_s, 0});
}

}  // namespace gw::fusion
