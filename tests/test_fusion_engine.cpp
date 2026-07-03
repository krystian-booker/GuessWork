#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "apriltag/pose_math.hpp"
#include "fusion/fusion_engine.hpp"
#include "fusion/fusion_types.hpp"

// Hardware-free simulation suite: a figure-8 trajectory with analytic ground
// truth drives fabricated tag / VIO / chassis-speeds measurements (known
// noise, deterministic LCG) through the engine in arrival order — tags
// arrive 30 ms late, VIO 60 ms late, matching the real pipeline's latency
// ordering. GTSAM is exercised for real through the engine's std-only API.

namespace gw::fusion {

namespace {

namespace ga = gw::apriltag;

// Deterministic noise (same approach as test_rio_clock_sync.cpp).
struct Lcg {
    uint64_t state = 0x9E3779B97F4A7C15ull;
    uint64_t next() {
        state = state * 6364136223846793005ull + 1442695040888963407ull;
        return state >> 33;
    }
    double uniform() {  // [0, 1)
        return static_cast<double>(next()) / 2147483648.0;
    }
    double gauss() {  // Irwin–Hall ≈ N(0,1)
        double s = 0;
        for (int i = 0; i < 12; ++i) s += uniform();
        return s - 6.0;
    }
};

constexpr int64_t kT0Ns = 1'000'000'000'000ll;  // arbitrary Teensy-clock epoch

ga::Mat4 planar_pose(double x, double y, double theta) {
    const double c = std::cos(theta), s = std::sin(theta);
    return ga::Mat4{{{c, -s, 0, x}, {s, c, 0, y}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
}

double yaw_of(const ga::Mat4& T) { return std::atan2(T[1][0], T[0][0]); }

// Figure-8 with heading along the velocity vector.
struct Sim {
    double A = 2.0, B = 1.0, w = 0.5;

    // Tags see this (teleport applied); relative sources never do.
    double shift_x = 0, shift_y = 0;
    double shift_after_s = 1e18;

    ga::Mat4 truth_unshifted(double t) const {
        const double x  = A * std::sin(w * t);
        const double y  = B * std::sin(2 * w * t);
        const double dx = A * w * std::cos(w * t);
        const double dy = 2 * B * w * std::cos(2 * w * t);
        return planar_pose(x, y, std::atan2(dy, dx));
    }

    ga::Mat4 truth(double t) const {
        ga::Mat4 T = truth_unshifted(t);
        if (t >= shift_after_s) {
            T[0][3] += shift_x;
            T[1][3] += shift_y;
        }
        return T;
    }

    // Body twist over [t, t+dt) of the unshifted trajectory — what wheel
    // encoders / VIO report (a constant teleport offset cancels in deltas).
    void body_twist(double t, double dt, double& vx, double& vy,
                    double& omega) const {
        const ga::Vec6 xi = ga::log_se3(ga::mat4_mul(
            ga::mat4_inverse_se3(truth_unshifted(t)), truth_unshifted(t + dt)));
        omega = xi[2] / dt;
        vx    = xi[3] / dt;
        vy    = xi[4] / dt;
    }
};

constexpr double kTagSigmaRot   = 0.01;  // rad
constexpr double kTagSigmaTrans = 0.02;  // m

gw::apriltag::TagPoseMeasurement make_tag(const Sim& sim, double t, Lcg& rng,
                                          bool outlier = false) {
    gw::apriltag::TagPoseMeasurement m;
    m.t_ns         = kT0Ns + static_cast<int64_t>(t * 1e9);
    m.clock_source = gw::apriltag::TagPoseMeasurement::Clock::kTeensy;
    m.n_tags       = 2;

    ga::Vec6 xi{};
    for (int i = 0; i < 3; ++i) xi[i] = kTagSigmaRot * rng.gauss();
    for (int i = 3; i < 6; ++i) xi[i] = kTagSigmaTrans * rng.gauss();
    m.T_field_robot = ga::retract(sim.truth(t), xi);
    if (outlier) {
        // 1–3 m gross error in a random direction.
        const double r   = 1.0 + 2.0 * rng.uniform();
        const double ang = 2 * M_PI * rng.uniform();
        m.T_field_robot[0][3] += r * std::cos(ang);
        m.T_field_robot[1][3] += r * std::sin(ang);
    }
    for (int i = 0; i < 3; ++i)
        m.cov[i * 6 + i] = kTagSigmaRot * kTagSigmaRot;
    for (int i = 3; i < 6; ++i)
        m.cov[i * 6 + i] = kTagSigmaTrans * kTagSigmaTrans;
    return m;
}

gw::ChassisSpeeds make_odom(const Sim& sim, double t, Lcg& rng) {
    double vx, vy, omega;
    sim.body_twist(t, 0.01, vx, vy, omega);
    gw::ChassisSpeeds s;
    s.t_ns        = static_cast<uint64_t>(kT0Ns + static_cast<int64_t>(t * 1e9));
    s.vx_mps      = static_cast<float>(vx + 0.01 * rng.gauss());
    s.vy_mps      = static_cast<float>(vy + 0.01 * rng.gauss());
    s.omega_radps = static_cast<float>(omega + 0.01 * rng.gauss());
    return s;
}

// Fixed nontrivial IMU mounting for the VIO path.
const ga::Mat4 kTRobotImu = ga::exp_se3({0.1, -0.05, 0.7, 0.2, 0.1, 0.3});

gw::vio::VioOdometry make_vio(const Sim& sim, double t, Lcg& rng,
                              uint64_t epoch, const ga::Mat4& T_field_odom) {
    gw::vio::VioOdometry m;
    m.t_ns        = kT0Ns + static_cast<int64_t>(t * 1e9);
    m.epoch       = epoch;
    m.initialized = true;
    m.tracked_features = 50;

    // T_odom_imu = T_odom_field · T_field_robot · T_robot_imu, with small
    // white per-sample noise (deltas inherit √2 of it).
    const ga::Mat4 T_odom_robot = ga::mat4_mul(
        ga::mat4_inverse_se3(T_field_odom), sim.truth_unshifted(t));
    ga::Vec6 xi{};
    for (int i = 0; i < 3; ++i) xi[i] = 0.001 * rng.gauss();
    for (int i = 3; i < 6; ++i) xi[i] = 0.002 * rng.gauss();
    m.T_odom_imu = ga::retract(ga::mat4_mul(T_odom_robot, kTRobotImu), xi);
    for (int i = 0; i < 6; ++i) m.cov[i * 6 + i] = 1e-4;  // healthy
    return m;
}

// ---------------------------------------------------------------------------
// Harness: builds the merged event stream and tracks pose error vs truth.

struct Scenario {
    double duration_s    = 12.0;
    double tag_period    = 1.0 / 30;
    double odom_period   = 0.01;
    double vio_period    = 1.0 / 30;
    double tag_latency   = 0.03;
    double vio_latency   = 0.06;
    bool   with_vio      = true;
    double vio_stop_at   = 1e18;  // VIO death
    double odom_stop_at  = 1e18;  // chassis-speeds death
    double vio_epoch_at  = 1e18;  // epoch reset (new odom frame)
    double tags_off_from = 1e18, tags_off_to = -1.0;  // tag drought window
    int    tag_outlier_every = 0;  // every Nth tag is gross
};

struct RunResult {
    double rmse_pos_m  = 0;  // after 2 s warmup
    double rmse_yaw_rad = 0;
    double max_pos_after_s   = 0;   // max error after `max_err_from`
    double max_err_from      = 2.0;
    int    tags_sent          = 0;
    int    tags_outliers_sent = 0;
    bool   saw_collision_mode = false;
    double collision_seen_at  = -1;
    double first_recovered_at = -1;  // error < 0.1 after shift_after_s
    uint8_t quality_at_drought_start = 0, quality_at_drought_end = 0;
};

RunResult run_scenario(FusionEngine& engine, const Sim& sim, const Scenario& sc,
                       Lcg& rng) {
    struct Event {
        double  arrival;
        int     type;  // 0 odom, 1 tag, 2 vio
        double  t;
        bool    outlier = false;
        uint64_t epoch  = 0;
    };
    std::vector<Event> events;

    for (double t = 0; t < std::min(sc.duration_s, sc.odom_stop_at);
         t += sc.odom_period) {
        events.push_back({t, 0, t});
    }
    int tag_idx = 0;
    for (double t = 0.01; t < sc.duration_s; t += sc.tag_period) {
        if (t >= sc.tags_off_from && t <= sc.tags_off_to) continue;
        ++tag_idx;
        const bool outlier =
            sc.tag_outlier_every > 0 && tag_idx % sc.tag_outlier_every == 0;
        events.push_back({t + sc.tag_latency, 1, t, outlier});
    }
    if (sc.with_vio) {
        for (double t = 0.02; t < std::min(sc.duration_s, sc.vio_stop_at);
             t += sc.vio_period) {
            const uint64_t epoch = t >= sc.vio_epoch_at ? 1u : 0u;
            events.push_back({t + sc.vio_latency, 2, t, false, epoch});
        }
    }
    std::stable_sort(events.begin(), events.end(),
                     [](const Event& a, const Event& b) {
                         return a.arrival < b.arrival;
                     });

    const ga::Mat4 odom_frame0 = planar_pose(1.0, -2.0, 0.4);
    const ga::Mat4 odom_frame1 = planar_pose(-3.0, 0.5, -1.1);

    RunResult res;
    double sum_p2 = 0, sum_y2 = 0;
    int    samples = 0;
    double next_sample = 2.0;  // warmup

    for (const auto& e : events) {
        switch (e.type) {
            case 0: engine.feed_odom(make_odom(sim, e.t, rng)); break;
            case 1:
                engine.feed_tag(make_tag(sim, e.t, rng, e.outlier));
                ++res.tags_sent;
                if (e.outlier) ++res.tags_outliers_sent;
                break;
            case 2:
                engine.feed_vio(make_vio(sim, e.t, rng, e.epoch,
                                         e.epoch == 0 ? odom_frame0
                                                      : odom_frame1));
                break;
        }

        const auto& st = engine.state();
        if (st.initialized && st.collision_mode && !res.saw_collision_mode) {
            res.saw_collision_mode = true;
            res.collision_seen_at  = e.arrival;
        }
        if (st.initialized && e.arrival >= next_sample) {
            next_sample += 0.1;
            const double t_state =
                static_cast<double>(st.t_ns - kT0Ns) * 1e-9;
            const ga::Mat4 tru = sim.truth(t_state);
            const double ex = st.T_field_robot[0][3] - tru[0][3];
            const double ey = st.T_field_robot[1][3] - tru[1][3];
            const double ep = std::sqrt(ex * ex + ey * ey);
            double dyaw = yaw_of(st.T_field_robot) - yaw_of(tru);
            while (dyaw > M_PI) dyaw -= 2 * M_PI;
            while (dyaw < -M_PI) dyaw += 2 * M_PI;
            sum_p2 += ep * ep;
            sum_y2 += dyaw * dyaw;
            ++samples;
            if (e.arrival >= res.max_err_from) {
                res.max_pos_after_s = std::max(res.max_pos_after_s, ep);
            }
            if (t_state >= sim.shift_after_s && ep < 0.1 &&
                res.first_recovered_at < 0) {
                res.first_recovered_at = t_state;
            }
            if (std::abs(e.arrival - sc.tags_off_from) < 0.15) {
                res.quality_at_drought_start = st.quality;
            }
            if (sc.tags_off_to > 0 &&
                std::abs(e.arrival - sc.tags_off_to) < 0.15) {
                res.quality_at_drought_end = st.quality;
            }
        }
    }
    if (samples > 0) {
        res.rmse_pos_m  = std::sqrt(sum_p2 / samples);
        res.rmse_yaw_rad = std::sqrt(sum_y2 / samples);
    }
    return res;
}

FusionParams test_params() {
    FusionParams p;
    p.lag_s       = 1.0;  // faster tests; behavior identical
    p.T_robot_imu = kTRobotImu;
    return p;
}

}  // namespace

// ---------------------------------------------------------------------------

TEST(FusionEngineTest, CleanRunTracksTruth) {
    FusionEngine engine(test_params());
    Sim sim;
    Lcg rng;
    const auto res = run_scenario(engine, sim, Scenario{}, rng);

    EXPECT_LT(res.rmse_pos_m, 0.05);
    EXPECT_LT(res.rmse_yaw_rad, 2.0 * M_PI / 180.0);
    const auto& c = engine.counters();
    EXPECT_EQ(c.reinits, 0u);
    // The 5-tag init burst yields one acceptance (the medoid), so 4 sent
    // tags never reach the gate; beyond that, essentially everything passes.
    EXPECT_GE(c.tag_accepted + 4, static_cast<uint64_t>(res.tags_sent) - 2);
    EXPECT_LE(c.tag_rejected_gate, 2u);
    EXPECT_GT(c.vio_fused_intervals, 0u);
    EXPECT_GT(c.odom_fused_intervals, 0u);
}

TEST(FusionEngineTest, OutlierTagsRejected) {
    FusionEngine engine(test_params());
    Sim sim;
    Lcg rng;
    Scenario sc;
    sc.tag_outlier_every = 10;
    const auto res = run_scenario(engine, sim, sc, rng);

    const auto& c = engine.counters();
    EXPECT_GE(static_cast<double>(c.tag_rejected_gate),
              0.8 * res.tags_outliers_sent);
    EXPECT_LE(static_cast<double>(c.tag_rejected_gate),
              1.2 * res.tags_outliers_sent);
    EXPECT_LT(res.rmse_pos_m, 0.075);  // ≤1.5× the clean-run bound
    EXPECT_EQ(c.reinits, 0u);
}

TEST(FusionEngineTest, InitMedoidRobustToOutlier) {
    FusionEngine engine(test_params());
    Sim sim;
    Lcg rng;
    // Feed exactly 5 init tags, the middle one grossly wrong.
    for (int i = 0; i < 5; ++i) {
        const double t = 0.03 * (i + 1);
        auto tag = make_tag(sim, t, rng, /*outlier=*/i == 2);
        engine.feed_tag(tag);
    }
    const auto& st = engine.state();
    ASSERT_TRUE(st.initialized);
    const double t_state = static_cast<double>(st.t_ns - kT0Ns) * 1e-9;
    const ga::Mat4 tru = sim.truth(t_state);
    const double ex = st.T_field_robot[0][3] - tru[0][3];
    const double ey = st.T_field_robot[1][3] - tru[1][3];
    EXPECT_LT(std::sqrt(ex * ex + ey * ey), 0.1);
}

TEST(FusionEngineTest, CollisionDetectedAndRecovered) {
    FusionEngine engine(test_params());
    Sim sim;
    sim.shift_after_s = 6.0;  // 0.58 m teleport tags see, dead-reckoning doesn't
    sim.shift_x       = 0.5;
    sim.shift_y       = 0.3;
    Lcg rng;
    Scenario sc;
    const auto res = run_scenario(engine, sim, sc, rng);

    ASSERT_TRUE(res.saw_collision_mode);
    EXPECT_LT(res.collision_seen_at, 7.0);  // within ~1 s of the jump
    ASSERT_GE(res.first_recovered_at, 0.0);
    EXPECT_LT(res.first_recovered_at, 8.0);  // re-converged within 2 s
    EXPECT_EQ(engine.counters().reinits, 0u);
    EXPECT_FALSE(engine.state().collision_mode);  // exited by run end
}

TEST(FusionEngineTest, VioDeathDegradesGracefully) {
    FusionEngine engine(test_params());
    Sim sim;
    Lcg rng;
    Scenario sc;
    sc.vio_stop_at = 5.0;
    const auto res = run_scenario(engine, sim, sc, rng);

    EXPECT_EQ(engine.counters().reinits, 0u);
    EXPECT_LT(res.rmse_pos_m, 0.10);
    EXPECT_GT(engine.counters().vio_fused_intervals, 0u);
}

TEST(FusionEngineTest, OdomDeathMidRunStaysBounded) {
    // CAN odom dies; tag-created states fall back to bridge factors while
    // VIO betweens keep covering intervals — the `no_odom` matrix row.
    FusionEngine engine(test_params());
    Sim sim;
    Lcg rng;
    Scenario sc;
    sc.odom_stop_at = 6.0;
    const auto res = run_scenario(engine, sim, sc, rng);

    EXPECT_EQ(engine.counters().reinits, 0u);
    EXPECT_LT(res.rmse_pos_m, 0.15);
    EXPECT_LT(res.max_pos_after_s, 0.5);
    EXPECT_GT(engine.counters().bridge_factors, 0u);
    EXPECT_TRUE(engine.state().initialized);
}

TEST(FusionEngineTest, TagsOnlyStaysBounded) {
    // Both relative sources gone — pure tag operation (`tags_only` row).
    FusionEngine engine(test_params());
    Sim sim;
    Lcg rng;
    Scenario sc;
    sc.with_vio     = false;
    sc.odom_stop_at = 6.0;
    const auto res = run_scenario(engine, sim, sc, rng);

    EXPECT_EQ(engine.counters().reinits, 0u);
    EXPECT_LT(res.rmse_pos_m, 0.3);
    EXPECT_GT(engine.counters().bridge_factors, 0u);
    EXPECT_TRUE(engine.state().initialized);
}

TEST(FusionEngineTest, OdomAndTagBlackoutVioKeepsStatesAdvancing) {
    // Regression: states used to be created only by odom samples and newer
    // tags. With both dead simultaneously the chain froze — buffered VIO
    // deltas never attached and the published pose stopped advancing even
    // though VIO was healthy. feed_vio now drives key creation in that
    // regime (the true `dead_reckoning` matrix row).
    FusionEngine engine(test_params());
    Sim sim;
    Lcg rng;
    Scenario sc;
    sc.duration_s    = 12.0;
    sc.odom_stop_at  = 6.0;
    sc.tags_off_from = 6.0;
    sc.tags_off_to   = 12.0;
    const auto res = run_scenario(engine, sim, sc, rng);

    EXPECT_EQ(engine.counters().reinits, 0u);
    EXPECT_TRUE(engine.state().initialized);
    // VIO dead reckoning drifts, but a frozen pose against the moving
    // figure-8 truth lands over a metre — this bound separates the two.
    EXPECT_LT(res.max_pos_after_s, 0.35);
    EXPECT_GT(engine.counters().bridge_factors, 0u);
}

TEST(FusionEngineTest, EpochResetNeverDifferencesAcross) {
    FusionEngine engine(test_params());
    Sim sim;
    Lcg rng;
    Scenario sc;
    sc.vio_epoch_at = 6.0;  // odom frame jumps to a different transform
    const auto res = run_scenario(engine, sim, sc, rng);

    EXPECT_GE(engine.counters().vio_skipped_epoch, 1u);
    EXPECT_LT(res.rmse_pos_m, 0.06);
    EXPECT_LT(res.max_pos_after_s, 0.15);  // no cross-epoch error spike
    EXPECT_EQ(engine.counters().reinits, 0u);
}

TEST(FusionEngineTest, TagDroughtPastLagSurvives) {
    FusionEngine engine(test_params());
    Sim sim;
    Lcg rng;
    Scenario sc;
    sc.tags_off_from = 5.0;
    sc.tags_off_to   = 8.0;  // 3 s ≫ the 1 s lag
    const auto res = run_scenario(engine, sim, sc, rng);

    EXPECT_EQ(engine.counters().reinits, 0u);
    EXPECT_TRUE(engine.state().initialized);
    // Quality decays while the marginal grows tag-less.
    EXPECT_LT(res.quality_at_drought_end, res.quality_at_drought_start);
    // Recovers once tags return (errors sampled after the drought).
    EXPECT_LT(res.rmse_pos_m, 0.08);
}

TEST(FusionEngineTest, DroughtTriggersReinitWhenStdExplodes) {
    FusionParams p = test_params();
    p.odom_sigma_vx = p.odom_sigma_vy = p.odom_sigma_omega = 0.5;  // sloppy odom
    p.reinit_pos_std_m = 0.05;
    FusionEngine engine(p);
    Sim sim;
    Lcg rng;
    Scenario sc;
    sc.with_vio      = false;  // nothing to bound drift but odom
    sc.tags_off_from = 5.0;
    sc.tags_off_to   = 9.0;
    const auto res = run_scenario(engine, sim, sc, rng);
    (void)res;

    EXPECT_GE(engine.counters().reinits, 1u);
    // Re-initialized from tags after the drought and is healthy again.
    EXPECT_TRUE(engine.state().initialized);
}

TEST(FusionEngineTest, HostClockTagsRejected) {
    FusionEngine engine(test_params());
    Sim sim;
    Lcg rng;
    auto tag         = make_tag(sim, 0.1, rng);
    tag.clock_source = gw::apriltag::TagPoseMeasurement::Clock::kHost;
    engine.feed_tag(tag);
    EXPECT_EQ(engine.counters().tag_rejected_clock, 1u);
    EXPECT_FALSE(engine.state().initialized);
}

TEST(FusionEngineTest, ManualResetReturnsToUninitialized) {
    FusionEngine engine(test_params());
    Sim sim;
    Lcg rng;
    for (int i = 0; i < 5; ++i) {
        engine.feed_tag(make_tag(sim, 0.03 * (i + 1), rng));
    }
    ASSERT_TRUE(engine.state().initialized);
    engine.reset();
    EXPECT_FALSE(engine.state().initialized);
    EXPECT_EQ(engine.counters().reinits, 0u);  // manual reset isn't automatic
}

TEST(FusionEngineTest, ExtrapolationMatchesAnalyticCircle) {
    // Constant twist (v, ω) from the origin: x = (v/ω)sin(ωt),
    // y = (v/ω)(1 − cos(ωt)), θ = ωt.
    const double v = 1.5, w = M_PI / 2, dt = 0.8;
    const ga::Mat4 out =
        extrapolate_planar(ga::mat4_identity(), v, 0.0, w, dt);
    EXPECT_NEAR(out[0][3], v / w * std::sin(w * dt), 1e-9);
    EXPECT_NEAR(out[1][3], v / w * (1.0 - std::cos(w * dt)), 1e-9);
    EXPECT_NEAR(yaw_of(out), w * dt, 1e-9);

    // Right perturbation from a non-identity pose.
    const ga::Mat4 T = planar_pose(3.0, -1.0, 0.7);
    const ga::Mat4 expect = ga::mat4_mul(
        T, extrapolate_planar(ga::mat4_identity(), v, 0.2, w, dt));
    const ga::Mat4 got = extrapolate_planar(T, v, 0.2, w, dt);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) EXPECT_NEAR(got[r][c], expect[r][c], 1e-9);
}

}  // namespace gw::fusion
