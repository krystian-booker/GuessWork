#include "server/fusion_supervisor.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <mutex>
#include <thread>
#include <variant>
#include <vector>

#include "calibration/calibration_store.hpp"
#include "core/latency_stats.hpp"
#include "core/odom_types.hpp"
#include "fusion/fusion_engine.hpp"
#include "fusion/teensy_now.hpp"
#include "server/apriltag_supervisor.hpp"
#include "server/fusion_config_repository.hpp"
#include "server/fusion_mode.hpp"
#include "server/imu_config_repository.hpp"
#include "net/robot_link.hpp"
#include "net/udp_payloads.h"
#include "server/vio_supervisor.hpp"

namespace gw::server {

namespace {

constexpr size_t kQueueCap = 4096;

bool mat4_equal(const gw::apriltag::Mat4& a, const gw::apriltag::Mat4& b) {
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            if (a[r][c] != b[r][c]) return false;
    return true;
}

// Engine-relevant configuration — compared memberwise so that live-applied
// fields (output_hz, max_extrapolation_ms) don't force an engine rebuild.
struct EngineConfig {
    bool                     enabled = false;
    gw::fusion::FusionParams params;

    bool operator==(const EngineConfig& o) const {
        const auto& p = params;
        const auto& q = o.params;
        if (enabled != o.enabled) return false;
        if (p.lag_s != q.lag_s || p.min_state_dt_ns != q.min_state_dt_ns ||
            p.tag_gate_chi2 != q.tag_gate_chi2 ||
            p.tag_huber_k != q.tag_huber_k || p.vio_huber_k != q.vio_huber_k ||
            p.odom_cauchy_k != q.odom_cauchy_k ||
            p.odom_sigma_vx != q.odom_sigma_vx ||
            p.odom_sigma_vy != q.odom_sigma_vy ||
            p.odom_sigma_omega != q.odom_sigma_omega ||
            p.vio_sigma_rot != q.vio_sigma_rot ||
            p.vio_sigma_trans != q.vio_sigma_trans ||
            p.collision_inflation != q.collision_inflation ||
            p.collision_window != q.collision_window ||
            p.reinit_pos_std_m != q.reinit_pos_std_m) {
            return false;
        }
        if (p.T_robot_imu.has_value() != q.T_robot_imu.has_value()) return false;
        if (p.T_robot_imu && !mat4_equal(*p.T_robot_imu, *q.T_robot_imu)) {
            return false;
        }
        return true;
    }
};

// Per-source arrival statistics, written by a drainer thread, read by
// status().
struct SourceStats {
    std::mutex mu;
    std::chrono::steady_clock::time_point window_start{};
    uint64_t                              window_count = 0;
    double                                rate_hz      = 0.0;
    std::chrono::steady_clock::time_point last_at{};
    bool                                  seen = false;

    void note() {
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard lk(mu);
        last_at = now;
        seen    = true;
        if (window_count == 0) window_start = now;
        ++window_count;
        const auto elapsed = now - window_start;
        if (elapsed >= std::chrono::seconds(1)) {
            rate_hz = static_cast<double>(window_count) /
                      std::chrono::duration<double>(elapsed).count();
            window_count = 0;
        }
    }

    void fill(FusionStatus::SourceEntry& e) {
        std::lock_guard lk(mu);
        e.rate_hz = rate_hz;
        if (seen) {
            e.last_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - last_at)
                                .count();
        }
    }
};

struct Reconfigure {
    EngineConfig cfg;
};
struct Reset {};

using EventPayload = std::variant<gw::apriltag::TagPoseMeasurement,
                                  gw::vio::VioOdometry, gw::ChassisSpeeds,
                                  Reconfigure, Reset>;

struct Event {
    EventPayload payload;
    int64_t      pushed_ns = 0;  // host steady clock at push (queue_wait stage)
};

int64_t steady_now_ns() {
    return std::chrono::steady_clock::now().time_since_epoch().count();
}

}  // namespace

struct FusionSupervisor::Impl {
    FusionConfigRepository& fusion_config;
    ImuConfigRepository&    imu_config;
    ApriltagSupervisor&     apriltag;
    VioSupervisor&          vio;
    gw::net::RobotLink&     robot;

    // --- internal event queue (drainers → engine thread) ---------------------
    std::mutex              q_mu;
    std::condition_variable q_cv;
    std::deque<Event>       queue;
    std::atomic<uint64_t>   queue_dropped{0};
    bool                    stopping = false;  // guarded by q_mu

    // --- engine (engine thread only) + published snapshot --------------------
    std::unique_ptr<gw::fusion::FusionEngine> engine;
    std::mutex                 snap_mu;
    gw::fusion::FusedState     snap_state;
    gw::fusion::FusionCounters snap_counters;
    bool                       snap_engine_present = false;

    // --- configuration --------------------------------------------------------
    std::mutex                  cfg_mu;
    std::optional<EngineConfig> active_cfg;  // last pushed to the engine thread
    std::string                 vio_reason;  // why VIO ingestion is off
    std::atomic<int64_t>        output_period_ns{10'000'000};
    std::atomic<int64_t>        max_extrapolation_ns{150'000'000};

    // --- Teensy-now (odom + tag drainers write, output/status read) -----------
    std::mutex                     now_mu;
    gw::fusion::TeensyNowEstimator teensy_now;

    // --- per-stage latency (drainer/engine/output threads write) --------------
    std::mutex       lat_mu;
    gw::LatencyStats tag_lat;        // trigger pulse → tag entering fusion
    gw::LatencyStats queue_lat;      // internal queue dwell
    gw::LatencyStats staleness_lat;  // teensy_now − newest state at CAN send

    // --- per-source stats + output counters -----------------------------------
    SourceStats           tag_stats, vio_stats, odom_stats;
    std::atomic<uint64_t> output_sent{0};
    std::atomic<uint64_t> output_send_errors{0};

    // --- bus subscriptions -------------------------------------------------------
    std::shared_ptr<gw::apriltag::TagPoseBus>            tag_bus;
    std::shared_ptr<gw::vio::VioBus>                     vio_bus;
    gw::apriltag::TagPoseBus::SubscriberHandle           tag_sub;
    gw::vio::VioBus::SubscriberHandle                    vio_sub;
    gw::OdomBus::SubscriberHandle                        odom_sub;

    std::atomic<bool> stop_flag{false};
    std::thread       tag_thread, vio_thread, odom_thread;
    std::thread       engine_thread, output_thread;

    Impl(FusionConfigRepository& fc, ImuConfigRepository& ic,
         ApriltagSupervisor& at, VioSupervisor& v, gw::net::RobotLink& r)
        : fusion_config(fc), imu_config(ic), apriltag(at), vio(v), robot(r) {}

    void push(EventPayload payload) {
        {
            std::lock_guard lk(q_mu);
            if (queue.size() >= kQueueCap) {
                queue.pop_front();
                queue_dropped.fetch_add(1, std::memory_order_relaxed);
            }
            queue.push_back(Event{std::move(payload), steady_now_ns()});
        }
        q_cv.notify_one();
    }

    EngineConfig build_config(std::string& vio_reason_out) {
        EngineConfig cfg;
        const auto row = fusion_config.get();
        cfg.enabled                 = row.enabled;
        cfg.params.lag_s            = row.lag_s;
        cfg.params.min_state_dt_ns  = row.min_state_dt_ms * 1'000'000;
        cfg.params.tag_gate_chi2    = row.tag_gate_chi2;
        cfg.params.tag_huber_k      = row.tag_huber_k;
        cfg.params.vio_huber_k      = row.vio_huber_k;
        cfg.params.odom_cauchy_k    = row.odom_cauchy_k;
        cfg.params.odom_sigma_vx    = row.odom_sigma_vx;
        cfg.params.odom_sigma_vy    = row.odom_sigma_vy;
        cfg.params.odom_sigma_omega = row.odom_sigma_omega;
        cfg.params.vio_sigma_rot    = row.vio_sigma_rot;
        cfg.params.vio_sigma_trans  = row.vio_sigma_trans;
        cfg.params.collision_inflation = row.collision_inflation;
        cfg.params.collision_window = static_cast<int>(row.collision_window);
        cfg.params.reinit_pos_std_m = row.reinit_pos_std_m;

        output_period_ns.store(1'000'000'000 / std::max<int64_t>(row.output_hz, 1),
                               std::memory_order_relaxed);
        max_extrapolation_ns.store(row.max_extrapolation_ms * 1'000'000,
                                   std::memory_order_relaxed);

        vio_reason_out.clear();
        try {
            const auto imu = imu_config.get();
            if (imu.t_imu_robot_json) {
                cfg.params.T_robot_imu =
                    gw::calib::parse_t_robot_imu(*imu.t_imu_robot_json);
            } else {
                vio_reason_out = "T_robot_imu unset";
            }
        } catch (const std::exception& e) {
            vio_reason_out = std::string("T_robot_imu unusable: ") + e.what();
        }
        return cfg;
    }

    void publish_snapshot() {
        std::lock_guard lk(snap_mu);
        if (engine) {
            snap_state          = engine->state();
            snap_counters       = engine->counters();
            snap_engine_present = true;
        } else {
            snap_state          = {};
            snap_counters       = {};
            snap_engine_present = false;
        }
    }

    void run_engine() {
        std::vector<Event> batch;
        while (true) {
            {
                std::unique_lock lk(q_mu);
                q_cv.wait(lk, [&] { return stopping || !queue.empty(); });
                if (stopping && queue.empty()) return;
                while (!queue.empty()) {
                    batch.push_back(std::move(queue.front()));
                    queue.pop_front();
                }
            }
            const int64_t pop_ns = steady_now_ns();
            {
                std::lock_guard lk(lat_mu);
                for (const auto& e : batch) {
                    queue_lat.add(static_cast<double>(pop_ns - e.pushed_ns) * 1e-6);
                }
            }
            for (auto& ev : batch) {
                std::visit(
                    [&](auto&& m) {
                        using T = std::decay_t<decltype(m)>;
                        if constexpr (std::is_same_v<T, Reconfigure>) {
                            engine.reset();
                            if (m.cfg.enabled) {
                                engine = std::make_unique<gw::fusion::FusionEngine>(
                                    m.cfg.params);
                            }
                        } else if constexpr (std::is_same_v<T, Reset>) {
                            if (engine) engine->reset();
                        } else if constexpr (std::is_same_v<
                                                 T, gw::apriltag::
                                                        TagPoseMeasurement>) {
                            if (engine) engine->feed_tag(m);
                        } else if constexpr (std::is_same_v<T,
                                                            gw::vio::VioOdometry>) {
                            if (engine) engine->feed_vio(m);
                        } else if constexpr (std::is_same_v<T, gw::ChassisSpeeds>) {
                            if (engine) engine->feed_odom(m);
                        }
                    },
                    ev.payload);
            }
            batch.clear();
            publish_snapshot();
        }
    }

    void run_output() {
        while (!stop_flag.load(std::memory_order_acquire)) {
            const auto period = std::chrono::nanoseconds(
                output_period_ns.load(std::memory_order_relaxed));
            std::this_thread::sleep_for(period);
            if (stop_flag.load(std::memory_order_acquire)) return;

            gw::fusion::FusedState st;
            {
                std::lock_guard lk(snap_mu);
                st = snap_state;
            }
            if (!st.initialized) continue;

            std::optional<int64_t> t_now;
            {
                const auto host_now =
                    std::chrono::steady_clock::now().time_since_epoch().count();
                std::lock_guard lk(now_mu);
                t_now = teensy_now.now(host_now);
            }
            if (!t_now) continue;

            // Pre-clamp staleness IS the headline trigger-pulse→pose-on-CAN
            // latency (st.t_ns is the newest fused state's pulse stamp).
            // Signed: slightly negative when the estimator is tag-biased.
            const int64_t raw_ns = *t_now - st.t_ns;
            {
                std::lock_guard lat_lk(lat_mu);
                staleness_lat.add(static_cast<double>(raw_ns) * 1e-6);
            }
            const int64_t max_ns =
                max_extrapolation_ns.load(std::memory_order_relaxed);
            const int64_t dt_ns = std::clamp<int64_t>(raw_ns, 0, max_ns);
            const double dt_s = static_cast<double>(dt_ns) * 1e-9;

            const auto T = gw::fusion::extrapolate_planar(
                st.T_field_robot, st.vx_mps, st.vy_mps, st.omega_radps, dt_s);

            gw::net::RobotLink::PoseSend pose;
            pose.t_ns      = static_cast<uint64_t>(st.t_ns + dt_ns);
            pose.x_m       = static_cast<float>(T[0][3]);
            pose.y_m       = static_cast<float>(T[1][3]);
            pose.theta_rad = static_cast<float>(std::atan2(T[1][0], T[0][0]));
            pose.quality   = st.quality;
            pose.mode      = current_mode_enum(st);
            pose.extrap_clamped = raw_ns > max_ns;
            // Planar marginal out of the 6×6 body-tangent cov ([ω, t]:
            // θz = index 2, x = 3, y = 4): xx yy tt xy xt yt.
            pose.cov[0] = static_cast<float>(st.cov[3 * 6 + 3]);
            pose.cov[1] = static_cast<float>(st.cov[4 * 6 + 4]);
            pose.cov[2] = static_cast<float>(st.cov[2 * 6 + 2]);
            pose.cov[3] = static_cast<float>(st.cov[3 * 6 + 4]);
            pose.cov[4] = static_cast<float>(st.cov[3 * 6 + 2]);
            pose.cov[5] = static_cast<float>(st.cov[4 * 6 + 2]);

            if (robot.send_pose(pose)) {
                output_sent.fetch_add(1, std::memory_order_relaxed);
            } else {
                output_send_errors.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }

    // The degraded-mode byte for the POSE packet — same derivation as
    // status(), evaluated fresh on the output thread.
    uint8_t current_mode_enum(const gw::fusion::FusedState& st) {
        bool vio_enabled = false;
        {
            std::lock_guard lk(cfg_mu);
            if (active_cfg) {
                vio_enabled = active_cfg->params.T_robot_imu.has_value();
            }
        }
        FusionStatus::SourceEntry tag_e, vio_e, odom_e;
        tag_stats.fill(tag_e);
        vio_stats.fill(vio_e);
        odom_stats.fill(odom_e);
        const std::string mode = derive_fusion_mode(
            st.initialized, tag_e.last_age_ms, vio_e.last_age_ms,
            odom_e.last_age_ms, vio_enabled, st.collision_mode);
        if (mode == "nominal") return gw::udpp::kModeNominal;
        if (mode == "no_vio") return gw::udpp::kModeNoVio;
        if (mode == "no_odom") return gw::udpp::kModeNoOdom;
        if (mode == "tags_only") return gw::udpp::kModeTagsOnly;
        if (mode == "dead_reckoning") return gw::udpp::kModeDeadReckoning;
        if (mode == "collision") return gw::udpp::kModeCollision;
        return gw::udpp::kModeUninitialized;
    }
};

FusionSupervisor::FusionSupervisor(FusionConfigRepository& fusion_config,
                                   ImuConfigRepository&    imu_config,
                                   ApriltagSupervisor&     apriltag,
                                   VioSupervisor&          vio,
                                   gw::net::RobotLink&     robot)
    : impl_(std::make_unique<Impl>(fusion_config, imu_config, apriltag, vio,
                                   robot)) {
    auto& im = *impl_;

    im.tag_bus  = apriltag.bus();
    im.vio_bus  = vio.bus();
    im.tag_sub  = im.tag_bus->subscribe(1024);
    im.vio_sub  = im.vio_bus->subscribe(1024);
    im.odom_sub = robot.odom_bus().subscribe(1024);

    reload();  // seeds the first Reconfigure before any measurement events

    im.tag_thread = std::thread([this] {
        gw::apriltag::TagPoseMeasurement m;
        while (impl_->tag_bus->wait_pop(impl_->tag_sub, m)) {
            impl_->tag_stats.note();
            if (m.clock_source ==
                gw::apriltag::TagPoseMeasurement::Clock::kTeensy) {
                // Tags also feed the Teensy-now estimator so it survives
                // CAN-odom death. The ~15–40 ms detect latency biases the
                // estimate EARLY (the EMA mixes it with the dominant
                // higher-rate odom feed when that's alive), which only
                // shortens output extrapolation — never overshoots it.
                const int64_t host_now = steady_now_ns();
                std::optional<int64_t> t_now;
                {
                    std::lock_guard lk(impl_->now_mu);
                    impl_->teensy_now.feed(m.t_ns, host_now);
                    t_now = impl_->teensy_now.now(host_now);
                }
                if (t_now) {
                    std::lock_guard lk(impl_->lat_mu);
                    impl_->tag_lat.add(
                        static_cast<double>(*t_now - m.t_ns) * 1e-6);
                }
            }
            impl_->push(m);
        }
    });
    im.vio_thread = std::thread([this] {
        gw::vio::VioOdometry m;
        while (impl_->vio_bus->wait_pop(impl_->vio_sub, m)) {
            impl_->vio_stats.note();
            impl_->push(m);
        }
    });
    im.odom_thread = std::thread([this] {
        gw::ChassisSpeeds m;
        while (impl_->robot.odom_bus().wait_pop(impl_->odom_sub, m)) {
            impl_->odom_stats.note();
            // t_ns / t_arrival_ns are 0 while the two-hop clock mapping is
            // unhealthy (Teensy telemetry down) — such samples can't be
            // placed on the fusion timeline; count the rate, feed nothing.
            if (m.t_ns == 0) continue;
            if (m.t_arrival_ns != 0) {
                const auto host_now =
                    std::chrono::steady_clock::now().time_since_epoch().count();
                std::lock_guard lk(impl_->now_mu);
                impl_->teensy_now.feed(static_cast<int64_t>(m.t_arrival_ns),
                                       host_now);
            }
            impl_->push(m);
        }
    });
    im.engine_thread = std::thread([this] { impl_->run_engine(); });
    im.output_thread = std::thread([this] { impl_->run_output(); });
}

FusionSupervisor::~FusionSupervisor() {
    auto& im = *impl_;
    im.stop_flag.store(true, std::memory_order_release);
    im.tag_bus->unsubscribe(im.tag_sub);
    im.vio_bus->unsubscribe(im.vio_sub);
    im.robot.odom_bus().unsubscribe(im.odom_sub);
    {
        std::lock_guard lk(im.q_mu);
        im.stopping = true;
    }
    im.q_cv.notify_all();

    for (std::thread* t : {&im.tag_thread, &im.vio_thread, &im.odom_thread,
                           &im.engine_thread, &im.output_thread}) {
        if (t->joinable()) t->join();
    }
}

bool FusionSupervisor::reload() {
    auto& im = *impl_;
    std::string vio_reason;
    EngineConfig cfg;
    try {
        cfg = im.build_config(vio_reason);
    } catch (const std::exception& e) {
        std::cerr << "FusionSupervisor: config unreadable: " << e.what() << "\n";
        return false;
    }

    bool rebuild = false;
    {
        std::lock_guard lk(im.cfg_mu);
        im.vio_reason = vio_reason;
        rebuild       = !im.active_cfg || !(*im.active_cfg == cfg);
        if (rebuild) im.active_cfg = cfg;
    }
    if (rebuild) {
        std::cerr << "FusionSupervisor: "
                  << (cfg.enabled ? "rebuilding engine" : "disabling engine")
                  << "\n";
        im.push(Reconfigure{std::move(cfg)});
    }
    return rebuild;
}

void FusionSupervisor::reset() { impl_->push(Reset{}); }

FusionStatus FusionSupervisor::status() {
    auto& im = *impl_;
    FusionStatus st;

    bool   enabled = false;
    double lag_s   = 0.0;
    {
        std::lock_guard lk(im.cfg_mu);
        if (im.active_cfg) {
            enabled = im.active_cfg->enabled;
            lag_s   = im.active_cfg->params.lag_s;
            st.vio_enabled = im.active_cfg->params.T_robot_imu.has_value();
        }
        st.vio_reason = im.vio_reason;
    }
    st.enabled = enabled;
    st.lag_s   = lag_s;

    {
        std::lock_guard lk(im.snap_mu);
        st.state    = im.snap_state;
        st.counters = im.snap_counters;
    }
    st.reason = !enabled            ? "disabled"
                : st.state.initialized ? "ok"
                                       : "awaiting first tag pose";

    im.tag_stats.fill(st.tag);
    im.vio_stats.fill(st.vio);
    im.odom_stats.fill(st.odom);
    st.tag.bus_dropped  = im.tag_bus->dropped(im.tag_sub);
    st.vio.bus_dropped  = im.vio_bus->dropped(im.vio_sub);
    st.odom.bus_dropped = im.robot.odom_bus().dropped(im.odom_sub);

    st.mode = derive_fusion_mode(st.state.initialized, st.tag.last_age_ms,
                                 st.vio.last_age_ms, st.odom.last_age_ms,
                                 st.vio_enabled, st.state.collision_mode);

    {
        std::lock_guard lk(im.lat_mu);
        const auto fill = [](FusionStatus::LatencyEntry& e,
                             const gw::LatencyStats& s) {
            e.last_ms = s.last_ms();
            e.p95_ms  = s.p95_ms();
            e.count   = s.count();
        };
        fill(st.lat_tag_pulse_to_fusion, im.tag_lat);
        fill(st.lat_queue_wait, im.queue_lat);
        fill(st.lat_pose_staleness, im.staleness_lat);
    }

    {
        std::lock_guard lk(im.now_mu);
        st.teensy_now_healthy   = im.teensy_now.healthy();
        st.teensy_now_offset_ms = im.teensy_now.offset_ms();
    }
    st.output_sent        = im.output_sent.load(std::memory_order_relaxed);
    st.output_send_errors = im.output_send_errors.load(std::memory_order_relaxed);
    st.queue_dropped      = im.queue_dropped.load(std::memory_order_relaxed);
    return st;
}

}  // namespace gw::server
