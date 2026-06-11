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
#include "core/odom_types.hpp"
#include "fusion/fusion_engine.hpp"
#include "fusion/teensy_now.hpp"
#include "server/apriltag_supervisor.hpp"
#include "server/fusion_config_repository.hpp"
#include "server/imu_config_repository.hpp"
#include "server/teensy_manager.hpp"
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

using Event = std::variant<gw::apriltag::TagPoseMeasurement,
                           gw::vio::VioOdometry, gw::ChassisSpeeds,
                           Reconfigure, Reset>;

}  // namespace

struct FusionSupervisor::Impl {
    FusionConfigRepository& fusion_config;
    ImuConfigRepository&    imu_config;
    ApriltagSupervisor&     apriltag;
    VioSupervisor&          vio;
    TeensyManager&          teensy;

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

    // --- Teensy-now (odom drainer writes, output/status read) -----------------
    std::mutex                     now_mu;
    gw::fusion::TeensyNowEstimator teensy_now;

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
         ApriltagSupervisor& at, VioSupervisor& v, TeensyManager& t)
        : fusion_config(fc), imu_config(ic), apriltag(at), vio(v), teensy(t) {}

    void push(Event e) {
        {
            std::lock_guard lk(q_mu);
            if (queue.size() >= kQueueCap) {
                queue.pop_front();
                queue_dropped.fetch_add(1, std::memory_order_relaxed);
            }
            queue.push_back(std::move(e));
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
            for (auto& e : batch) {
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
                    e);
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

            const int64_t max_ns =
                max_extrapolation_ns.load(std::memory_order_relaxed);
            const int64_t dt_ns =
                std::clamp<int64_t>(*t_now - st.t_ns, 0, max_ns);
            const double dt_s = static_cast<double>(dt_ns) * 1e-9;

            const auto T = gw::fusion::extrapolate_planar(
                st.T_field_robot, st.vx_mps, st.vy_mps, st.omega_radps, dt_s);

            gw::FusedPose pose;
            pose.t_ns      = static_cast<uint64_t>(st.t_ns + dt_ns);
            pose.x_m       = static_cast<float>(T[0][3]);
            pose.y_m       = static_cast<float>(T[1][3]);
            pose.theta_rad = static_cast<float>(std::atan2(T[1][0], T[0][0]));
            pose.quality   = st.quality;

            std::string err;
            if (teensy.send_pose(pose, err)) {
                output_sent.fetch_add(1, std::memory_order_relaxed);
            } else {
                output_send_errors.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }
};

FusionSupervisor::FusionSupervisor(FusionConfigRepository& fusion_config,
                                   ImuConfigRepository&    imu_config,
                                   ApriltagSupervisor&     apriltag,
                                   VioSupervisor&          vio,
                                   TeensyManager&          teensy)
    : impl_(std::make_unique<Impl>(fusion_config, imu_config, apriltag, vio,
                                   teensy)) {
    auto& im = *impl_;

    im.tag_bus  = apriltag.bus();
    im.vio_bus  = vio.bus();
    im.tag_sub  = im.tag_bus->subscribe(1024);
    im.vio_sub  = im.vio_bus->subscribe(1024);
    im.odom_sub = teensy.odom_bus().subscribe(1024);

    reload();  // seeds the first Reconfigure before any measurement events

    im.tag_thread = std::thread([this] {
        gw::apriltag::TagPoseMeasurement m;
        while (impl_->tag_bus->wait_pop(impl_->tag_sub, m)) {
            impl_->tag_stats.note();
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
        while (impl_->teensy.odom_bus().wait_pop(impl_->odom_sub, m)) {
            impl_->odom_stats.note();
            {
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
    im.teensy.odom_bus().unsubscribe(im.odom_sub);
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
    st.odom.bus_dropped = im.teensy.odom_bus().dropped(im.odom_sub);

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
