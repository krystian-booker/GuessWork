#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include "server/apriltag_supervisor.hpp"
#include "server/camera_repository.hpp"
#include "server/database.hpp"
#include "server/field_layout_repository.hpp"
#include "server/fusion_config_repository.hpp"
#include "server/fusion_supervisor.hpp"
#include "server/imu_config_repository.hpp"
#include "server/routes_apriltag.hpp"
#include "server/teensy_manager.hpp"
#include "server/vio_config_repository.hpp"
#include "server/vio_supervisor.hpp"

// Threaded integration test for the fusion supervisor stack — the primary
// TSAN target: three producer threads hammer the real measurement buses
// while pollers hit status()/reload()/reset(), with no hardware anywhere
// (TeensyManager is constructed but never start()ed — the buses don't need
// its I/O thread).

namespace gw::server {

namespace {

constexpr int64_t kBaseNs = 5'000'000'000'000ll;  // synthetic Teensy epoch

int64_t steady_ns() {
    return std::chrono::steady_clock::now().time_since_epoch().count();
}

class FusionSupervisorTest : public ::testing::Test {
protected:
    void SetUp() override {
        path_ = std::filesystem::temp_directory_path() /
                ("gw_fusion_sup_test_" + std::to_string(::getpid()) + ".db");
        Database::remove_files(path_);
        db_ = std::make_unique<Database>(path_);
    }
    void TearDown() override {
        db_.reset();
        Database::remove_files(path_);
    }

    std::filesystem::path     path_;
    std::unique_ptr<Database> db_;
};

}  // namespace

TEST_F(FusionSupervisorTest, ConcurrentFeedStatusReloadIsClean) {
    CameraRepository       cameras(*db_);
    FieldLayoutRepository  layouts(*db_);
    ImuConfigRepository    imu_cfg(*db_);
    VioConfigRepository    vio_cfg(*db_);
    FusionConfigRepository fusion_cfg(*db_);
    seed_default_field_layout(layouts);

    TeensyManager      teensy;  // not start()ed — no serial probing in tests
    ApriltagSupervisor apriltag(cameras, layouts, imu_cfg);
    VioSupervisor      vio(cameras, imu_cfg, vio_cfg, teensy);
    FusionSupervisor   fusion(fusion_cfg, imu_cfg, apriltag, vio, teensy);

    std::atomic<bool> stop{false};
    const int64_t     t0 = steady_ns();
    // Synthetic Teensy time tracks wall time so the TeensyNowEstimator's
    // host↔Teensy mapping is self-consistent.
    const auto t_now = [&] { return kBaseNs + (steady_ns() - t0); };

    std::thread tag_producer([&] {
        auto bus = apriltag.bus();
        while (!stop.load(std::memory_order_acquire)) {
            gw::apriltag::TagPoseMeasurement m;
            m.t_ns         = t_now();
            m.clock_source = gw::apriltag::TagPoseMeasurement::Clock::kTeensy;
            m.n_tags       = 2;
            m.T_field_robot = gw::apriltag::mat4_identity();
            for (int i = 0; i < 3; ++i) m.cov[i * 6 + i] = 1e-4;
            for (int i = 3; i < 6; ++i) m.cov[i * 6 + i] = 1e-4;
            bus->publish(m);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    std::thread odom_producer([&] {
        uint8_t counter = 0;
        while (!stop.load(std::memory_order_acquire)) {
            gw::ChassisSpeeds s;
            s.t_ns         = static_cast<uint64_t>(t_now());
            s.t_arrival_ns = s.t_ns;
            s.vx_mps       = 0.1f;
            s.counter      = ++counter;
            teensy.odom_bus().publish(s);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });
    std::thread vio_producer([&] {
        // T_robot_imu is unset → the engine ignores these; the drainer +
        // queue paths are what TSAN watches.
        auto bus = vio.bus();
        while (!stop.load(std::memory_order_acquire)) {
            gw::vio::VioOdometry m;
            m.t_ns        = t_now();
            m.epoch       = 1;
            m.initialized = true;
            bus->publish(m);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    std::thread poller([&] {
        while (!stop.load(std::memory_order_acquire)) {
            (void)fusion.status();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    std::thread churner([&] {
        bool flip = false;
        while (!stop.load(std::memory_order_acquire)) {
            if (flip) fusion.reload();
            else fusion.reset();
            flip = !flip;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    });

    // Poll-with-deadline: generous under TSAN slowdown. The churner resets
    // the engine every ~400 ms, so just require initialization to be
    // OBSERVED at some point (init needs only ~50 ms of tags).
    bool saw_initialized = false;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
        if (fusion.status().state.initialized) {
            saw_initialized = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Keep feeding ≥1.5 s — the per-source rate windows need a full second
    // before they report a number.
    std::this_thread::sleep_for(std::chrono::milliseconds(1600));
    stop.store(true, std::memory_order_release);
    for (std::thread* t :
         {&tag_producer, &odom_producer, &vio_producer, &poller, &churner}) {
        t->join();
    }

    EXPECT_TRUE(saw_initialized);
    const auto st = fusion.status();
    EXPECT_TRUE(st.enabled);
    EXPECT_GT(st.tag.rate_hz, 50.0);   // 100 Hz nominal, generous floor
    EXPECT_GT(st.odom.rate_hz, 100.0);
    EXPECT_TRUE(st.teensy_now_healthy);
    EXPECT_GT(st.lat_tag_pulse_to_fusion.count, 0u);
    EXPECT_GT(st.lat_queue_wait.count, 0u);
    EXPECT_EQ(st.counters.update_exceptions, 0u);
    EXPECT_EQ(st.vio_enabled, false);  // no T_robot_imu in a fresh DB
    if (st.state.initialized) {
        EXPECT_TRUE(st.mode == "no_vio" || st.mode == "nominal") << st.mode;
    }
    // Destruction order (fusion → vio/apriltag/teensy) exercises the join
    // paths; reaching the end without deadlock/race is the real assertion.
}

}  // namespace gw::server
