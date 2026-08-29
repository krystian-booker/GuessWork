#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <optional>
#include <thread>

#include "core/clock.hpp"
#include "net/robot_link.hpp"
#include "net/udp_payloads.h"

// Loopback tests: a fake "controller" socket on 127.0.0.1 exchanges real
// datagrams with a RobotLink. The sync controller clock view is identity (host==
// controller), which exercises the full mapping plumbing without a sync controller.

namespace gw::net {

namespace {

RobotLink::SyncClockView identity_view() {
    return {
        [](uint64_t host_ns) { return std::optional<uint64_t>(host_ns); },
        [](uint64_t controller_ns) { return std::optional<uint64_t>(controller_ns); },
    };
}

RobotLink::SyncClockView dead_view() {
    return {
        [](uint64_t) { return std::optional<uint64_t>(); },
        [](uint64_t) { return std::optional<uint64_t>(); },
    };
}

// A UDP socket standing in for the robot controller. Bound to an ephemeral
// port on 127.0.0.1; the actual port is read back so the test can point the
// link's robot_port at it.
struct FakeController {
    int      fd   = -1;
    uint16_t port = 0;

    FakeController() {
        fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in a{};
        a.sin_family      = AF_INET;
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        a.sin_port        = 0;
        EXPECT_EQ(::bind(fd, reinterpret_cast<sockaddr*>(&a), sizeof(a)), 0);
        socklen_t len = sizeof(a);
        ::getsockname(fd, reinterpret_cast<sockaddr*>(&a), &len);
        port = ntohs(a.sin_port);
    }
    ~FakeController() {
        if (fd >= 0) ::close(fd);
    }

    void send_speeds(uint16_t dest_port, const udpp::ChassisSpeedsPacket& pkt) {
        uint8_t buf[udpp::kChassisSpeedsLen];
        udpp::encode_chassis_speeds(pkt, buf);
        sockaddr_in dest{};
        dest.sin_family      = AF_INET;
        dest.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        dest.sin_port        = htons(dest_port);
        ::sendto(fd, buf, sizeof(buf), 0,
                 reinterpret_cast<sockaddr*>(&dest), sizeof(dest));
    }

    void send_raw(uint16_t dest_port, const void* data, size_t len) {
        sockaddr_in dest{};
        dest.sin_family      = AF_INET;
        dest.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        dest.sin_port        = htons(dest_port);
        ::sendto(fd, data, len, 0, reinterpret_cast<sockaddr*>(&dest),
                 sizeof(dest));
    }

    // Blocks up to timeout_ms for one pose datagram.
    std::optional<udpp::PosePacket> recv_pose(int timeout_ms) {
        pollfd pfd{fd, POLLIN, 0};
        if (::poll(&pfd, 1, timeout_ms) <= 0) return std::nullopt;
        uint8_t buf[128];
        const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        udpp::PosePacket pkt;
        if (n <= 0 ||
            !udpp::decode_pose(buf, static_cast<size_t>(n), pkt)) {
            return std::nullopt;
        }
        return pkt;
    }
};

// Ephemeral-port link: bind_port 0 is rejected by the config CHECK in
// production, but RobotLink itself accepts it and the kernel assigns — the
// test reads the port back through status(). To keep RobotLink simple it
// reports the configured port, so instead bind to a fixed high port with
// retry to dodge collisions.
uint16_t pick_port() {
    static uint16_t next = 45809;
    return next++;
}

bool wait_for(const std::function<bool()>& pred, int timeout_ms) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return pred();
}

}  // namespace

TEST(RobotLinkTest, ReceivesSpeedsAndPublishesOnSyncControllerClock) {
    RobotLink link(identity_view());
    FakeController robot;

    RobotLink::Config cfg;
    cfg.bind_port  = pick_port();
    cfg.robot_port = robot.port;
    link.start(cfg);

    auto sub = link.odom_bus().subscribe(64);

    udpp::ChassisSpeedsPacket pkt;
    pkt.counter      = 1;
    pkt.rio_time_us  = 5'000'000;
    pkt.vx_mps       = 1.5f;
    pkt.vy_mps       = -0.25f;
    pkt.omega_radps  = 0.75f;
    pkt.status_flags = 0x2;
    robot.send_speeds(cfg.bind_port, pkt);

    gw::ChassisSpeeds m;
    ASSERT_TRUE(link.odom_bus().wait_pop(sub, m));
    EXPECT_EQ(m.vx_mps, 1.5f);
    EXPECT_EQ(m.vy_mps, -0.25f);
    EXPECT_EQ(m.omega_radps, 0.75f);
    EXPECT_EQ(m.rio_time_us, 5'000'000u);
    EXPECT_EQ(m.status_flags, 0x2);
    // RIO↔host sync can't be warm after one packet, so t_ns falls back to
    // the (identity-mapped) arrival stamp — non-zero and recent.
    EXPECT_NE(m.t_ns, 0u);
    EXPECT_NE(m.t_arrival_ns, 0u);
    const uint64_t now = gw::Clock::now_ns();
    EXPECT_LT(now - m.t_ns, 1'000'000'000ull);

    link.odom_bus().unsubscribe(sub);
    link.stop();
}

TEST(RobotLinkTest, DeadSyncControllerHopPublishesZeroStamp) {
    RobotLink link(dead_view());
    FakeController robot;

    RobotLink::Config cfg;
    cfg.bind_port  = pick_port();
    cfg.robot_port = robot.port;
    link.start(cfg);

    auto sub = link.odom_bus().subscribe(64);
    udpp::ChassisSpeedsPacket pkt;
    pkt.rio_time_us = 1'000'000;
    robot.send_speeds(cfg.bind_port, pkt);

    gw::ChassisSpeeds m;
    ASSERT_TRUE(link.odom_bus().wait_pop(sub, m));
    // No host→sync controller mapping ⇒ the sample can't be placed on the fusion
    // timeline; consumers drop zero stamps.
    EXPECT_EQ(m.t_ns, 0u);
    EXPECT_EQ(m.t_arrival_ns, 0u);

    link.odom_bus().unsubscribe(sub);
    link.stop();
}

TEST(RobotLinkTest, PoseReachesLearnedController) {
    RobotLink link(identity_view());
    FakeController robot;

    RobotLink::Config cfg;
    cfg.bind_port  = pick_port();
    cfg.robot_port = robot.port;
    link.start(cfg);

    // Before any inbound packet there is no destination.
    RobotLink::PoseSend pose;
    pose.x_m = 3.5f;
    EXPECT_FALSE(link.send_pose(pose));
    EXPECT_EQ(link.status().pose_no_dest, 1u);

    // One speeds packet teaches the link the controller's address.
    udpp::ChassisSpeedsPacket pkt;
    pkt.rio_time_us = 42;
    robot.send_speeds(cfg.bind_port, pkt);
    ASSERT_TRUE(
        wait_for([&] { return link.status().rx_packets >= 1; }, 1000));

    pose.y_m       = -1.0f;
    pose.theta_rad = 0.5f;
    pose.quality   = 128;
    pose.mode      = udpp::kModeNominal;
    pose.cov[0]    = 0.02f;
    EXPECT_TRUE(link.send_pose(pose));

    const auto rx = robot.recv_pose(1000);
    ASSERT_TRUE(rx.has_value());
    EXPECT_EQ(rx->x_m, 3.5f);
    EXPECT_EQ(rx->y_m, -1.0f);
    EXPECT_EQ(rx->theta_rad, 0.5f);
    EXPECT_EQ(rx->quality, 128);
    EXPECT_EQ(rx->mode, udpp::kModeNominal);
    EXPECT_EQ(rx->cov[0], 0.02f);
    EXPECT_EQ(rx->counter, 1u);
    // RIO↔host sync is cold ⇒ no mapped validity time, flag clear.
    EXPECT_EQ(rx->rio_time_us, 0u);
    EXPECT_EQ(rx->flags & udpp::kPoseFlagClockSynced, 0);

    // A second pose advances the rolling counter.
    EXPECT_TRUE(link.send_pose(pose));
    const auto rx2 = robot.recv_pose(1000);
    ASSERT_TRUE(rx2.has_value());
    EXPECT_EQ(rx2->counter, 2u);

    link.stop();
}

TEST(RobotLinkTest, RejectsGarbageAndCountsCounterGaps) {
    RobotLink link(identity_view());
    FakeController robot;

    RobotLink::Config cfg;
    cfg.bind_port  = pick_port();
    cfg.robot_port = robot.port;
    link.start(cfg);

    const uint8_t garbage[16] = {0xDE, 0xAD};
    robot.send_raw(cfg.bind_port, garbage, sizeof(garbage));
    ASSERT_TRUE(
        wait_for([&] { return link.status().rx_rejected >= 1; }, 1000));
    EXPECT_EQ(link.status().rx_packets, 0u);

    udpp::ChassisSpeedsPacket pkt;
    pkt.rio_time_us = 1'000;
    pkt.counter     = 10;
    robot.send_speeds(cfg.bind_port, pkt);
    pkt.counter = 11;  // consecutive — no gap
    robot.send_speeds(cfg.bind_port, pkt);
    pkt.counter = 15;  // dropped 12..14
    robot.send_speeds(cfg.bind_port, pkt);
    ASSERT_TRUE(
        wait_for([&] { return link.status().rx_packets >= 3; }, 1000));
    EXPECT_EQ(link.status().rx_counter_gaps, 1u);

    link.stop();
}

TEST(RobotLinkTest, TwoHopMappingWarmsAndMapsRioSampleTime) {
    RobotLink link(identity_view());
    FakeController robot;

    RobotLink::Config cfg;
    cfg.bind_port  = pick_port();
    cfg.robot_port = robot.port;
    link.start(cfg);

    auto sub = link.odom_bus().subscribe(4096);

    // Feed ~1.6 s of 100 Hz speeds with a fixed fake RIO clock offset so the
    // bucketed-min fit (4 completed 250 ms buckets + warm-up) becomes
    // healthy. RIO time advances in lock-step with real time.
    const uint64_t rio0  = 20'000'000;  // 20 s
    const uint64_t host0 = gw::Clock::now_ns() / 1000;
    udpp::ChassisSpeedsPacket pkt;
    for (int i = 0; i < 160; ++i) {
        pkt.counter     = static_cast<uint32_t>(i + 1);
        pkt.rio_time_us = rio0 + (gw::Clock::now_ns() / 1000 - host0);
        robot.send_speeds(cfg.bind_port, pkt);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(wait_for([&] { return link.status().sync_healthy; }, 2000))
        << "rio<->host sync failed to warm";

    // The freshest samples must now carry the MAPPED rio sample time, which
    // under the identity controller view is host0 + (rio − rio0) ± jitter floor.
    gw::ChassisSpeeds m{};
    while (link.odom_bus().try_pop(sub, m)) {
    }
    pkt.counter += 1;
    pkt.rio_time_us = rio0 + (gw::Clock::now_ns() / 1000 - host0);
    robot.send_speeds(cfg.bind_port, pkt);
    ASSERT_TRUE(link.odom_bus().wait_pop(sub, m));
    ASSERT_NE(m.t_ns, 0u);
    const int64_t expect_ns =
        static_cast<int64_t>((host0 + (pkt.rio_time_us - rio0)) * 1000);
    const int64_t err_ns = static_cast<int64_t>(m.t_ns) - expect_ns;
    // Loopback transit floor + scheduler jitter; generous bound.
    EXPECT_LT(std::abs(err_ns), 50'000'000ll) << "mapped err " << err_ns;

    link.odom_bus().unsubscribe(sub);
    link.stop();
}

}  // namespace gw::net
