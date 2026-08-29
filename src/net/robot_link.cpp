#include "net/robot_link.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include "core/clock.hpp"
#include "net/udp_payloads.h"

namespace gw::net {

namespace {

std::string addr_to_string(const sockaddr_in& a) {
    char buf[INET_ADDRSTRLEN] = {};
    inet_ntop(AF_INET, &a.sin_addr, buf, sizeof(buf));
    return std::string(buf) + ":" + std::to_string(ntohs(a.sin_port));
}

}  // namespace

struct RobotLink::Impl {
    gw::OdomBus     odom_bus;
    SyncClockView controller_clock;

    mutable std::mutex mu_;
    Config             cfg;
    int                fd = -1;
    std::thread        rx_thread;
    std::atomic<bool>  running{false};

    // --- guarded by mu_ ---
    gw::ClockSync rio_sync;  // remote = RIO FPGA µs, local = host µs
    std::optional<sockaddr_in> learned_addr;   // last speeds source
    std::optional<sockaddr_in> configured_addr;  // from cfg.robot_ip
    uint64_t rx_packets = 0, rx_rejected = 0, rx_counter_gaps = 0;
    uint32_t last_counter    = 0;
    bool     have_counter    = false;
    uint64_t last_rx_host_ns = 0;
    std::optional<gw::ChassisSpeeds> odom_last;
    std::deque<uint64_t> rate_window;  // host arrival ns, ~1 s
    uint32_t pose_counter = 0;
    uint64_t pose_sent = 0, pose_send_errors = 0, pose_no_dest = 0;

    explicit Impl(SyncClockView view) : controller_clock(std::move(view)) {}

    void run();
    void handle_packet(const uint8_t* buf, size_t len, const sockaddr_in& src,
                       uint64_t arrival_ns);
};

void RobotLink::Impl::run() {
    uint8_t buf[128];
    while (running.load(std::memory_order_relaxed)) {
        pollfd pfd{fd, POLLIN, 0};
        const int pr = ::poll(&pfd, 1, /*timeout_ms=*/200);
        if (pr <= 0) continue;

        sockaddr_in src{};
        socklen_t   srclen = sizeof(src);
        const ssize_t n = ::recvfrom(fd, buf, sizeof(buf), 0,
                                     reinterpret_cast<sockaddr*>(&src), &srclen);
        if (n <= 0) continue;
        handle_packet(buf, static_cast<size_t>(n), src, gw::Clock::now_ns());
    }
}

void RobotLink::Impl::handle_packet(const uint8_t* buf, size_t len,
                                    const sockaddr_in& src,
                                    uint64_t arrival_ns) {
    udpp::ChassisSpeedsPacket pkt;
    if (!udpp::decode_chassis_speeds(buf, len, pkt)) {
        std::lock_guard lk(mu_);
        ++rx_rejected;
        return;
    }

    gw::ChassisSpeeds out;
    {
        std::lock_guard lk(mu_);
        ++rx_packets;
        last_rx_host_ns = arrival_ns;
        learned_addr    = src;

        if (have_counter && pkt.counter != last_counter + 1) ++rx_counter_gaps;
        last_counter = pkt.counter;
        have_counter = true;

        rate_window.push_back(arrival_ns);
        while (!rate_window.empty() &&
               rate_window.front() + 1'000'000'000ull < arrival_ns) {
            rate_window.pop_front();
        }

        rio_sync.feed(pkt.rio_time_us, arrival_ns / 1000);

        // Timestamp chain: RIO sample time → host → sync controller. Falls back to
        // the mapped arrival time, then to 0 (consumers drop zero stamps —
        // if the sync controller hop is down there are no frames to fuse against
        // anyway).
        out.rio_time_us = pkt.rio_time_us;
        uint64_t t_controller = 0, arrival_controller = 0;
        if (controller_clock.to_controller_ns) {
            if (const auto a = controller_clock.to_controller_ns(arrival_ns)) {
                arrival_controller = *a;
            }
            if (const auto host_ns = rio_sync.to_local_ns(pkt.rio_time_us);
                host_ns && rio_sync.healthy(arrival_ns / 1000)) {
                if (const auto t = controller_clock.to_controller_ns(*host_ns)) {
                    t_controller = *t;
                }
            }
        }
        out.t_ns         = t_controller ? t_controller : arrival_controller;
        out.t_arrival_ns = arrival_controller;
        out.vx_mps       = pkt.vx_mps;
        out.vy_mps       = pkt.vy_mps;
        out.omega_radps  = pkt.omega_radps;
        out.status_flags = static_cast<uint16_t>(pkt.status_flags);
        out.counter      = static_cast<uint8_t>(pkt.counter);
        odom_last        = out;
    }
    odom_bus.publish(out);  // bus is thread-safe; publish outside the lock
}

RobotLink::RobotLink(SyncClockView controller_clock)
    : impl_(std::make_unique<Impl>(std::move(controller_clock))) {}

RobotLink::~RobotLink() { stop(); }

gw::OdomBus& RobotLink::odom_bus() { return impl_->odom_bus; }

void RobotLink::start(const Config& cfg) {
    stop();
    {
        std::lock_guard lk(impl_->mu_);
        impl_->cfg = cfg;
        impl_->configured_addr.reset();
        if (!cfg.robot_ip.empty()) {
            sockaddr_in a{};
            a.sin_family = AF_INET;
            a.sin_port   = htons(cfg.robot_port);
            if (inet_pton(AF_INET, cfg.robot_ip.c_str(), &a.sin_addr) == 1) {
                impl_->configured_addr = a;
            } else {
                throw std::runtime_error("robot_ip is not a valid IPv4 address: " +
                                         cfg.robot_ip);
            }
        }
    }
    if (!cfg.enabled) return;

    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) throw std::runtime_error("socket() failed");
    const int one = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in bind_addr{};
    bind_addr.sin_family      = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port        = htons(cfg.bind_port);
    if (::bind(fd, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) != 0) {
        ::close(fd);
        throw std::runtime_error("bind(:" + std::to_string(cfg.bind_port) +
                                 ") failed — port in use?");
    }
    const int flags = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    impl_->fd = fd;
    impl_->running.store(true, std::memory_order_relaxed);
    impl_->rx_thread = std::thread([this] { impl_->run(); });
}

void RobotLink::stop() {
    impl_->running.store(false, std::memory_order_relaxed);
    if (impl_->rx_thread.joinable()) impl_->rx_thread.join();
    if (impl_->fd >= 0) {
        ::close(impl_->fd);
        impl_->fd = -1;
    }
}

void RobotLink::reconfigure(const Config& cfg) { start(cfg); }

bool RobotLink::send_pose(const PoseSend& pose) {
    udpp::PosePacket pkt;
    sockaddr_in      dest{};
    int              fd = -1;
    {
        std::lock_guard lk(impl_->mu_);
        fd = impl_->fd;
        if (fd < 0) {
            ++impl_->pose_send_errors;
            return false;
        }
        std::optional<sockaddr_in> d = impl_->configured_addr;
        if (!d && impl_->learned_addr) {
            d = *impl_->learned_addr;
            d->sin_port = htons(impl_->cfg.robot_port);
        }
        if (!d) {
            ++impl_->pose_no_dest;
            return false;
        }
        dest = *d;

        pkt.counter   = ++impl_->pose_counter;
        pkt.x_m       = pose.x_m;
        pkt.y_m       = pose.y_m;
        pkt.theta_rad = pose.theta_rad;
        pkt.quality   = pose.quality;
        pkt.mode      = pose.mode;
        for (int i = 0; i < 6; ++i) pkt.cov[i] = pose.cov[i];
        if (pose.extrap_clamped) pkt.flags |= udpp::kPoseFlagExtrapClamped;

        // Validity time back through the chain: sync controller → host → RIO.
        if (impl_->controller_clock.to_host_ns) {
            if (const auto host_ns = impl_->controller_clock.to_host_ns(pose.t_ns)) {
                if (const auto rio = impl_->rio_sync.to_remote_us(*host_ns);
                    rio && impl_->rio_sync.healthy(gw::Clock::now_ns() / 1000)) {
                    pkt.rio_time_us = *rio;
                    pkt.flags |= udpp::kPoseFlagClockSynced;
                }
            }
        }
    }

    uint8_t buf[udpp::kPoseLen];
    udpp::encode_pose(pkt, buf);
    const ssize_t n = ::sendto(fd, buf, sizeof(buf), 0,
                               reinterpret_cast<const sockaddr*>(&dest),
                               sizeof(dest));
    std::lock_guard lk(impl_->mu_);
    if (n == static_cast<ssize_t>(sizeof(buf))) {
        ++impl_->pose_sent;
        return true;
    }
    ++impl_->pose_send_errors;
    return false;
}

RobotLink::Status RobotLink::status() const {
    std::lock_guard lk(impl_->mu_);
    Status st;
    st.running         = impl_->running.load(std::memory_order_relaxed);
    st.bind_port       = impl_->cfg.bind_port;
    st.rx_packets      = impl_->rx_packets;
    st.rx_rejected     = impl_->rx_rejected;
    st.rx_counter_gaps = impl_->rx_counter_gaps;
    st.odom_last       = impl_->odom_last;

    const uint64_t now = gw::Clock::now_ns();
    if (impl_->last_rx_host_ns > 0) {
        st.odom_last_age_ms =
            static_cast<int64_t>((now - impl_->last_rx_host_ns) / 1'000'000);
    }
    // Rate over the trailing window (entries older than 1 s are purged on
    // arrival; purge again here so an idle link decays to 0).
    uint64_t in_window = 0;
    for (const uint64_t t : impl_->rate_window) {
        if (t + 1'000'000'000ull >= now) ++in_window;
    }
    st.odom_rate_hz = static_cast<double>(in_window);

    if (impl_->configured_addr) {
        st.robot_addr = addr_to_string(*impl_->configured_addr);
    } else if (impl_->learned_addr) {
        sockaddr_in a = *impl_->learned_addr;
        a.sin_port    = htons(impl_->cfg.robot_port);
        st.robot_addr = addr_to_string(a);
    }

    st.pose_sent        = impl_->pose_sent;
    st.pose_send_errors = impl_->pose_send_errors;
    st.pose_no_dest     = impl_->pose_no_dest;

    st.sync_healthy   = impl_->rio_sync.healthy(now / 1000);
    st.sync_offset_us = impl_->rio_sync.offset_us();
    st.sync_drift_ppm = impl_->rio_sync.drift_ppm();
    st.sync_samples   = impl_->rio_sync.samples();
    st.sync_resets    = impl_->rio_sync.resets();

    st.controller_hop_healthy =
        impl_->controller_clock.to_controller_ns &&
        impl_->controller_clock.to_controller_ns(now).has_value();
    return st;
}

}  // namespace gw::net
