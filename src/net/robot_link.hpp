#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "core/clock_sync.hpp"
#include "core/odom_types.hpp"

namespace gw::net {

// UDP link to the robot controller (RoboRIO / SystemCore) — the transport
// that replaced the sync controller CAN bridge. Wire contract: src/net/udp_payloads.h
// (prose copy docs/ethernet-protocol.md).
//
//   controller --UDP:bind_port--> host   CHASSIS_SPEEDS @ 50-100 Hz
//   host --UDP:robot_port--> controller  POSE @ fusion output_hz
//
// The host learns the controller's address from the source of the last valid
// speeds packet (a static robot_ip can override). Timestamps: each inbound
// packet carries the controller's 64-bit FPGA sample time; an internal
// ClockSync (fed with host recvfrom stamps) maps RIO→host, and the injected
// SyncClockView (fed by SyncControllerManager's telemetry stream) maps host→sync controller
// — chassis speeds are published on the sync controller clock like every other
// measurement, with no direct RIO↔sync controller link. Fallback ladder for t_ns:
// mapped sample time → mapped arrival time → 0 (consumers drop zero stamps).
class RobotLink {
public:
    struct Config {
        bool        enabled    = true;
        uint16_t    bind_port  = 5809;  // udpp::kDefaultHostPort
        uint16_t    robot_port = 5810;  // udpp::kDefaultRobotPort
        std::string robot_ip;           // empty = learn from inbound packets
    };

    // Host↔sync controller clock mapping, provided by SyncControllerManager (thread-safe
    // snapshots). Both return nullopt while the mapping is unhealthy.
    struct SyncClockView {
        std::function<std::optional<uint64_t>(uint64_t host_ns)>   to_controller_ns;
        std::function<std::optional<uint64_t>(uint64_t controller_ns)> to_host_ns;
    };

    // Everything the fusion output loop knows about the pose it is
    // publishing; maps 1:1 onto the POSE packet (udp_payloads.h).
    struct PoseSend {
        uint64_t t_ns = 0;  // sync controller-domain validity time
        float    x_m = 0, y_m = 0, theta_rad = 0;
        uint8_t  quality        = 0;
        uint8_t  mode           = 0;      // udpp::kMode*
        bool     extrap_clamped = false;
        float    cov[6]         = {0, 0, 0, 0, 0, 0};  // xx yy tt xy xt yt
    };

    struct Status {
        bool     running          = false;
        uint16_t bind_port        = 0;
        // Inbound.
        uint64_t rx_packets       = 0;  // accepted speeds packets
        uint64_t rx_rejected      = 0;  // wrong magic/version/type/length
        uint64_t rx_counter_gaps  = 0;  // counter jumped by more than +1
        double   odom_rate_hz     = 0.0;
        std::optional<int64_t>           odom_last_age_ms;
        std::optional<gw::ChassisSpeeds> odom_last;
        std::optional<std::string>       robot_addr;  // learned or configured
        // Outbound.
        uint64_t pose_sent        = 0;
        uint64_t pose_send_errors = 0;
        uint64_t pose_no_dest     = 0;  // no learned/configured robot address
        // RIO ↔ host clock sync.
        bool     sync_healthy   = false;
        double   sync_offset_us = 0.0;
        double   sync_drift_ppm = 0.0;
        uint64_t sync_samples   = 0;
        uint64_t sync_resets    = 0;
        // Host ↔ sync controller hop (from the injected view; false = unhealthy).
        bool controller_hop_healthy = false;
    };

    explicit RobotLink(SyncClockView controller_clock);
    ~RobotLink();

    RobotLink(const RobotLink&)            = delete;
    RobotLink& operator=(const RobotLink&) = delete;

    // Decoded chassis-speeds samples (sync controller-clock timestamps when the
    // two-hop mapping is healthy; see class comment). Subscriber: fusion.
    // Valid for the link's lifetime.
    gw::OdomBus& odom_bus();

    // Bind the socket and spawn the RX thread. No-op when cfg.enabled is
    // false. Throws std::runtime_error on socket/bind failure.
    void start(const Config& cfg);
    void stop();

    // stop() + start(cfg) — used by the config PUT route.
    void reconfigure(const Config& cfg);

    // Encode + sendto the controller. Returns false (and counts why) when
    // the link is down or no destination is known. Never blocks.
    bool send_pose(const PoseSend& pose);

    Status status() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace gw::net
