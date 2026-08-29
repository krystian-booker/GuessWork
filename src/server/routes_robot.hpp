#pragma once

#include <crow.h>

namespace gw::net {
class RobotLink;
}

namespace gw::server {

class NetConfigRepository;
class SyncControllerManager;

// UDP robot-link routes (docs/ethernet-protocol.md):
//   GET  /api/robot/config — persisted link settings (ports, static IP)
//   PUT  /api/robot/config — partial update; rebinds the link (response
//                            carries restarted/restart_error — the DB update
//                            succeeds even when the rebind fails)
//   GET  /api/robot/status — link counters, odometry, and BOTH clock-sync
//                            hops (rio↔host from the link, host↔controller from
//                            the SyncControllerManager)
//   POST /api/robot/pose   — bench/debug pose downlink
void register_robot_routes(crow::SimpleApp&     app,
                           NetConfigRepository& net_config,
                           gw::net::RobotLink&  robot,
                           SyncControllerManager& controller);

}  // namespace gw::server
