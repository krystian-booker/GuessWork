#pragma once

#include <crow.h>

namespace gw::server {

class CanConfigRepository;
class TeensyManager;

// CAN bridge routes:
//   GET  /api/can/config — the persisted mode ('off'|'roborio'|'systemcore')
//   PUT  /api/can/config — update mode; pushes CAN_MODE to the Teensy (the
//                          DB update succeeds even when the Teensy is offline
//                          — reconnect resync delivers it; response carries
//                          pushed/push_error)
//   GET  /api/can/status — bridge + odometry + clock-sync observability
//   POST /api/can/pose   — bench/debug pose downlink (placeholder until the
//                          Phase 6 fusion graph produces real poses)
void register_can_routes(crow::SimpleApp&     app,
                         CanConfigRepository& can_config,
                         TeensyManager&       teensy);

}  // namespace gw::server
