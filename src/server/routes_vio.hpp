#pragma once

#include <crow.h>

namespace gw::server {

class VioSupervisor;
class VioConfigRepository;

// /api/vio/status   GET   — runner/init state, rates, counters, gating reason.
// /api/vio/config   GET, PUT — single-row OpenVINS tunables (PUT reloads).
// /api/vio/restart  POST  — forces a VIO reinitialization (epoch bump).
void register_vio_routes(crow::SimpleApp&     app,
                         VioSupervisor&       vio,
                         VioConfigRepository& vio_config);

}  // namespace gw::server
