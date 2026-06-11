#pragma once

#include <crow.h>

namespace gw::server {

class FusionConfigRepository;
class FusionSupervisor;

// /api/fusion/status  GET       — fused pose + per-source health + counters.
// /api/fusion/config  GET, PUT  — single-row fusion tunables; PUT reloads the
//                                 supervisor (engine-relevant changes rebuild
//                                 the engine ⇒ reinit; response carries
//                                 `restarted`).
// /api/fusion/reset   POST      — manual engine reinit.
void register_fusion_routes(crow::SimpleApp&        app,
                            FusionSupervisor&       fusion,
                            FusionConfigRepository& fusion_config);

}  // namespace gw::server
