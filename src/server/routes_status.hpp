#pragma once

#include <chrono>

#include <crow.h>

namespace gw::server {

class CameraSupervisor;

void register_status_routes(crow::SimpleApp&            app,
                            CameraSupervisor&           supervisor,
                            std::chrono::steady_clock::time_point started_at);

}  // namespace gw::server
