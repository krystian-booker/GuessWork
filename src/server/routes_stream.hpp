#pragma once

#include <crow.h>

namespace gw::server {

class CameraSupervisor;

void register_stream_routes(crow::SimpleApp& app, CameraSupervisor& supervisor);

}  // namespace gw::server
