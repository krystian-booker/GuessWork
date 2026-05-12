#pragma once

#include <crow.h>

namespace gw::server {

class PipelineStatsView;

void register_status_routes(crow::SimpleApp& app, PipelineStatsView& view);

}  // namespace gw::server
