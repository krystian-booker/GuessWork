#include "server/routes_status.hpp"

#include <crow.h>

#include "server/pipeline_stats.hpp"

namespace gw::server {

void register_status_routes(crow::SimpleApp& app, PipelineStatsView& view) {
    CROW_ROUTE(app, "/api/status")([&view] {
        const auto s = view.snapshot();

        crow::json::wvalue pipeline;
        pipeline["camera_connected"]  = s.camera_connected;
        pipeline["frames_produced"]   = s.frames_produced;
        pipeline["frames_dropped"]    = s.frames_dropped;
        pipeline["frames_incomplete"] = s.frames_incomplete;
        pipeline["fps_1s"]            = s.fps_1s;

        crow::json::wvalue body;
        body["ok"]       = true;
        body["uptime_s"] = s.uptime_s;
        body["pipeline"] = std::move(pipeline);

        crow::response res(body);
        res.add_header("Cache-Control", "no-store");
        return res;
    });
}

}  // namespace gw::server
