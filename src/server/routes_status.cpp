#include "server/routes_status.hpp"

#include <chrono>

#include <crow.h>

#include "server/camera_supervisor.hpp"
#include "server/route_helpers.hpp"

namespace gw::server {

void register_status_routes(crow::SimpleApp&                       app,
                            CameraSupervisor&                      supervisor,
                            std::chrono::steady_clock::time_point  started_at) {
    CROW_ROUTE(app, "/api/status")([&supervisor, started_at] {
        const auto cams = supervisor.snapshot_all();

        crow::json::wvalue::list items;
        items.reserve(cams.size());
        for (const auto& c : cams) {
            crow::json::wvalue j;
            j["id"]                = c.id;
            j["name"]              = c.name;
            j["serial"]            = c.serial;
            j["online"]            = c.online;
            j["frames_produced"]   = c.frames_produced;
            j["frames_dropped"]    = c.frames_dropped;
            j["frames_incomplete"] = c.frames_incomplete;
            j["fps_1s"]            = c.fps_1s;
            items.emplace_back(std::move(j));
        }

        const auto uptime = std::chrono::duration<double>(
                                std::chrono::steady_clock::now() - started_at)
                                .count();

        crow::json::wvalue body;
        body["ok"]       = true;
        body["uptime_s"] = uptime;
        body["cameras"]  = std::move(items);

        return json_response(200, std::move(body));
    });
}

}  // namespace gw::server
