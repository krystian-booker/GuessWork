#include "server/static_assets.hpp"

#include <crow.h>

#ifdef GW_HAS_EMBEDDED_WEB

#include <cmrc/cmrc.hpp>
#include <string>
#include <string_view>

CMRC_DECLARE(gw_web_assets);

namespace gw::server {

namespace {

std::string_view mime_for(std::string_view path) {
    auto ends_with = [&](std::string_view suffix) {
        return path.size() >= suffix.size() &&
               path.compare(path.size() - suffix.size(), suffix.size(), suffix) == 0;
    };
    if (ends_with(".html")) return "text/html; charset=utf-8";
    if (ends_with(".js"))   return "application/javascript; charset=utf-8";
    if (ends_with(".mjs"))  return "application/javascript; charset=utf-8";
    if (ends_with(".css"))  return "text/css; charset=utf-8";
    if (ends_with(".json")) return "application/json; charset=utf-8";
    if (ends_with(".svg"))  return "image/svg+xml";
    if (ends_with(".png"))  return "image/png";
    if (ends_with(".jpg") || ends_with(".jpeg")) return "image/jpeg";
    if (ends_with(".ico"))  return "image/x-icon";
    if (ends_with(".woff2"))return "font/woff2";
    if (ends_with(".woff")) return "font/woff";
    if (ends_with(".map"))  return "application/json";
    return "application/octet-stream";
}

crow::response serve(const cmrc::embedded_filesystem& fs, const std::string& path) {
    if (!fs.exists(path)) {
        return crow::response(404);
    }
    const auto file = fs.open(path);
    crow::response res;
    res.body.assign(file.begin(), file.end());
    res.add_header("Content-Type", std::string(mime_for(path)));
    return res;
}

crow::response serve_or_index(const cmrc::embedded_filesystem& fs, std::string path) {
    if (path.empty() || fs.exists(path)) {
        return serve(fs, path.empty() ? "index.html" : path);
    }
    // SPA fallback: unknown paths return index.html so client-side routing
    // can pick them up.
    return serve(fs, "index.html");
}

}  // namespace

void register_static_routes(crow::SimpleApp& app) {
    const auto fs = cmrc::gw_web_assets::get_filesystem();

    CROW_ROUTE(app, "/")([fs] {
        return serve(fs, "index.html");
    });

    CROW_ROUTE(app, "/<path>")([fs](const crow::request&, std::string path) {
        // /api/* routes are registered before this one and take precedence,
        // but guard anyway in case a request slips through.
        if (path.rfind("api/", 0) == 0) {
            return crow::response(404);
        }
        return serve_or_index(fs, std::move(path));
    });
}

bool has_embedded_web() { return true; }

}  // namespace gw::server

#else  // !GW_HAS_EMBEDDED_WEB

namespace gw::server {
void register_static_routes(crow::SimpleApp&) {}
bool has_embedded_web() { return false; }
}  // namespace gw::server

#endif
