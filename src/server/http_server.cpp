#include "server/http_server.hpp"

#include <crow.h>

#include "server/routes_status.hpp"
#include "server/static_assets.hpp"

namespace gw::server {

struct HttpServer::Impl {
    crow::SimpleApp app;
    uint16_t        port;

    Impl(uint16_t p, PipelineStatsView& stats) : port(p) {
        register_status_routes(app, stats);
        register_static_routes(app);
    }
};

HttpServer::HttpServer(uint16_t port, PipelineStatsView& stats)
    : impl_(std::make_unique<Impl>(port, stats)) {}

HttpServer::~HttpServer() = default;

void HttpServer::run() {
    impl_->app.port(impl_->port).multithreaded().run();
}

void HttpServer::stop() {
    impl_->app.stop();
}

}  // namespace gw::server
