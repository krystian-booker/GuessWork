#include "server/http_server.hpp"

#include <crow.h>

#include "server/routes_calibration.hpp"
#include "server/routes_camera.hpp"
#include "server/routes_status.hpp"
#include "server/routes_stream.hpp"
#include "server/static_assets.hpp"

namespace gw::server {

struct HttpServer::Impl {
    crow::SimpleApp app;
    uint16_t        port;

    Impl(uint16_t                              p,
         CameraSupervisor&                     supervisor,
         CameraRepository&                     cameras,
         CalibrationSupervisor&                calibration,
         std::chrono::steady_clock::time_point started_at)
        : port(p) {
        register_status_routes(app, supervisor, started_at);
        register_stream_routes(app, supervisor);
        register_camera_routes(app, cameras, supervisor);
        register_calibration_routes(app, cameras, calibration);
        register_static_routes(app);
    }
};

HttpServer::HttpServer(uint16_t                              port,
                       CameraSupervisor&                     supervisor,
                       CameraRepository&                     cameras,
                       CalibrationSupervisor&                calibration,
                       std::chrono::steady_clock::time_point started_at)
    : impl_(std::make_unique<Impl>(port, supervisor, cameras, calibration, started_at)) {}

HttpServer::~HttpServer() = default;

void HttpServer::run() {
    impl_->app.port(impl_->port).multithreaded().run();
}

void HttpServer::stop() {
    impl_->app.stop();
}

}  // namespace gw::server
