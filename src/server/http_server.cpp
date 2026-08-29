#include "server/http_server.hpp"

#include <cstdlib>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

#include <crow.h>

#include "server/routes_apriltag.hpp"
#include "server/routes_calibration.hpp"
#include "server/routes_camera.hpp"
#include "server/routes_robot.hpp"
#include "server/routes_config.hpp"
#include "server/routes_fusion.hpp"
#include "server/routes_hardware_sync.hpp"
#include "server/routes_imu.hpp"
#include "server/routes_status.hpp"
#include "server/routes_stream.hpp"
#include "server/routes_vio.hpp"
#include "server/static_assets.hpp"

namespace gw::server {

struct HttpServer::Impl {
    crow::SimpleApp app;
    uint16_t        port;

    Impl(uint16_t                              p,
         CameraSupervisor&                     supervisor,
         CameraRepository&                     cameras,
         CalibrationSupervisor&                calibration,
         TriggerGroupRepository&               trigger_groups,
         SyncControllerManager&                        controller,
         ImuConfigRepository&                  imu_config,
         FieldLayoutRepository&                field_layouts,
         ApriltagSupervisor&                   apriltag,
         VioSupervisor&                        vio,
         VioConfigRepository&                  vio_config,
         NetConfigRepository&                  net_config,
         gw::net::RobotLink&                   robot,
         FusionSupervisor&                     fusion,
         FusionConfigRepository&               fusion_config,
         ImuAllanService&                      allan,
         ImuAttitudeService&                   attitude,
         std::chrono::steady_clock::time_point started_at)
        : port(p) {
        register_status_routes(app, supervisor, started_at);
        register_stream_routes(app, supervisor);
        register_camera_routes(app, cameras, supervisor);
        register_calibration_routes(app, cameras, calibration, supervisor);
        register_hardware_sync_routes(app, trigger_groups, controller);
        register_imu_routes(app, imu_config, controller, apriltag, fusion, allan,
                            attitude);
        register_apriltag_routes(app, field_layouts, apriltag, supervisor);
        register_vio_routes(app, vio, vio_config);
        register_robot_routes(app, net_config, robot, controller);
        register_fusion_routes(app, fusion, fusion_config);
        register_config_routes(app, cameras, trigger_groups, field_layouts,
                               imu_config, vio_config, net_config,
                               fusion_config, supervisor, apriltag, vio,
                               fusion, robot);
        register_static_routes(app);
    }
};

HttpServer::HttpServer(uint16_t                              port,
                       CameraSupervisor&                     supervisor,
                       CameraRepository&                     cameras,
                       CalibrationSupervisor&                calibration,
                       TriggerGroupRepository&               trigger_groups,
                       SyncControllerManager&                        controller,
                       ImuConfigRepository&                  imu_config,
                       FieldLayoutRepository&                field_layouts,
                       ApriltagSupervisor&                   apriltag,
                       VioSupervisor&                        vio,
                       VioConfigRepository&                  vio_config,
                       NetConfigRepository&                  net_config,
                       gw::net::RobotLink&                   robot,
                       FusionSupervisor&                     fusion,
                       FusionConfigRepository&               fusion_config,
                       ImuAllanService&                      allan,
                       ImuAttitudeService&                   attitude,
                       std::chrono::steady_clock::time_point started_at)
    : impl_(std::make_unique<Impl>(port, supervisor, cameras, calibration,
                                   trigger_groups, controller, imu_config,
                                   field_layouts, apriltag, vio, vio_config,
                                   net_config, robot, fusion, fusion_config,
                                   allan, attitude, started_at)) {}

HttpServer::~HttpServer() = default;

void HttpServer::run() {
    // Crow logs every request/response at INFO — with the UI polling at 1 Hz
    // that's hundreds of lines a minute, burying anything useful (like the
    // startup banner). Warnings and errors still come through; set
    // GW_HTTP_LOG=1 to get the full request log back.
    const char* verbose = std::getenv("GW_HTTP_LOG");
    if (!verbose || std::string_view(verbose) == "0") {
        crow::logger::setLogLevel(crow::LogLevel::Warning);
    }
    try {
        impl_->app.port(impl_->port).multithreaded().run();
    } catch (const std::system_error& e) {
        // Most commonly EADDRINUSE — another guesswork (or a dev server) owns
        // the port. Without this catch the exception escapes via terminate()
        // and looks like a crash inside asio/detail/throw_exception.hpp under
        // a debugger. Rethrown as a plain runtime_error so main() can print
        // it and unwind normally (camera/sync controller teardown must run in order).
        throw std::runtime_error(
            "failed to start HTTP server on port " + std::to_string(impl_->port) +
            ": " + e.what() +
            " (is another guesswork instance already running? try: lsof -i :" +
            std::to_string(impl_->port) + ")");
    }
}

void HttpServer::stop() {
    impl_->app.stop();
}

}  // namespace gw::server
