#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <limits>
#include <string_view>

#include <rtc/rtc.hpp>

#include "server/apriltag_supervisor.hpp"
#include "server/calibration_supervisor.hpp"
#include "server/camera_repository.hpp"
#include "net/robot_link.hpp"
#include "server/net_config_repository.hpp"
#include "server/camera_supervisor.hpp"
#include "server/database.hpp"
#include "server/field_layout_repository.hpp"
#include "server/fusion_config_repository.hpp"
#include "server/fusion_supervisor.hpp"
#include "server/http_server.hpp"
#include "server/imu_allan_service.hpp"
#include "server/imu_config_repository.hpp"
#include "server/static_assets.hpp"
#include "server/routes_apriltag.hpp"
#include "server/teensy_manager.hpp"
#include "server/trigger_group_repository.hpp"
#include "server/vio_config_repository.hpp"
#include "server/vio_supervisor.hpp"

namespace {

struct Cli {
    uint16_t port            = 8080;
    uint32_t stream_width    = 1280;
    uint32_t stream_height   = 960;
    uint32_t stream_fps      = 30;
    uint32_t stream_bitrate  = 2'000'000;
    bool     reset_db        = false;
};

template <typename T>
T parse_uint(const char* s, T fallback) {
    const long long v = std::atoll(s);
    if (v <= 0 || v > std::numeric_limits<T>::max()) return fallback;
    return static_cast<T>(v);
}

Cli parse_cli(int argc, char** argv) {
    Cli cli;
    for (int i = 1; i < argc; ++i) {
        const std::string_view k = argv[i];
        if (k == "--reset-db") cli.reset_db = true;
    }
    for (int i = 1; i + 1 < argc; ++i) {
        const std::string_view k = argv[i];
        const char*           v = argv[i + 1];
        if      (k == "--port")            cli.port           = parse_uint<uint16_t>(v, cli.port);
        else if (k == "--stream-width")    cli.stream_width   = parse_uint<uint32_t>(v, cli.stream_width);
        else if (k == "--stream-height")   cli.stream_height  = parse_uint<uint32_t>(v, cli.stream_height);
        else if (k == "--stream-fps")      cli.stream_fps     = parse_uint<uint32_t>(v, cli.stream_fps);
        else if (k == "--stream-bitrate")  cli.stream_bitrate = parse_uint<uint32_t>(v, cli.stream_bitrate);
    }
    return cli;
}

}  // namespace

int main(int argc, char** argv) {
    const Cli cli = parse_cli(argc, argv);

    rtc::InitLogger(rtc::LogLevel::Warning);

    const auto db_path = gw::server::Database::default_path();
    if (cli.reset_db) {
        gw::server::Database::remove_files(db_path);
        std::cerr << "guesswork: --reset-db wiped " << db_path << "\n";
    }
    gw::server::Database               database(db_path);
    gw::server::CameraRepository       cameras(database);
    gw::server::TriggerGroupRepository trigger_groups(database);
    gw::server::ImuConfigRepository    imu_config(database);
    gw::server::NetConfigRepository    net_config(database);
    gw::server::TeensyManager          teensy;
    teensy.start();

    // UDP robot link (chassis speeds in, fused pose out). The Teensy clock
    // view chains the host<->Teensy sync into the link's RIO<->host sync so
    // chassis speeds land on the Teensy clock (docs/ethernet-protocol.md).
    gw::net::RobotLink robot(gw::net::RobotLink::TeensyClockView{
        [&teensy](uint64_t host_ns) { return teensy.host_to_teensy_ns(host_ns); },
        [&teensy](uint64_t teensy_ns) { return teensy.teensy_to_host_ns(teensy_ns); }});
    try {
        const auto nc = net_config.get();
        gw::net::RobotLink::Config lc;
        lc.enabled    = nc.enabled;
        lc.bind_port  = static_cast<uint16_t>(nc.bind_port);
        lc.robot_port = static_cast<uint16_t>(nc.robot_port);
        lc.robot_ip   = nc.robot_ip;
        robot.start(lc);
        if (lc.enabled) {
            std::cerr << "guesswork: robot link listening on UDP :"
                      << lc.bind_port << "\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "guesswork: robot link failed to start: " << e.what()
                  << " (fix via PUT /api/robot/config)\n";
    }
    std::cerr << "guesswork: database at " << db_path << "\n";

    gw::server::StreamParams params{
        cli.stream_width, cli.stream_height, cli.stream_fps, cli.stream_bitrate};
    gw::server::CameraSupervisor supervisor(cameras, params, &teensy);

    // AprilTag pipeline: seed the bundled season layout, then register the
    // detection-consumer factory BEFORE supervisor.start() so cameras that
    // come online at boot get their consumers attached.
    gw::server::FieldLayoutRepository field_layouts(database);
    gw::server::seed_default_field_layout(field_layouts);
    gw::server::ApriltagSupervisor apriltag(cameras, field_layouts, imu_config);
    supervisor.register_consumer_factory(
        [&apriltag](const gw::server::Camera& row) {
            return apriltag.make_consumer(row);
        });

    // OpenVINS stereo VIO: per-camera feeder consumers ride the same slot
    // lifecycle; the runner itself lives in the supervisor and is gated on
    // calibration quality.
    gw::server::VioConfigRepository vio_config(database);
    gw::server::VioSupervisor vio(cameras, imu_config, vio_config, teensy);
    supervisor.register_consumer_factory(
        [&vio](const gw::server::Camera& row) { return vio.make_consumer(row); });

    try {
        supervisor.start();
    } catch (const std::exception& e) {
        std::cerr << "guesswork: supervisor start failed: " << e.what() << "\n";
        return 1;
    }
    vio.reload();  // evaluate gating once the boot-time slots are up

    const auto calibration_root = gw::server::Database::data_dir() / "calibrations";
    gw::server::CalibrationSupervisor calibration(supervisor, cameras, teensy,
                                                  imu_config, calibration_root);
    std::cerr << "guesswork: calibration recordings at " << calibration_root << "\n";

    // GTSAM fusion: subscribes the tag/VIO/odom buses (all live for the
    // supervisors' lifetimes) and ships the fused pose via robot.send_pose.
    // Declared after apriltag/vio/robot so it tears down first.
    gw::server::FusionConfigRepository fusion_config(database);
    gw::server::FusionSupervisor fusion(fusion_config, imu_config, apriltag,
                                        vio, robot);

    // Allan-variance IMU refinement: long static recordings + analysis.
    gw::server::ImuAllanService allan(
        teensy, imu_config, gw::server::Database::data_dir() / "imu_logs");

    const auto started_at = std::chrono::steady_clock::now();
    std::cerr << "guesswork: stream defaults "
              << cli.stream_width << "x" << cli.stream_height
              << " @ " << cli.stream_fps << " fps, "
              << (cli.stream_bitrate / 1000) << " kbps\n";

    gw::server::HttpServer server(cli.port, supervisor, cameras, calibration,
                                  trigger_groups, teensy, imu_config,
                                  field_layouts, apriltag, vio, vio_config,
                                  net_config, robot, fusion, fusion_config,
                                  allan, started_at);

    // Startup banner — printed last so it's the first thing you see in a
    // debug console. In non-embedded (Debug) builds the web UI is served by
    // the Vite dev server, not this binary.
    std::cerr << "\n"
              << "guesswork ready\n"
              << "  API:    http://localhost:" << cli.port << "/api/status\n";
    if (gw::server::has_embedded_web()) {
        std::cerr << "  Web UI: http://localhost:" << cli.port << "\n\n";
    } else {
        std::cerr << "  Web UI: not embedded in this build — run `cd web && npm run dev`\n"
                  << "          then open http://localhost:5173\n\n";
    }

    try {
        server.run();  // Blocks; Crow installs SIGINT/SIGTERM handlers that call stop().
    } catch (const std::exception& e) {
        std::cerr << "guesswork: " << e.what() << "\n";
        return 1;  // normal unwinding — supervisors/Teensy/Spinnaker tear down in order
    }

    return 0;
}
