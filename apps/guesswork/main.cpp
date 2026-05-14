#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <limits>
#include <string_view>

#include <rtc/rtc.hpp>

#include "server/calibration_supervisor.hpp"
#include "server/camera_repository.hpp"
#include "server/camera_supervisor.hpp"
#include "server/database.hpp"
#include "server/http_server.hpp"

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
    gw::server::Database         database(db_path);
    gw::server::CameraRepository cameras(database);
    std::cerr << "guesswork: database at " << db_path << "\n";

    gw::server::StreamParams params{
        cli.stream_width, cli.stream_height, cli.stream_fps, cli.stream_bitrate};
    gw::server::CameraSupervisor supervisor(cameras, params);

    try {
        supervisor.start();
    } catch (const std::exception& e) {
        std::cerr << "guesswork: supervisor start failed: " << e.what() << "\n";
        return 1;
    }

    const auto calibration_root = gw::server::Database::data_dir() / "calibrations";
    gw::server::CalibrationSupervisor calibration(supervisor, calibration_root);
    std::cerr << "guesswork: calibration recordings at " << calibration_root << "\n";

    const auto started_at = std::chrono::steady_clock::now();
    std::cerr << "guesswork: stream defaults "
              << cli.stream_width << "x" << cli.stream_height
              << " @ " << cli.stream_fps << " fps, "
              << (cli.stream_bitrate / 1000) << " kbps\n";
    std::cerr << "guesswork: listening on http://localhost:" << cli.port << "\n";

    gw::server::HttpServer server(cli.port, supervisor, cameras, calibration, started_at);
    server.run();  // Blocks; Crow installs SIGINT/SIGTERM handlers that call stop().

    return 0;
}
