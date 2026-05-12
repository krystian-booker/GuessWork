#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string_view>

#include "producer/spinnaker_producer.hpp"
#include "server/http_server.hpp"
#include "server/pipeline_stats.hpp"

namespace {

uint16_t parse_port(int argc, char** argv) {
    constexpr uint16_t kDefault = 8080;
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string_view(argv[i]) == "--port") {
            const int v = std::atoi(argv[i + 1]);
            if (v <= 0 || v > 65535) {
                std::cerr << "guesswork: --port out of range; using " << kDefault << "\n";
                return kDefault;
            }
            return static_cast<uint16_t>(v);
        }
    }
    return kDefault;
}

}  // namespace

int main(int argc, char** argv) {
    const uint16_t port = parse_port(argc, argv);

    gw::SpinnakerProducer producer("cam0");
    try {
        producer.start();
    } catch (const std::exception& e) {
        std::cerr << "guesswork: producer start failed: " << e.what() << "\n";
        return 1;
    }

    gw::server::PipelineStatsView stats(producer, std::chrono::steady_clock::now());

    std::cerr << "guesswork: listening on http://localhost:" << port << "\n";
    gw::server::HttpServer server(port, stats);
    server.run();  // Blocks; Crow installs SIGINT/SIGTERM handlers that call stop().

    producer.stop();
    return 0;
}
