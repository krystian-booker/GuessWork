#include <chrono>
#include <iostream>
#include <memory>

#include "posest/config/SqliteConfigStore.h"
#include "posest/runtime/Daemon.h"
#include "posest/runtime/ProductionFactories.h"

int main(int argc, const char* const argv[]) {
    posest::runtime::DaemonOptions options;
    try {
        options = posest::runtime::parseDaemonOptions(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n" << posest::runtime::daemonUsage(argv[0]);
        return 2;
    }

    if (options.help) {
        std::cout << posest::runtime::daemonUsage(argv[0]);
        return 0;
    }

    posest::runtime::ProductionCameraFactory camera_factory;
    posest::runtime::ProductionPipelineFactory pipeline_factory;
    posest::runtime::ShutdownSignal shutdown_signal;
    posest::runtime::installProcessSignalHandlers(shutdown_signal);

    try {
        auto config_store =
            std::make_unique<posest::config::SqliteConfigStore>(options.config_path);
        if (options.command != posest::runtime::DaemonCommand::Run) {
            posest::runtime::runConfigCommand(options, *config_store);
            return 0;
        }

        posest::runtime::DaemonController daemon(
            options, std::move(config_store), camera_factory, pipeline_factory);

        if (options.health_once) {
            daemon.loadAndBuild();
            std::cout << posest::runtime::healthToJson(daemon.health()) << "\n";
            return 0;
        }

        daemon.start();
        auto maybe_print_health = [&] {
            if (options.health_interval) {
                std::cout << posest::runtime::healthToJson(daemon.health()) << "\n";
            }
        };

        posest::runtime::waitUntilShutdownRequested(
            shutdown_signal,
            std::chrono::milliseconds(100),
            maybe_print_health);

        daemon.stop(shutdown_signal.signalNumber());
        if (options.health_interval) {
            std::cout << posest::runtime::healthToJson(daemon.health()) << "\n";
        }
        return 0;
    } catch (const std::exception& e) {
        posest::runtime::DaemonHealth health;
        health.state = posest::runtime::DaemonState::Failed;
        health.config_path = options.config_path.string();
        health.last_error = e.what();
        std::cerr << posest::runtime::healthToJson(health) << "\n";
        return 1;
    }
}
