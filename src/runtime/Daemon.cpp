#include "posest/runtime/Daemon.h"

#include <atomic>
#include <csignal>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include <nlohmann/json.hpp>

namespace posest::runtime {

namespace {

constexpr std::size_t kMeasurementBusCapacity = 4096;
std::atomic<int>* g_process_signal = nullptr;

void signalHandler(int signal_number) {
    if (g_process_signal) {
        g_process_signal->store(signal_number, std::memory_order_release);
    }
}

std::chrono::milliseconds parseMilliseconds(const std::string& value) {
    std::size_t consumed = 0;
    const long long parsed = std::stoll(value, &consumed);
    if (consumed != value.size() || parsed <= 0) {
        throw std::invalid_argument("--health-interval-ms must be a positive integer");
    }
    return std::chrono::milliseconds(parsed);
}

}  // namespace

DaemonOptions parseDaemonOptions(int argc, const char* const argv[]) {
    DaemonOptions options;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            options.help = true;
        } else if (arg == "--config") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--config requires a path");
            }
            options.config_path = argv[++i];
        } else if (arg == "--health-once") {
            options.health_once = true;
        } else if (arg == "--health-interval-ms") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--health-interval-ms requires a value");
            }
            options.health_interval = parseMilliseconds(argv[++i]);
        } else {
            throw std::invalid_argument("unknown argument: " + arg);
        }
    }

    return options;
}

std::string daemonUsage(const char* argv0) {
    std::ostringstream out;
    out << "usage: " << (argv0 ? argv0 : "posest_daemon")
        << " [--config PATH] [--health-once] [--health-interval-ms N]\n";
    return out.str();
}

const char* daemonStateName(DaemonState state) {
    switch (state) {
        case DaemonState::Created: return "created";
        case DaemonState::LoadedConfig: return "loaded_config";
        case DaemonState::Built: return "built";
        case DaemonState::Running: return "running";
        case DaemonState::Stopping: return "stopping";
        case DaemonState::Stopped: return "stopped";
        case DaemonState::Failed: return "failed";
    }
    return "failed";
}

std::string healthToJson(const DaemonHealth& health) {
    nlohmann::json out = {
        {"state", daemonStateName(health.state)},
        {"config_path", health.config_path},
        {"camera_count", health.camera_count},
        {"pipeline_count", health.pipeline_count},
        {"measurements_dropped", health.measurements_dropped},
        {"measurements_processed", health.measurements_processed},
        {"stale_measurements", health.stale_measurements},
        {"has_latest_pose", health.has_latest_pose},
        {"last_error", health.last_error},
        {"shutdown_signal", health.shutdown_signal},
    };
    return out.dump();
}

void ShutdownSignal::request(int signal_number) {
    signal_number_.store(signal_number, std::memory_order_release);
}

bool ShutdownSignal::requested() const {
    return signal_number_.load(std::memory_order_acquire) != 0;
}

int ShutdownSignal::signalNumber() const {
    return signal_number_.load(std::memory_order_acquire);
}

void installProcessSignalHandlers(ShutdownSignal& signal) {
    g_process_signal = &signal.signal_number_;
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
}

void waitUntilShutdownRequested(
    const ShutdownSignal& signal,
    std::chrono::milliseconds poll_interval,
    const std::function<void()>& on_tick) {
    while (!signal.requested()) {
        if (on_tick) {
            on_tick();
        }
        std::this_thread::sleep_for(poll_interval);
    }
}

DaemonController::DaemonController(
    DaemonOptions options,
    std::unique_ptr<config::IConfigStore> config_store,
    ICameraBackendFactory& camera_factory,
    IPipelineFactory& pipeline_factory)
    : options_(std::move(options)),
      config_store_(std::move(config_store)),
      camera_factory_(camera_factory),
      pipeline_factory_(pipeline_factory) {
    if (!config_store_) {
        throw std::invalid_argument("DaemonController requires a config store");
    }
    health_.config_path = options_.config_path.string();
}

DaemonController::~DaemonController() {
    stop();
}

void DaemonController::loadAndBuild() {
    try {
        config_ = config_store_->loadRuntimeConfig();
        {
            std::lock_guard<std::mutex> g(mu_);
            health_.state = DaemonState::LoadedConfig;
        }

        measurement_bus_ = std::make_unique<MeasurementBus>(kMeasurementBusCapacity);
        fusion_ = std::make_unique<fusion::FusionService>(*measurement_bus_);
        teensy_ = std::make_shared<teensy::TeensyService>(config_.teensy);
        fusion_->addOutputSink(teensy_);
        web_ = std::make_unique<WebService>(*config_store_);
        graph_ = std::make_unique<RuntimeGraph>(
            config_, camera_factory_, pipeline_factory_, *measurement_bus_);
        graph_->build();
        built_ = true;

        {
            std::lock_guard<std::mutex> g(mu_);
            health_.state = DaemonState::Built;
        }
        refreshHealth();
    } catch (const std::exception& e) {
        markFailed(e.what());
        throw;
    } catch (...) {
        markFailed("unknown daemon build failure");
        throw;
    }
}

void DaemonController::start() {
    try {
        if (!built_) {
            loadAndBuild();
        }
        if (started_) {
            return;
        }
        fusion_->start();
        graph_->start();
        started_ = true;
        {
            std::lock_guard<std::mutex> g(mu_);
            health_.state = DaemonState::Running;
        }
        refreshHealth();
    } catch (const std::exception& e) {
        markFailed(e.what());
        stop();
        throw;
    } catch (...) {
        markFailed("unknown daemon startup failure");
        stop();
        throw;
    }
}

void DaemonController::stop(int shutdown_signal) {
    const bool should_stop = started_ || built_;
    if (!should_stop) {
        return;
    }

    {
        std::lock_guard<std::mutex> g(mu_);
        if (health_.state != DaemonState::Failed) {
            health_.state = DaemonState::Stopping;
        }
        if (shutdown_signal != 0) {
            health_.shutdown_signal = shutdown_signal;
        }
    }

    if (graph_) {
        graph_->stop();
    }
    if (fusion_) {
        fusion_->stop();
    }
    started_ = false;

    {
        std::lock_guard<std::mutex> g(mu_);
        if (health_.state != DaemonState::Failed) {
            health_.state = DaemonState::Stopped;
        }
    }
    refreshHealth();
}

DaemonHealth DaemonController::health() const {
    const_cast<DaemonController*>(this)->refreshHealth();
    std::lock_guard<std::mutex> g(mu_);
    return health_;
}

void DaemonController::refreshHealth() {
    std::lock_guard<std::mutex> g(mu_);
    if (graph_) {
        health_.camera_count = graph_->cameraCount();
        health_.pipeline_count = graph_->pipelineCount();
    }
    if (measurement_bus_) {
        health_.measurements_dropped = measurement_bus_->droppedNewestCount();
    }
    if (fusion_) {
        const auto stats = fusion_->stats();
        health_.measurements_processed = stats.measurements_processed;
        health_.stale_measurements = stats.stale_measurements;
        health_.has_latest_pose = fusion_->latestEstimate().has_value();
    }
}

void DaemonController::markFailed(const std::string& error) {
    std::lock_guard<std::mutex> g(mu_);
    health_.state = DaemonState::Failed;
    health_.last_error = error;
}

}  // namespace posest::runtime
