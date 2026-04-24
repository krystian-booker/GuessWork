#include "posest/runtime/Daemon.h"

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include <nlohmann/json.hpp>

#include "posest/config/CalibrationParsers.h"

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

std::string requireValue(int& index, int argc, const char* const argv[], const std::string& arg) {
    if (index + 1 >= argc) {
        throw std::invalid_argument(arg + " requires a value");
    }
    return argv[++index];
}

std::string shellQuote(const std::string& value) {
    std::string out = "'";
    for (const char ch : value) {
        if (ch == '\'') {
            out += "'\"'\"'";
        } else {
            out += ch;
        }
    }
    out += "'";
    return out;
}

std::filesystem::path mountPath(const std::filesystem::path& path) {
    const auto parent = path.parent_path();
    return parent.empty() ? std::filesystem::current_path() : std::filesystem::absolute(parent);
}

std::string mountedFilePath(const char* mount_point, const std::filesystem::path& path) {
    return std::string(mount_point) + "/" + path.filename().string();
}

std::string nowIsoUtc() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &time);
#else
    gmtime_r(&time, &tm);
#endif
    std::ostringstream out;
    out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
    return out.str();
}

std::filesystem::path findKalibrCamchain(const std::filesystem::path& output_dir) {
    for (const auto& entry : std::filesystem::directory_iterator(output_dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const auto filename = entry.path().filename().string();
        const auto extension = entry.path().extension().string();
        if (filename.find("camchain") != std::string::npos &&
            (extension == ".yaml" || extension == ".yml")) {
            return entry.path();
        }
    }
    throw std::runtime_error("Kalibr output did not contain a camchain YAML file");
}

void replaceCalibration(
    RuntimeConfig& config,
    CameraCalibrationConfig calibration,
    CameraExtrinsicsConfig extrinsics) {
    bool camera_found = false;
    for (const auto& camera : config.cameras) {
        if (camera.id == calibration.camera_id) {
            camera_found = true;
            break;
        }
    }
    if (!camera_found) {
        throw std::invalid_argument("unknown camera id: " + calibration.camera_id);
    }

    for (auto& entry : config.calibrations) {
        if (entry.camera_id == calibration.camera_id) {
            entry.active = false;
        }
    }

    config.calibrations.erase(
        std::remove_if(
            config.calibrations.begin(),
            config.calibrations.end(),
            [&calibration](const CameraCalibrationConfig& entry) {
                return entry.camera_id == calibration.camera_id &&
                       entry.version == calibration.version;
            }),
        config.calibrations.end());
    config.camera_extrinsics.erase(
        std::remove_if(
            config.camera_extrinsics.begin(),
            config.camera_extrinsics.end(),
            [&extrinsics](const CameraExtrinsicsConfig& entry) {
                return entry.camera_id == extrinsics.camera_id &&
                       entry.version == extrinsics.version;
            }),
        config.camera_extrinsics.end());

    calibration.active = true;
    config.calibrations.push_back(std::move(calibration));
    config.camera_extrinsics.push_back(std::move(extrinsics));
}

void replaceFieldLayout(
    RuntimeConfig& config,
    FieldLayoutConfig layout,
    bool activate) {
    config.field_layouts.erase(
        std::remove_if(
            config.field_layouts.begin(),
            config.field_layouts.end(),
            [&layout](const FieldLayoutConfig& entry) { return entry.id == layout.id; }),
        config.field_layouts.end());
    if (activate) {
        config.active_field_layout_id = layout.id;
    }
    config.field_layouts.push_back(std::move(layout));
}

}  // namespace

DaemonOptions parseDaemonOptions(int argc, const char* const argv[]) {
    DaemonOptions options;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "calibrate-camera") {
            if (options.command != DaemonCommand::Run) {
                throw std::invalid_argument("only one subcommand may be provided");
            }
            options.command = DaemonCommand::CalibrateCamera;
        } else if (arg == "import-field-layout") {
            if (options.command != DaemonCommand::Run) {
                throw std::invalid_argument("only one subcommand may be provided");
            }
            options.command = DaemonCommand::ImportFieldLayout;
        } else if (arg == "--help" || arg == "-h") {
            options.help = true;
        } else if (arg == "--config") {
            options.config_path = requireValue(i, argc, argv, arg);
        } else if (arg == "--health-once") {
            options.health_once = true;
        } else if (arg == "--health-interval-ms") {
            options.health_interval = parseMilliseconds(requireValue(i, argc, argv, arg));
        } else if (arg == "--camera-id") {
            options.calibrate_camera.camera_id = requireValue(i, argc, argv, arg);
        } else if (arg == "--bag") {
            options.calibrate_camera.bag_path = requireValue(i, argc, argv, arg);
        } else if (arg == "--target") {
            options.calibrate_camera.target_path = requireValue(i, argc, argv, arg);
        } else if (arg == "--topic") {
            options.calibrate_camera.topic = requireValue(i, argc, argv, arg);
        } else if (arg == "--output-dir") {
            options.calibrate_camera.output_dir = requireValue(i, argc, argv, arg);
        } else if (arg == "--version") {
            options.calibrate_camera.version = requireValue(i, argc, argv, arg);
        } else if (arg == "--camera-to-robot") {
            options.calibrate_camera.camera_to_robot =
                config::parsePoseCsv(requireValue(i, argc, argv, arg));
            options.calibrate_camera.has_camera_to_robot = true;
        } else if (arg == "--docker-image") {
            options.calibrate_camera.docker_image = requireValue(i, argc, argv, arg);
        } else if (arg == "--field-id") {
            options.import_field_layout.field_id = requireValue(i, argc, argv, arg);
        } else if (arg == "--name") {
            options.import_field_layout.name = requireValue(i, argc, argv, arg);
        } else if (arg == "--file") {
            options.import_field_layout.file_path = requireValue(i, argc, argv, arg);
        } else if (arg == "--activate") {
            options.import_field_layout.activate = true;
        } else {
            throw std::invalid_argument("unknown argument: " + arg);
        }
    }

    if (options.command == DaemonCommand::CalibrateCamera) {
        const auto& command = options.calibrate_camera;
        if (command.camera_id.empty() || command.bag_path.empty() ||
            command.target_path.empty() || command.topic.empty() ||
            command.output_dir.empty() || command.version.empty() ||
            !command.has_camera_to_robot || command.docker_image.empty()) {
            throw std::invalid_argument("calibrate-camera requires camera-id, bag, target, topic, "
                                        "output-dir, version, camera-to-robot, and docker-image");
        }
    } else if (options.command == DaemonCommand::ImportFieldLayout) {
        const auto& command = options.import_field_layout;
        if (command.field_id.empty() || command.name.empty() || command.file_path.empty()) {
            throw std::invalid_argument("import-field-layout requires field-id, name, and file");
        }
    }

    return options;
}

std::string daemonUsage(const char* argv0) {
    std::ostringstream out;
    const char* exe = argv0 ? argv0 : "posest_daemon";
    out << "usage: " << exe
        << " [--config PATH] [--health-once] [--health-interval-ms N]\n"
        << "       " << exe
        << " calibrate-camera --config PATH --camera-id ID --bag BAG --target TARGET "
           "--topic TOPIC --output-dir DIR --version VERSION "
           "--camera-to-robot x,y,z,roll,pitch,yaw --docker-image IMAGE\n"
        << "       " << exe
        << " import-field-layout --config PATH --field-id ID --name NAME --file PATH "
           "[--activate]\n";
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
    const auto now = std::chrono::steady_clock::now();
    auto ageMs = [now](const std::optional<Timestamp>& timestamp) -> nlohmann::json {
        if (!timestamp) {
            return nullptr;
        }
        return std::chrono::duration_cast<std::chrono::milliseconds>(now - *timestamp).count();
    };

    nlohmann::json out = {
        {"state", daemonStateName(health.state)},
        {"config_path", health.config_path},
        {"camera_count", health.camera_count},
        {"pipeline_count", health.pipeline_count},
        {"measurements_dropped", health.measurements_dropped},
        {"measurements_processed", health.measurements_processed},
        {"stale_measurements", health.stale_measurements},
        {"has_latest_pose", health.has_latest_pose},
        {"teensy",
         {
             {"enabled", health.teensy.enabled},
             {"connected", health.teensy.connected},
             {"reconnect_attempts", health.teensy.reconnect_attempts},
             {"successful_connects", health.teensy.successful_connects},
             {"disconnects", health.teensy.disconnects},
             {"crc_failures", health.teensy.crc_failures},
             {"sequence_gaps", health.teensy.sequence_gaps},
             {"invalid_payloads", health.teensy.invalid_payloads},
             {"inbound_imu_samples", health.teensy.inbound_imu_samples},
             {"inbound_wheel_odometry_samples",
              health.teensy.inbound_wheel_odometry_samples},
             {"outbound_frames_queued", health.teensy.outbound_frames_queued},
             {"outbound_frames_sent", health.teensy.outbound_frames_sent},
             {"outbound_frames_dropped", health.teensy.outbound_frames_dropped},
             {"last_receive_age_ms", ageMs(health.teensy.last_receive_time)},
             {"last_transmit_age_ms", ageMs(health.teensy.last_transmit_time)},
             {"last_error", health.teensy.last_error},
             {"time_sync_established", health.teensy.time_sync_established},
             {"time_sync_offset_us", health.teensy.time_sync_offset_us},
             {"time_sync_round_trip_us", health.teensy.time_sync_round_trip_us},
         }},
        {"last_error", health.last_error},
        {"shutdown_signal", health.shutdown_signal},
    };
    return out.dump();
}

std::string buildKalibrDockerCommand(const CalibrateCameraOptions& options) {
    std::ostringstream command;
    command << "docker run --rm "
            << "-v " << shellQuote(mountPath(options.bag_path).string()) << ":/data:ro "
            << "-v " << shellQuote(mountPath(options.target_path).string()) << ":/target:ro "
            << "-v " << shellQuote(std::filesystem::absolute(options.output_dir).string())
            << ":/output "
            << shellQuote(options.docker_image) << " "
            << "kalibr_calibrate_cameras "
            << "--bag " << shellQuote(mountedFilePath("/data", options.bag_path)) << " "
            << "--target " << shellQuote(mountedFilePath("/target", options.target_path)) << " "
            << "--topics " << shellQuote(options.topic) << " "
            << "--models pinhole-radtan";
    return command.str();
}

void runConfigCommand(const DaemonOptions& options, config::IConfigStore& config_store) {
    if (options.command == DaemonCommand::Run) {
        return;
    }

    if (options.command == DaemonCommand::CalibrateCamera) {
        const auto command = buildKalibrDockerCommand(options.calibrate_camera);
        const int rc = std::system(command.c_str());
        if (rc != 0) {
            throw std::runtime_error("Kalibr Docker command failed");
        }

        auto config = config_store.loadRuntimeConfig();
        auto calibration = config::parseKalibrCameraCalibration(
            findKalibrCamchain(options.calibrate_camera.output_dir),
            options.calibrate_camera.camera_id,
            options.calibrate_camera.version,
            true,
            nowIsoUtc(),
            options.calibrate_camera.topic);
        CameraExtrinsicsConfig extrinsics;
        extrinsics.camera_id = options.calibrate_camera.camera_id;
        extrinsics.version = options.calibrate_camera.version;
        extrinsics.camera_to_robot = options.calibrate_camera.camera_to_robot;
        replaceCalibration(config, std::move(calibration), std::move(extrinsics));
        config_store.saveRuntimeConfig(config);
        return;
    }

    auto config = config_store.loadRuntimeConfig();
    auto layout = config::parseWpilibFieldLayout(
        options.import_field_layout.file_path,
        options.import_field_layout.field_id,
        options.import_field_layout.name);
    replaceFieldLayout(config, std::move(layout), options.import_field_layout.activate);
    config_store.saveRuntimeConfig(config);
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
        teensy_ = std::make_shared<teensy::TeensyService>(
            config_.teensy, config_.camera_triggers, *measurement_bus_);
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
        teensy_->start();
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
    if (teensy_) {
        teensy_->stop();
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
    if (teensy_) {
        health_.teensy = teensy_->stats();
    }
}

void DaemonController::markFailed(const std::string& error) {
    std::lock_guard<std::mutex> g(mu_);
    health_.state = DaemonState::Failed;
    health_.last_error = error;
}

}  // namespace posest::runtime
