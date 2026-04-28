#include "posest/runtime/Daemon.h"

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include <nlohmann/json.hpp>

#include "posest/calibration/CalibrationRecorder.h"
#include "posest/calibration/CalibrationTargetWriter.h"
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

void upsertCalibrationTarget(
    RuntimeConfig& config,
    CalibrationTargetConfig target) {
    config.calibration_targets.erase(
        std::remove_if(
            config.calibration_targets.begin(),
            config.calibration_targets.end(),
            [&target](const CalibrationTargetConfig& entry) {
                return entry.id == target.id;
            }),
        config.calibration_targets.end());
    config.calibration_targets.push_back(std::move(target));
}

const CalibrationTargetConfig& findCalibrationTarget(
    const RuntimeConfig& config,
    const std::string& id) {
    const auto it = std::find_if(
        config.calibration_targets.begin(),
        config.calibration_targets.end(),
        [&id](const CalibrationTargetConfig& entry) { return entry.id == id; });
    if (it == config.calibration_targets.end()) {
        throw std::invalid_argument("unknown calibration target id: " + id);
    }
    return *it;
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

std::string resolvedKalibrDockerImage(
    const std::string& cli_image,
    const RuntimeConfig& config) {
    if (!cli_image.empty()) {
        return cli_image;
    }
    if (const char* env = std::getenv("POSEST_KALIBR_DOCKER_IMAGE")) {
        if (env[0] != '\0') {
            return env;
        }
    }
    if (!config.calibration_tools.docker_image.empty()) {
        return config.calibration_tools.docker_image;
    }
    return "kalibr:latest";
}

std::vector<std::string> loadDatasetCameraIds(const std::filesystem::path& dataset_dir) {
    std::ifstream in(dataset_dir / "session.json");
    if (!in) {
        throw std::runtime_error("dataset is missing session.json: " + dataset_dir.string());
    }
    const auto json = nlohmann::json::parse(in);
    return json.at("camera_ids").get<std::vector<std::string>>();
}

std::filesystem::path findKalibrImuCamchain(const std::filesystem::path& output_dir) {
    for (const auto& entry : std::filesystem::directory_iterator(output_dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const auto filename = entry.path().filename().string();
        const auto extension = entry.path().extension().string();
        if (filename.find("camchain-imucam") != std::string::npos &&
            (extension == ".yaml" || extension == ".yml")) {
            return entry.path();
        }
    }
    throw std::runtime_error("Kalibr output did not contain a camchain-imucam YAML file");
}

void exportActiveCamchain(
    const RuntimeConfig& config,
    const std::vector<std::string>& camera_ids,
    const std::filesystem::path& path) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("failed to create camchain: " + path.string());
    }

    std::size_t index = 0;
    for (const auto& camera_id : camera_ids) {
        const auto it = std::find_if(
            config.calibrations.begin(),
            config.calibrations.end(),
            [&camera_id](const CameraCalibrationConfig& calibration) {
                return calibration.camera_id == camera_id && calibration.active;
            });
        if (it == config.calibrations.end()) {
            throw std::runtime_error("camera has no active calibration: " + camera_id);
        }
        out << "cam" << index++ << ":\n"
            << "  camera_model: " << it->camera_model << "\n"
            << "  intrinsics: [" << it->fx << ", " << it->fy << ", " << it->cx << ", "
            << it->cy << "]\n"
            << "  distortion_model: " << it->distortion_model << "\n"
            << "  distortion_coeffs: [";
        for (std::size_t i = 0; i < it->distortion_coefficients.size(); ++i) {
            if (i != 0) out << ", ";
            out << it->distortion_coefficients[i];
        }
        out << "]\n"
            << "  resolution: [" << it->image_width << ", " << it->image_height << "]\n"
            << "  rostopic: /posest/" << camera_id << "/image_raw\n";
    }
}

void replaceCameraImuCalibration(
    RuntimeConfig& config,
    CameraImuCalibrationConfig calibration) {
    for (auto& entry : config.camera_imu_calibrations) {
        if (entry.camera_id == calibration.camera_id) {
            entry.active = false;
        }
    }
    config.camera_imu_calibrations.erase(
        std::remove_if(
            config.camera_imu_calibrations.begin(),
            config.camera_imu_calibrations.end(),
            [&calibration](const CameraImuCalibrationConfig& entry) {
                return entry.camera_id == calibration.camera_id &&
                       entry.version == calibration.version;
            }),
        config.camera_imu_calibrations.end());
    calibration.active = true;
    config.camera_imu_calibrations.push_back(std::move(calibration));
}

void rememberDataset(
    RuntimeConfig& config,
    const std::filesystem::path& path,
    const std::vector<std::string>& camera_ids,
    double duration_s) {
    const std::string id = std::filesystem::absolute(path).string();
    config.kalibr_datasets.erase(
        std::remove_if(
            config.kalibr_datasets.begin(),
            config.kalibr_datasets.end(),
            [&id](const KalibrDatasetConfig& entry) { return entry.id == id; }),
        config.kalibr_datasets.end());
    config.kalibr_datasets.push_back({
        id,
        std::filesystem::absolute(path).string(),
        nowIsoUtc(),
        duration_s,
        camera_ids,
    });
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
        } else if (arg == "record-kalibr-dataset") {
            if (options.command != DaemonCommand::Run) {
                throw std::invalid_argument("only one subcommand may be provided");
            }
            options.command = DaemonCommand::RecordKalibrDataset;
        } else if (arg == "make-kalibr-bag") {
            if (options.command != DaemonCommand::Run) {
                throw std::invalid_argument("only one subcommand may be provided");
            }
            options.command = DaemonCommand::MakeKalibrBag;
        } else if (arg == "calibrate-camera-imu") {
            if (options.command != DaemonCommand::Run) {
                throw std::invalid_argument("only one subcommand may be provided");
            }
            options.command = DaemonCommand::CalibrateCameraImu;
        } else if (arg == "import-calibration-target") {
            if (options.command != DaemonCommand::Run) {
                throw std::invalid_argument("only one subcommand may be provided");
            }
            options.command = DaemonCommand::ImportCalibrationTarget;
        } else if (arg == "--help" || arg == "-h") {
            options.help = true;
        } else if (arg == "--config") {
            options.config_path = requireValue(i, argc, argv, arg);
        } else if (arg == "--health-once") {
            options.health_once = true;
        } else if (arg == "--health-interval-ms") {
            options.health_interval = parseMilliseconds(requireValue(i, argc, argv, arg));
        } else if (arg == "--camera-id") {
            const std::string camera_id = requireValue(i, argc, argv, arg);
            options.calibrate_camera.camera_id = camera_id;
            options.record_kalibr_dataset.camera_ids.push_back(camera_id);
        } else if (arg == "--bag") {
            const std::string path = requireValue(i, argc, argv, arg);
            options.calibrate_camera.bag_path = path;
            options.make_kalibr_bag.bag_path = path;
        } else if (arg == "--target") {
            const std::string path = requireValue(i, argc, argv, arg);
            options.calibrate_camera.target_path = path;
            options.calibrate_camera_imu.target_path = path;
        } else if (arg == "--topic") {
            options.calibrate_camera.topic = requireValue(i, argc, argv, arg);
        } else if (arg == "--output-dir") {
            const std::string path = requireValue(i, argc, argv, arg);
            options.calibrate_camera.output_dir = path;
            options.record_kalibr_dataset.output_dir = path;
        } else if (arg == "--version") {
            const std::string version = requireValue(i, argc, argv, arg);
            options.calibrate_camera.version = version;
            options.calibrate_camera_imu.version = version;
        } else if (arg == "--camera-to-robot") {
            options.calibrate_camera.camera_to_robot =
                config::parsePoseCsv(requireValue(i, argc, argv, arg));
            options.calibrate_camera.has_camera_to_robot = true;
        } else if (arg == "--docker-image") {
            const std::string image = requireValue(i, argc, argv, arg);
            options.calibrate_camera.docker_image = image;
            options.make_kalibr_bag.docker_image = image;
            options.calibrate_camera_imu.docker_image = image;
        } else if (arg == "--field-id") {
            options.import_field_layout.field_id = requireValue(i, argc, argv, arg);
        } else if (arg == "--name") {
            options.import_field_layout.name = requireValue(i, argc, argv, arg);
        } else if (arg == "--file") {
            options.import_field_layout.file_path = requireValue(i, argc, argv, arg);
        } else if (arg == "--activate") {
            options.import_field_layout.activate = true;
        } else if (arg == "--duration-s") {
            options.record_kalibr_dataset.duration_s =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--dataset") {
            const std::string path = requireValue(i, argc, argv, arg);
            options.make_kalibr_bag.dataset_dir = path;
            options.calibrate_camera_imu.dataset_dir = path;
        } else if (arg == "--imu") {
            options.calibrate_camera_imu.imu_path = requireValue(i, argc, argv, arg);
        } else if (arg == "--target-id") {
            const std::string id = requireValue(i, argc, argv, arg);
            options.calibrate_camera.target_id = id;
            options.import_calibration_target.target_id = id;
        } else if (arg == "--from-yaml") {
            options.import_calibration_target.from_yaml = requireValue(i, argc, argv, arg);
        } else if (arg == "--type") {
            options.import_calibration_target.type = requireValue(i, argc, argv, arg);
        } else if (arg == "--rows") {
            options.import_calibration_target.rows =
                std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--cols") {
            options.import_calibration_target.cols =
                std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--tag-size-m") {
            options.import_calibration_target.tag_size_m =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--tag-spacing-ratio") {
            options.import_calibration_target.tag_spacing_ratio =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--square-size-m") {
            options.import_calibration_target.square_size_m =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--tag-family") {
            options.import_calibration_target.tag_family =
                requireValue(i, argc, argv, arg);
        } else if (arg == "--notes") {
            options.import_calibration_target.notes = requireValue(i, argc, argv, arg);
        } else {
            throw std::invalid_argument("unknown argument: " + arg);
        }
    }

    if (options.command == DaemonCommand::CalibrateCamera) {
        const auto& command = options.calibrate_camera;
        if (command.camera_id.empty() || command.bag_path.empty() ||
            (command.target_path.empty() && command.target_id.empty()) ||
            command.topic.empty() || command.output_dir.empty() ||
            command.version.empty() || !command.has_camera_to_robot) {
            throw std::invalid_argument(
                "calibrate-camera requires camera-id, bag, target/target-id, topic, "
                "output-dir, version, and camera-to-robot");
        }
    } else if (options.command == DaemonCommand::ImportCalibrationTarget) {
        const auto& command = options.import_calibration_target;
        if (command.target_id.empty()) {
            throw std::invalid_argument(
                "import-calibration-target requires --target-id");
        }
        if (command.from_yaml.empty()) {
            if (command.type.empty() || command.rows <= 0 || command.cols <= 0) {
                throw std::invalid_argument(
                    "import-calibration-target without --from-yaml requires "
                    "--type, --rows, --cols");
            }
        }
    } else if (options.command == DaemonCommand::ImportFieldLayout) {
        const auto& command = options.import_field_layout;
        if (command.field_id.empty() || command.name.empty() || command.file_path.empty()) {
            throw std::invalid_argument("import-field-layout requires field-id, name, and file");
        }
    } else if (options.command == DaemonCommand::RecordKalibrDataset) {
        const auto& command = options.record_kalibr_dataset;
        if (command.camera_ids.empty() || command.output_dir.empty() ||
            !(command.duration_s > 0.0)) {
            throw std::invalid_argument(
                "record-kalibr-dataset requires camera-id, output-dir, and duration-s");
        }
    } else if (options.command == DaemonCommand::MakeKalibrBag) {
        const auto& command = options.make_kalibr_bag;
        if (command.dataset_dir.empty() || command.bag_path.empty()) {
            throw std::invalid_argument("make-kalibr-bag requires dataset and bag");
        }
    } else if (options.command == DaemonCommand::CalibrateCameraImu) {
        const auto& command = options.calibrate_camera_imu;
        if (command.dataset_dir.empty() || command.target_path.empty() ||
            command.imu_path.empty() || command.version.empty()) {
            throw std::invalid_argument(
                "calibrate-camera-imu requires dataset, target, imu, and version");
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
        << " calibrate-camera --config PATH --camera-id ID --bag BAG "
           "(--target TARGET | --target-id ID) "
           "--topic TOPIC --output-dir DIR --version VERSION "
           "--camera-to-robot x,y,z,roll,pitch,yaw --docker-image IMAGE\n"
        << "       " << exe
        << " import-field-layout --config PATH --field-id ID --name NAME --file PATH "
           "[--activate]\n"
        << "       " << exe
        << " record-kalibr-dataset --config PATH --camera-id ID... --duration-s N "
           "--output-dir DIR\n"
        << "       " << exe
        << " make-kalibr-bag --config PATH --dataset DIR --bag OUT.bag "
           "[--docker-image IMAGE]\n"
        << "       " << exe
        << " calibrate-camera-imu --config PATH --dataset DIR --target TARGET.yaml "
           "--imu IMU.yaml --version VERSION [--docker-image IMAGE]\n"
        << "       " << exe
        << " import-calibration-target --config PATH --target-id ID "
           "(--from-yaml PATH | --type TYPE --rows N --cols N "
           "[--tag-size-m X] [--tag-spacing-ratio X] [--square-size-m X] "
           "[--tag-family STR] [--notes ...])\n";
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

    nlohmann::json cameras_json = nlohmann::json::array();
    for (const auto& camera : health.cameras) {
        cameras_json.push_back({
            {"camera_id", camera.camera_id},
            {"state", connectionStateToString(camera.live.state)},
            {"disconnect_count", camera.live.disconnect_count},
            {"reconnect_attempts", camera.live.reconnect_attempts},
            {"successful_connects", camera.live.successful_connects},
            {"measured_fps", camera.live.measured_fps},
            {"last_frame_age_ms", ageMs(camera.live.last_frame_time)},
            {"last_error", camera.live.last_error},
        });
    }

    nlohmann::json apriltag_pipelines_json = nlohmann::json::array();
    for (const auto& s : health.apriltag_pipelines) {
        apriltag_pipelines_json.push_back({
            {"pipeline_id", s.pipeline_id},
            {"frames_processed", s.frames_processed},
            {"frames_no_detection", s.frames_no_detection},
            {"frames_dropped_no_calibration", s.frames_dropped_no_calibration},
            {"frames_dropped_by_ambiguity", s.frames_dropped_by_ambiguity},
            {"frames_solved_single", s.frames_solved_single},
            {"frames_solved_multi", s.frames_solved_multi},
            {"mailbox_drops", s.mailbox_drops},
            {"last_solve_latency_us", s.last_solve_latency_us},
            {"max_solve_latency_us", s.max_solve_latency_us},
            {"last_reprojection_rms_px", s.last_reprojection_rms_px},
        });
    }

    nlohmann::json out = {
        {"state", daemonStateName(health.state)},
        {"config_path", health.config_path},
        {"camera_count", health.camera_count},
        {"pipeline_count", health.pipeline_count},
        {"cameras", cameras_json},
        {"apriltag_pipelines", apriltag_pipelines_json},
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
             {"inbound_chassis_speeds_samples",
              health.teensy.inbound_chassis_speeds_samples},
             {"inbound_chassis_speeds_dropped_pre_sync",
              health.teensy.inbound_chassis_speeds_dropped_pre_sync},
             {"outbound_frames_queued", health.teensy.outbound_frames_queued},
             {"outbound_frames_sent", health.teensy.outbound_frames_sent},
             {"outbound_frames_dropped", health.teensy.outbound_frames_dropped},
             {"last_receive_age_ms", ageMs(health.teensy.last_receive_time)},
             {"last_transmit_age_ms", ageMs(health.teensy.last_transmit_time)},
             {"last_error", health.teensy.last_error},
             {"time_sync_established", health.teensy.time_sync_established},
             {"time_sync_offset_us", health.teensy.time_sync_offset_us},
             {"time_sync_round_trip_us", health.teensy.time_sync_round_trip_us},
             {"time_sync_samples_accepted", health.teensy.time_sync_samples_accepted},
             {"time_sync_samples_rejected", health.teensy.time_sync_samples_rejected},
             {"time_sync_skew_ppm", health.teensy.time_sync_skew_ppm},
             {"rio_time_sync_established", health.teensy.rio_time_sync_established},
             {"rio_to_teensy_offset_us", health.teensy.rio_to_teensy_offset_us},
             // F-6 firmware-side fused-pose latency window.
             {"fused_pose_decode_to_tx_min_us",
              health.teensy.fused_pose_decode_to_tx_min_us},
             {"fused_pose_decode_to_tx_avg_us",
              health.teensy.fused_pose_decode_to_tx_avg_us},
             {"fused_pose_decode_to_tx_max_us",
              health.teensy.fused_pose_decode_to_tx_max_us},
             {"fused_pose_latency_samples",
              health.teensy.fused_pose_latency_samples},
         }},
        {"last_error", health.last_error},
        {"shutdown_signal", health.shutdown_signal},
    };

    // Fusion observability. Each histogram becomes a sub-object with the
    // standard percentile set. count == 0 indicates the histogram has not
    // accumulated samples yet (e.g. before the first chassis update).
    auto histogramJson = [](const util::LatencyHistogram::Snapshot& s) {
        return nlohmann::json{
            {"count", s.count},
            {"min_us", s.min_us},
            {"avg_us", s.avg_us},
            {"p50_us", s.p50_us},
            {"p95_us", s.p95_us},
            {"p99_us", s.p99_us},
            {"max_us", s.max_us},
        };
    };
    out["fusion"] = {
        {"measurements_processed", health.fusion.measurements_processed},
        {"stale_measurements", health.fusion.stale_measurements},
        {"measurements_vio_processed", health.fusion.measurements_vio_processed},
        {"measurements_vio_skipped_no_tracking",
         health.fusion.measurements_vio_skipped_no_tracking},
        {"graph_factor_count", health.fusion.graph_factor_count},
        {"graph_variable_count", health.fusion.graph_variable_count},
        {"last_update_wall_clock_us", health.fusion.last_update_wall_clock_us},
        {"graph_update_us", histogramJson(health.fusion.graph_update_us)},
        {"publish_us", histogramJson(health.fusion.publish_us)},
        // Phase C counters — surfaced unconditionally; non-zero only when
        // enable_imu_preintegration is on and the state machine has reached
        // kRunning (or kCalibratingBias for bias_calibrations_completed).
        {"keyframes_committed", health.fusion.keyframes_committed},
        {"imu_factors_committed", health.fusion.imu_factors_committed},
        {"imu_out_of_order", health.fusion.imu_out_of_order},
        {"imu_resets", health.fusion.imu_resets},
        {"bias_calibrations_completed", health.fusion.bias_calibrations_completed},
        // F-3 / F-1 counters.
        {"chassis_speed_gated", health.fusion.chassis_speed_gated},
        {"config_reloads_applied", health.fusion.config_reloads_applied},
        {"config_reloads_structural_skipped",
         health.fusion.config_reloads_structural_skipped},
        // F-4 counters. Only ever non-zero when Phase C is on and the state
        // machine has reached kRunning; safe to emit unconditionally.
        {"slip_disagreement_events", health.fusion.slip_disagreement_events},
        {"slip_disagreement_inflations",
         health.fusion.slip_disagreement_inflations},
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

std::string buildMakeKalibrBagDockerCommand(
    const MakeKalibrBagOptions& options,
    const std::string& docker_image) {
    const auto script_dir = std::filesystem::absolute("scripts/kalibr");
    std::ostringstream command;
    command << "docker run --rm "
            << "-v " << shellQuote(std::filesystem::absolute(options.dataset_dir).string())
            << ":/dataset:ro "
            << "-v " << shellQuote(mountPath(options.bag_path).string()) << ":/out "
            << "-v " << shellQuote(script_dir.string()) << ":/tools:ro "
            << shellQuote(docker_image) << " "
            << "bash -lc "
            << shellQuote("python3 /tools/make_rosbag.py --dataset /dataset --bag " +
                          mountedFilePath("/out", options.bag_path));
    return command.str();
}

std::string buildCalibrateCameraImuDockerCommand(
    const CalibrateCameraImuOptions& options,
    const std::filesystem::path& bag_path,
    const std::filesystem::path& camchain_path,
    const std::string& docker_image) {
    std::ostringstream command;
    command << "docker run --rm "
            << "-v " << shellQuote(std::filesystem::absolute(options.dataset_dir).string())
            << ":/dataset "
            << "-v " << shellQuote(mountPath(options.target_path).string()) << ":/target:ro "
            << "-v " << shellQuote(mountPath(options.imu_path).string()) << ":/imu:ro "
            << shellQuote(docker_image) << " "
            << "bash -lc "
            << shellQuote("source /catkin_ws/devel/setup.bash 2>/dev/null || true; "
                          "kalibr_calibrate_imu_camera --bag " +
                          mountedFilePath("/dataset", bag_path) + " --cam " +
                          mountedFilePath("/dataset", camchain_path) + " --imu " +
                          mountedFilePath("/imu", options.imu_path) + " --target " +
                          mountedFilePath("/target", options.target_path));
    return command.str();
}

void runConfigCommand(
    const DaemonOptions& options,
    config::IConfigStore& config_store,
    ICameraBackendFactory& camera_factory) {
    if (options.command == DaemonCommand::Run) {
        return;
    }

    if (options.command == DaemonCommand::CalibrateCamera) {
        auto config = config_store.loadRuntimeConfig();
        auto kalibr_options = options.calibrate_camera;
        kalibr_options.docker_image =
            resolvedKalibrDockerImage(kalibr_options.docker_image, config);
        // Resolve --target-id to a materialized target.yaml when --target was
        // not explicitly given. Path precedence: --target wins over --target-id.
        if (kalibr_options.target_path.empty() && !kalibr_options.target_id.empty()) {
            const auto& target = findCalibrationTarget(config, kalibr_options.target_id);
            std::filesystem::create_directories(kalibr_options.output_dir);
            const auto materialized =
                kalibr_options.output_dir / (target.id + "-target.yaml");
            calibration::writeKalibrTargetYaml(target, materialized);
            kalibr_options.target_path = materialized;
        }
        const auto command = buildKalibrDockerCommand(kalibr_options);
        const int rc = std::system(command.c_str());
        if (rc != 0) {
            throw std::runtime_error("Kalibr Docker command failed");
        }

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

    if (options.command == DaemonCommand::ImportFieldLayout) {
        auto config = config_store.loadRuntimeConfig();
        auto layout = config::parseWpilibFieldLayout(
            options.import_field_layout.file_path,
            options.import_field_layout.field_id,
            options.import_field_layout.name);
        replaceFieldLayout(config, std::move(layout), options.import_field_layout.activate);
        config_store.saveRuntimeConfig(config);
        return;
    }

    if (options.command == DaemonCommand::ImportCalibrationTarget) {
        auto config = config_store.loadRuntimeConfig();
        const auto& cmd = options.import_calibration_target;
        CalibrationTargetConfig target;
        if (!cmd.from_yaml.empty()) {
            target = config::parseKalibrTargetYaml(cmd.from_yaml, cmd.target_id);
            target.notes = cmd.notes;
        } else {
            target.id = cmd.target_id;
            target.type = cmd.type;
            target.rows = cmd.rows;
            target.cols = cmd.cols;
            target.tag_size_m = cmd.tag_size_m;
            target.tag_spacing_ratio = cmd.tag_spacing_ratio;
            target.square_size_m = cmd.square_size_m;
            target.tag_family = cmd.tag_family;
            target.notes = cmd.notes;
        }
        upsertCalibrationTarget(config, std::move(target));
        config_store.saveRuntimeConfig(config);
        return;
    }

    if (options.command == DaemonCommand::RecordKalibrDataset) {
        auto config = config_store.loadRuntimeConfig();
        calibration::CalibrationRecorder recorder({
            options.record_kalibr_dataset.output_dir,
            options.record_kalibr_dataset.camera_ids,
            config.camera_triggers,
            options.record_kalibr_dataset.duration_s,
        });
        std::vector<std::shared_ptr<IFrameProducer>> cameras;
        auto recorder_consumer =
            std::shared_ptr<IFrameConsumer>(&recorder, [](IFrameConsumer*) {});
        for (const auto& camera_id : options.record_kalibr_dataset.camera_ids) {
            const auto camera_it = std::find_if(
                config.cameras.begin(),
                config.cameras.end(),
                [&camera_id](const CameraConfig& camera) { return camera.id == camera_id; });
            if (camera_it == config.cameras.end() || !camera_it->enabled) {
                throw std::runtime_error("selected camera is missing or disabled: " + camera_id);
            }
            auto camera = camera_factory.createCamera(*camera_it);
            camera->addConsumer(recorder_consumer);
            cameras.push_back(camera);
        }
        recorder.start();
        // Kalibr recording doesn't run the VIO companion — leaving the
        // VioConfig at its default (enabled=false) makes the firmware path
        // a no-op for this transient session.
        teensy::TeensyService teensy(
            config.teensy, config.camera_triggers, recorder);
        teensy.start();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!teensy.stats().time_sync_established &&
               std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!teensy.stats().time_sync_established) {
            teensy.stop();
            recorder.stop();
            throw std::runtime_error("Teensy time sync was not established");
        }

        for (auto& camera : cameras) {
            if (camera->start() != ProducerState::Running) {
                throw std::runtime_error(
                    "Failed to start camera " + camera->id() +
                    " for kalibr recording (already in terminal state)");
            }
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(
            options.record_kalibr_dataset.duration_s));
        for (auto& camera : cameras) camera->stop();
        teensy.stop();
        recorder.stop();
        recorder.throwIfUnacceptable();
        rememberDataset(
            config,
            options.record_kalibr_dataset.output_dir,
            options.record_kalibr_dataset.camera_ids,
            options.record_kalibr_dataset.duration_s);
        config_store.saveRuntimeConfig(config);
        return;
    }

    if (options.command == DaemonCommand::MakeKalibrBag) {
        const auto config = config_store.loadRuntimeConfig();
        const auto image = resolvedKalibrDockerImage(options.make_kalibr_bag.docker_image, config);
        const int rc = std::system(
            buildMakeKalibrBagDockerCommand(options.make_kalibr_bag, image).c_str());
        if (rc != 0) {
            throw std::runtime_error("Kalibr bag Docker command failed");
        }
        return;
    }

    auto config = config_store.loadRuntimeConfig();
    const auto image = resolvedKalibrDockerImage(options.calibrate_camera_imu.docker_image, config);
    const auto camera_ids = loadDatasetCameraIds(options.calibrate_camera_imu.dataset_dir);
    const auto bag_path = options.calibrate_camera_imu.dataset_dir / "kalibr.bag";
    MakeKalibrBagOptions bag_options;
    bag_options.dataset_dir = options.calibrate_camera_imu.dataset_dir;
    bag_options.bag_path = bag_path;
    const int bag_rc =
        std::system(buildMakeKalibrBagDockerCommand(bag_options, image).c_str());
    if (bag_rc != 0) {
        throw std::runtime_error("Kalibr bag Docker command failed");
    }

    const auto camchain_path = options.calibrate_camera_imu.dataset_dir / "input-camchain.yaml";
    exportActiveCamchain(config, camera_ids, camchain_path);
    const int kalibr_rc = std::system(
        buildCalibrateCameraImuDockerCommand(
            options.calibrate_camera_imu, bag_path, camchain_path, image).c_str());
    if (kalibr_rc != 0) {
        throw std::runtime_error("Kalibr camera-IMU Docker command failed");
    }

    const auto result_path = findKalibrImuCamchain(options.calibrate_camera_imu.dataset_dir);
    for (const auto& camera_id : camera_ids) {
        replaceCameraImuCalibration(
            config,
            config::parseKalibrCameraImuCalibration(
                result_path,
                camera_id,
                options.calibrate_camera_imu.version,
                true,
                nowIsoUtc(),
                "/posest/" + camera_id + "/image_raw"));
    }
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
        std::unordered_map<std::int32_t, std::string> pin_to_camera;
        for (const auto& trigger : config_.camera_triggers) {
            pin_to_camera[trigger.teensy_pin] = trigger.camera_id;
        }
        trigger_cache_ = std::make_shared<CameraTriggerCache>(std::move(pin_to_camera));
        if (config_.vio.enabled && !config_.vio.vio_camera_id.empty()) {
            tof_cache_ =
                std::make_shared<ToFSampleCache>(config_.vio.vio_camera_id);
        } else {
            tof_cache_.reset();
        }
        fusion_ = std::make_unique<fusion::FusionService>(
            *measurement_bus_,
            fusion::buildFusionConfig(config_));
        teensy_ = std::make_shared<teensy::TeensyService>(
            config_.teensy, config_.camera_triggers,
            *measurement_bus_,
            teensy::makePosixSerialTransport, trigger_cache_,
            config_.vio, tof_cache_);
        fusion_->addOutputSink(teensy_);
        // F-1: hand new fusion config to the running service whenever the web
        // layer saves a RuntimeConfig. The callback runs on the web thread but
        // only stages a pending swap under FusionService::mu_ — the worker
        // applies it at the top of its next process() iteration.
        web_ = std::make_unique<WebService>(
            *config_store_,
            [fusion = fusion_.get()](const RuntimeConfig& cfg) {
                if (fusion) {
                    fusion->applyConfig(fusion::buildFusionConfig(cfg));
                }
            });
        graph_ = std::make_unique<RuntimeGraph>(
            config_, camera_factory_, pipeline_factory_, *measurement_bus_);
        graph_->build();
        for (const auto& camera : graph_->cameraProducers()) {
            camera->setTriggerCache(trigger_cache_);
            if (tof_cache_) {
                camera->setToFSampleCache(tof_cache_);
            }
        }
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
        // F-7: persist a freshly-calibrated IMU bias back to SQLite before
        // stopping the worker. Returns nullopt if the boot stationary window
        // never produced a fresh mean (so we don't clobber a previously-good
        // seed with identity). Save failures must not block shutdown — the
        // next boot recalibrates anyway.
        if (auto bias = fusion_->currentBiasIfTrusted(); bias && config_store_) {
            try {
                config_.fusion.persisted_bias = *bias;
                config_store_->saveRuntimeConfig(config_);
            } catch (const std::exception&) {
                // Swallow: shutdown must always make forward progress.
            }
        }
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
    std::vector<CameraLiveStats> camera_stats;
    std::vector<pipelines::AprilTagPipelineStats> apriltag_stats;
    if (graph_) {
        const auto camera_producers = graph_->cameraProducers();
        camera_stats.reserve(camera_producers.size());
        for (const auto& camera : camera_producers) {
            camera_stats.push_back({camera->id(), camera->liveStats()});
        }
        const auto vision_pipelines = graph_->pipelines();
        apriltag_stats.reserve(vision_pipelines.size());
        for (const auto& pipeline : vision_pipelines) {
            const auto value = pipeline->pipelineStats();
            if (const auto* tag_stats =
                    std::get_if<pipelines::AprilTagPipelineStats>(&value)) {
                apriltag_stats.push_back(*tag_stats);
            }
        }
    }

    std::lock_guard<std::mutex> g(mu_);
    if (graph_) {
        health_.camera_count = graph_->cameraCount();
        health_.pipeline_count = graph_->pipelineCount();
    }
    if (measurement_bus_) {
        health_.measurements_dropped = measurement_bus_->droppedNewestCount();
    }
    if (fusion_) {
        auto stats = fusion_->stats();
        health_.measurements_processed = stats.measurements_processed;
        health_.stale_measurements = stats.stale_measurements;
        health_.has_latest_pose = fusion_->latestEstimate().has_value();
        health_.fusion = std::move(stats);
    }
    if (teensy_) {
        health_.teensy = teensy_->stats();
    }
    health_.cameras = std::move(camera_stats);
    health_.apriltag_pipelines = std::move(apriltag_stats);
}

void DaemonController::markFailed(const std::string& error) {
    std::lock_guard<std::mutex> g(mu_);
    health_.state = DaemonState::Failed;
    health_.last_error = error;
}

}  // namespace posest::runtime
