#pragma once

#include <chrono>
#include <atomic>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "posest/CameraCapabilities.h"
#include "posest/CameraTriggerCache.h"
#include "posest/ToFSampleCache.h"
#include "posest/MeasurementTypes.h"
#include "posest/MeasurementBus.h"
#include "posest/config/IConfigStore.h"
#include "posest/fusion/FusionService.h"
#include "posest/pipelines/PipelineStats.h"
#include "posest/runtime/Factories.h"
#include "posest/runtime/RuntimeGraph.h"
#include "posest/runtime/WebService.h"
#include "posest/teensy/TeensyService.h"

namespace posest::runtime {

enum class DaemonState {
    Created,
    LoadedConfig,
    Built,
    Running,
    Stopping,
    Stopped,
    Failed,
};

enum class DaemonCommand {
    Run,
    CalibrateCamera,
    ImportFieldLayout,
    RecordKalibrDataset,
    MakeKalibrBag,
    CalibrateCameraImu,
    ImportCalibrationTarget,
};

struct CalibrateCameraOptions {
    // W3: repeated --camera-id / --topic / --camera-to-robot flags drive
    // a single multi-camera Kalibr invocation. The three vectors must have
    // matching non-zero size; index i in each refers to the same camera.
    std::vector<std::string> camera_ids;
    std::vector<std::string> topics;
    std::vector<Pose3d> camera_to_robots;

    std::filesystem::path bag_path;
    // Path to a Kalibr-shaped target.yaml. Either this or target_id must be
    // set. When both are present target_path wins (operator override).
    std::filesystem::path target_path;
    // Calibration target id to look up in RuntimeConfig::calibration_targets;
    // the daemon materializes the Kalibr YAML to a tempfile for the run.
    std::string target_id;
    std::filesystem::path output_dir;
    std::string version;
    std::string docker_image;
    // W2: bypass the post-Kalibr quality gate. Persists the calibration row
    // even when the reprojection RMS exceeds the threshold or could not
    // be parsed. Does NOT bypass the W3 partial-Kalibr-result guard, which
    // is a structural mismatch rather than a quality issue.
    bool force{false};
    // W2: per-run override of CalibrationToolConfig::max_reprojection_rms_px.
    std::optional<double> max_reprojection_rms_px;
};

struct ImportCalibrationTargetOptions {
    std::string target_id;
    // When --from-yaml is given, every other field is ignored: the Kalibr
    // YAML at this path is parsed into a CalibrationTargetConfig.
    std::filesystem::path from_yaml;
    // Otherwise these populate the row directly.
    std::string type;
    int rows{0};
    int cols{0};
    double tag_size_m{0.0};
    double tag_spacing_ratio{0.0};
    double square_size_m{0.0};
    std::string tag_family{"tag36h11"};
    std::string notes;
};

struct ImportFieldLayoutOptions {
    std::string field_id;
    std::string name;
    std::filesystem::path file_path;
    bool activate{false};
};

// W4: how strictly the recorder requires a working Teensy + IMU stream.
//   Auto: gate iff a Teensy serial port is configured. Skip when empty.
//   Yes:  always require Teensy time sync; throw if not established.
//   No:   intrinsic-only recording. Don't construct TeensyService at all
//         and accept frames that have no trigger event attached.
enum class ImuRequirement {
    Auto,
    Yes,
    No,
};

struct RecordKalibrDatasetOptions {
    std::vector<std::string> camera_ids;
    std::filesystem::path output_dir;
    double duration_s{0.0};
    ImuRequirement require_imu{ImuRequirement::Auto};
};

struct MakeKalibrBagOptions {
    std::filesystem::path dataset_dir;
    std::filesystem::path bag_path;
    std::string docker_image;
    // W4: when true, make_rosbag.py keeps every recorded frame (matched or
    // not) and skips emitting an IMU topic entirely. The MakeKalibrBag
    // dispatch branch sets this automatically by inspecting session.json's
    // imu_samples_recorded field; the camera-IMU path always leaves it false.
    bool no_imu{false};
};

struct CalibrateCameraImuOptions {
    std::filesystem::path dataset_dir;
    std::filesystem::path target_path;
    std::filesystem::path imu_path;
    std::string version;
    std::string docker_image;
    // W2: bypass the post-Kalibr quality gate.
    bool force{false};
    // W2: per-run override of CalibrationToolConfig::max_camera_imu_rms_px.
    std::optional<double> max_reprojection_rms_px;
};

struct DaemonOptions {
    DaemonCommand command{DaemonCommand::Run};
    std::filesystem::path config_path{"./posest.db"};
    bool health_once{false};
    bool help{false};
    std::optional<std::chrono::milliseconds> health_interval;
    CalibrateCameraOptions calibrate_camera;
    ImportFieldLayoutOptions import_field_layout;
    RecordKalibrDatasetOptions record_kalibr_dataset;
    MakeKalibrBagOptions make_kalibr_bag;
    CalibrateCameraImuOptions calibrate_camera_imu;
    ImportCalibrationTargetOptions import_calibration_target;
};

struct DaemonHealth {
    DaemonState state{DaemonState::Created};
    std::string config_path{"./posest.db"};
    std::size_t camera_count{0};
    std::size_t pipeline_count{0};
    std::uint64_t measurements_dropped{0};
    std::uint64_t measurements_processed{0};
    std::uint64_t stale_measurements{0};
    bool has_latest_pose{false};
    teensy::TeensyStats teensy;
    fusion::FusionStats fusion;
    std::vector<CameraLiveStats> cameras;
    std::vector<pipelines::AprilTagPipelineStats> apriltag_pipelines;
    std::string last_error;
    int shutdown_signal{0};
};

DaemonOptions parseDaemonOptions(int argc, const char* const argv[]);
std::string daemonUsage(const char* argv0);
const char* daemonStateName(DaemonState state);
std::string healthToJson(const DaemonHealth& health);
// W2 acceptance gate. Throws std::runtime_error when the parsed Kalibr
// metrics fall short of the threshold and force == false; a missing /
// non-positive RMS is treated as a failure (fail-safe). Exposed in the
// header so unit tests can drive it directly.
void throwIfUnacceptableCalibration(
    const CameraCalibrationConfig& calibration,
    const CalibrationToolConfig& tool,
    bool force);
void throwIfUnacceptableCameraImu(
    const CameraImuCalibrationConfig& calibration,
    const CalibrationToolConfig& tool,
    bool force);
std::string buildKalibrDockerCommand(const CalibrateCameraOptions& options);
std::string buildMakeKalibrBagDockerCommand(
    const MakeKalibrBagOptions& options,
    const std::string& docker_image);
std::string buildCalibrateCameraImuDockerCommand(
    const CalibrateCameraImuOptions& options,
    const std::filesystem::path& bag_path,
    const std::filesystem::path& camchain_path,
    const std::string& docker_image);
void runConfigCommand(
    const DaemonOptions& options,
    config::IConfigStore& config_store,
    ICameraBackendFactory& camera_factory);

class ShutdownSignal final {
public:
    void request(int signal_number);
    bool requested() const;
    int signalNumber() const;

private:
    friend void installProcessSignalHandlers(ShutdownSignal& signal);
    std::atomic<int> signal_number_{0};
};

void installProcessSignalHandlers(ShutdownSignal& signal);
void waitUntilShutdownRequested(
    const ShutdownSignal& signal,
    std::chrono::milliseconds poll_interval,
    const std::function<void()>& on_tick);

class DaemonController final {
public:
    DaemonController(
        DaemonOptions options,
        std::unique_ptr<config::IConfigStore> config_store,
        ICameraBackendFactory& camera_factory,
        IPipelineFactory& pipeline_factory);
    ~DaemonController();

    DaemonController(const DaemonController&) = delete;
    DaemonController& operator=(const DaemonController&) = delete;

    void loadAndBuild();
    void start();
    void stop(int shutdown_signal = 0);

    DaemonHealth health() const;

private:
    void refreshHealth();
    void markFailed(const std::string& error);

    DaemonOptions options_;
    std::unique_ptr<config::IConfigStore> config_store_;
    ICameraBackendFactory& camera_factory_;
    IPipelineFactory& pipeline_factory_;

    RuntimeConfig config_;
    std::unique_ptr<MeasurementBus> measurement_bus_;
    std::shared_ptr<CameraTriggerCache> trigger_cache_;
    std::shared_ptr<ToFSampleCache> tof_cache_;
    std::unique_ptr<fusion::FusionService> fusion_;
    std::shared_ptr<teensy::TeensyService> teensy_;
    std::unique_ptr<WebService> web_;
    std::unique_ptr<RuntimeGraph> graph_;
    mutable std::mutex mu_;
    DaemonHealth health_;
    bool built_{false};
    bool started_{false};
};

}  // namespace posest::runtime
