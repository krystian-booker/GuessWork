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
#include "posest/TeeSink.h"
#include "posest/config/IConfigStore.h"
#include "posest/fusion/FusionService.h"
#include "posest/pipelines/PipelineStats.h"
#include "posest/runtime/Factories.h"
#include "posest/runtime/RuntimeGraph.h"
#include "posest/runtime/WebService.h"
#include "posest/teensy/TeensyService.h"
#include "posest/vio/KimeraVioStats.h"

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
    ListKalibrDatasets,
    DeleteKalibrDataset,
    CalibrateCameraEndToEnd,
};

// W6: orchestrator mode toggle. Intrinsic runs only kalibr_calibrate_cameras;
// IntrinsicAndImu additionally chains kalibr_calibrate_imu_camera against
// the just-recorded dataset.
enum class CalibrationMode {
    Intrinsic,
    IntrinsicAndImu,
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

// W5: dataset lifecycle CLI. list emits the kalibr_datasets vector to
// stdout (text or JSON); delete removes a row by id and optionally its
// directory.
struct ListKalibrDatasetsOptions {
    bool json{false};
};

struct DeleteKalibrDatasetOptions {
    std::string id;
    bool remove_files{false};
};

// W6: chains record → bag → Kalibr (→ optional camera-IMU) → persist
// in one CLI invocation. The legacy single-step subcommands stay for power
// users; this struct mirrors their flag set so the same parser branches
// can fan into it.
struct CalibrateCameraEndToEndOptions {
    // Per-camera (W3 shape, vectors index-aligned).
    std::vector<std::string> camera_ids;
    std::vector<std::string> topics;
    std::vector<Pose3d> camera_to_robots;

    // Recording + Kalibr inputs.
    std::string target_id;
    std::filesystem::path output_dir;
    std::string version;
    double duration_s{0.0};
    ImuRequirement require_imu{ImuRequirement::Auto};

    // Mode + IMU-cam inputs. imu_path is required iff mode == IntrinsicAndImu.
    CalibrationMode mode{CalibrationMode::Intrinsic};
    std::filesystem::path imu_path;

    // W2 quality gate overrides.
    bool force{false};
    std::optional<double> max_reprojection_rms_px;

    // Resolved through resolvedKalibrDockerImage when empty.
    std::string docker_image;

    // After success, remove the dataset directory + kalibr_datasets row.
    bool cleanup_dataset{false};
};

struct ImportFieldLayoutOptions {
    std::string field_id;
    std::string name;
    std::filesystem::path file_path;
    bool activate{false};
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
    ListKalibrDatasetsOptions list_kalibr_datasets;
    DeleteKalibrDatasetOptions delete_kalibr_dataset;
    CalibrateCameraEndToEndOptions calibrate_camera_end_to_end;
};

// W6: shell-out injection point. Every Kalibr Docker invocation in the
// runtime goes through this hook so end-to-end tests can stage Kalibr
// fixture YAMLs at the path the parser expects, return 0, and skip the
// real `docker run`. The default impl wraps std::system().
//
// Tests must restore via resetSystemImplForTesting() in TearDown to avoid
// leaking a captured lambda into other test cases.
using SystemImpl = std::function<int(const char*)>;
SystemImpl setSystemImplForTesting(SystemImpl impl);
void resetSystemImplForTesting();

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
    // Per-VIO-pipeline counters fanned out from
    // IVisionPipeline::pipelineStats(). last_output_age_ms is the
    // watchdog signal: a value that grows without bound means VIO is
    // consuming frames and IMU but not emitting (autoinit failure or
    // silent stall in Kimera's pipeline). Empty until the first
    // VioMeasurement is published.
    std::vector<vio::KimeraVioStats> vio_pipelines;
    // Phase 3.1: Kimera YAML lifecycle observability. Counts the
    // successful emitKimeraParamYamls calls fired by the WebService
    // save callback (the initial loadAndBuild emit does NOT count —
    // only post-startup repaints). vio_yaml_restart_required becomes
    // true on the first successful repaint that touched a field the
    // consumer's in-place backend cycle cannot absorb (intrinsics,
    // ImuParams sigmas, vio_camera_id, etc.) and stays true thereafter.
    // mono_translation_scale_factor changes do NOT set this flag —
    // KimeraVioConsumer cycles backend->stop()/start() on the next
    // process() iteration to pick up the freshly-emitted YAML.
    // vio_yaml_repaint_last_error is empty when the last attempt
    // succeeded; populated with std::exception::what() on failure
    // (the running pipeline keeps using the previously-emitted YAMLs
    // and is unaffected by a failed repaint).
    std::uint64_t vio_yaml_repaint_count{0};
    bool vio_yaml_restart_required{false};
    std::string vio_yaml_repaint_last_error;
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

// W6: refactored helpers shared between the legacy CLI dispatch and the
// end-to-end orchestrator. `runCameraImuCalibration` runs the bag →
// kalibr_calibrate_imu_camera → parse → gate → persist sequence for a
// single CalibrateCameraImuOptions; `cleanupKalibrDataset` removes a
// kalibr_datasets row (and optionally its directory) by id.
void runCameraImuCalibration(
    const CalibrateCameraImuOptions& imu_options,
    config::IConfigStore& config_store);
void cleanupKalibrDataset(
    config::IConfigStore& config_store,
    const std::string& id,
    bool remove_files);

// W6: end-to-end orchestrator entry point. The DaemonCommand::CalibrateCameraEndToEnd
// dispatch in runConfigCommand calls straight through to this; the future HTTP
// layer can call it directly without going through CLI parsing.
void runCalibrationEndToEnd(
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

    // Phase 3.1: test/web hook returning the WebService instance owned
    // by the controller. Returns nullptr before loadAndBuild() runs.
    // Production users (HTTP server, future web frontend) reach the
    // service through this same accessor; tests use it to drive
    // stageConfig without wrapping the unique_ptr.
    WebService* webService() { return web_.get(); }

private:
    void refreshHealth();
    void markFailed(const std::string& error);
    // Phase 3.1: invoked by the WebService save callback. Emits the
    // Kimera YAML set under vio_yaml_mu_ when `cfg` differs from
    // last_emitted_yaml_config_ in any field KimeraParamWriter
    // consumes. Captures any std::exception::what() into
    // health_.vio_yaml_repaint_last_error rather than re-throwing —
    // a failed repaint must not break the save callback for the
    // other live-reload paths above it.
    void repaintKimeraYamlsIfChanged(const RuntimeConfig& cfg);

    DaemonOptions options_;
    std::unique_ptr<config::IConfigStore> config_store_;
    ICameraBackendFactory& camera_factory_;
    IPipelineFactory& pipeline_factory_;

    RuntimeConfig config_;
    std::unique_ptr<MeasurementBus> measurement_bus_;
    // Dedicated bus for IMU samples consumed by the VIO pipeline. The
    // main MeasurementBus is single-consumer (FusionService); fanning
    // IMU into both buses via TeeSink lets the VIO consumer drain its
    // own queue without contending with FusionService.
    std::unique_ptr<MeasurementBus> imu_vio_bus_;
    // Fan-out sink installed in front of TeensyService when
    // imu_vio_bus_ exists. Routes ImuSample to both buses; everything
    // else to the main bus only.
    std::unique_ptr<TeeSink> imu_tee_;
    std::shared_ptr<CameraTriggerCache> trigger_cache_;
    std::shared_ptr<ToFSampleCache> tof_cache_;
    std::unique_ptr<fusion::FusionService> fusion_;
    std::shared_ptr<teensy::TeensyService> teensy_;
    std::unique_ptr<WebService> web_;
    std::unique_ptr<RuntimeGraph> graph_;
    mutable std::mutex mu_;
    DaemonHealth health_;
    // Phase 3.1: serialize the WebService save callback's calls to
    // emitKimeraParamYamls. Per-file atomic-rename is safe, but the
    // SET of seven YAML files is not atomic — back-to-back saves could
    // interleave a partial set on disk if the callback ran on
    // overlapping threads. Also guards last_emitted_yaml_config_,
    // which is the comparison baseline for "did anything Kimera-
    // relevant change since the last emit".
    mutable std::mutex vio_yaml_mu_;
    RuntimeConfig last_emitted_yaml_config_;
    bool built_{false};
    bool started_{false};
};

}  // namespace posest::runtime
