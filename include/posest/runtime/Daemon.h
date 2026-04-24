#pragma once

#include <chrono>
#include <atomic>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "posest/MeasurementTypes.h"
#include "posest/MeasurementBus.h"
#include "posest/config/IConfigStore.h"
#include "posest/fusion/FusionService.h"
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
};

struct CalibrateCameraOptions {
    std::string camera_id;
    std::filesystem::path bag_path;
    std::filesystem::path target_path;
    std::string topic;
    std::filesystem::path output_dir;
    std::string version;
    Pose3d camera_to_robot;
    bool has_camera_to_robot{false};
    std::string docker_image;
};

struct ImportFieldLayoutOptions {
    std::string field_id;
    std::string name;
    std::filesystem::path file_path;
    bool activate{false};
};

struct DaemonOptions {
    DaemonCommand command{DaemonCommand::Run};
    std::filesystem::path config_path{"./posest.db"};
    bool health_once{false};
    bool help{false};
    std::optional<std::chrono::milliseconds> health_interval;
    CalibrateCameraOptions calibrate_camera;
    ImportFieldLayoutOptions import_field_layout;
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
    std::string last_error;
    int shutdown_signal{0};
};

DaemonOptions parseDaemonOptions(int argc, const char* const argv[]);
std::string daemonUsage(const char* argv0);
const char* daemonStateName(DaemonState state);
std::string healthToJson(const DaemonHealth& health);
std::string buildKalibrDockerCommand(const CalibrateCameraOptions& options);
void runConfigCommand(const DaemonOptions& options, config::IConfigStore& config_store);

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
