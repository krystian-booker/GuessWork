#include <chrono>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "posest/IFrameProducer.h"
#include "posest/config/SqliteConfigStore.h"
#include "posest/runtime/Daemon.h"
#include "posest/runtime/ProductionFactories.h"

using namespace std::chrono_literals;

namespace {

std::filesystem::path tempDbPath(const std::string& name) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() /
           ("posest_daemon_" + name + "_" + std::to_string(stamp) + ".db");
}

class FakeCameraFactory final : public posest::runtime::ICameraBackendFactory {
public:
    std::shared_ptr<posest::IFrameProducer> createCamera(
        const posest::CameraConfig& /*config*/) override {
        throw std::runtime_error("fake camera factory should not be used by empty config");
    }
};

class FakePipelineFactory final : public posest::runtime::IPipelineFactory {
public:
    std::shared_ptr<posest::runtime::IVisionPipeline> createPipeline(
        const posest::runtime::PipelineConfig& /*config*/,
        posest::IMeasurementSink& /*measurement_sink*/) override {
        throw std::runtime_error("fake pipeline factory should not be used by empty config");
    }
};

}  // namespace

TEST(DaemonOptions, ParsesDefaultsAndOverrides) {
    const char* defaults[] = {"posest_daemon"};
    const auto default_options = posest::runtime::parseDaemonOptions(1, defaults);
    EXPECT_EQ(default_options.config_path, std::filesystem::path("./posest.db"));
    EXPECT_FALSE(default_options.health_once);
    EXPECT_FALSE(default_options.health_interval.has_value());

    const char* args[] = {
        "posest_daemon",
        "--config",
        "/tmp/robot.db",
        "--health-once",
        "--health-interval-ms",
        "250",
    };
    const auto options = posest::runtime::parseDaemonOptions(6, args);
    EXPECT_EQ(options.config_path, std::filesystem::path("/tmp/robot.db"));
    EXPECT_TRUE(options.health_once);
    ASSERT_TRUE(options.health_interval.has_value());
    EXPECT_EQ(*options.health_interval, 250ms);
}

TEST(DaemonOptions, ParsesCalibrationAndFieldImportCommands) {
    const char* calibrate_args[] = {
        "posest_daemon",
        "calibrate-camera",
        "--config",
        "/tmp/robot.db",
        "--camera-id",
        "cam0",
        "--bag",
        "/data/calib.bag",
        "--target",
        "/data/target.yaml",
        "--topic",
        "/cam/image_raw",
        "--output-dir",
        "/tmp/kalibr",
        "--version",
        "v1",
        "--camera-to-robot",
        "0.1,0.2,0.3,0.0,0.1,0.2",
        "--docker-image",
        "kalibr:test",
    };
    const auto calibrate = posest::runtime::parseDaemonOptions(
        static_cast<int>(sizeof(calibrate_args) / sizeof(calibrate_args[0])),
        calibrate_args);
    EXPECT_EQ(calibrate.command, posest::runtime::DaemonCommand::CalibrateCamera);
    EXPECT_EQ(calibrate.calibrate_camera.camera_id, "cam0");
    EXPECT_TRUE(calibrate.calibrate_camera.has_camera_to_robot);
    EXPECT_DOUBLE_EQ(calibrate.calibrate_camera.camera_to_robot.translation_m.z, 0.3);

    const char* field_args[] = {
        "posest_daemon",
        "import-field-layout",
        "--config",
        "/tmp/robot.db",
        "--field-id",
        "reefscape",
        "--name",
        "Reefscape",
        "--file",
        "/tmp/field.json",
        "--activate",
    };
    const auto field = posest::runtime::parseDaemonOptions(
        static_cast<int>(sizeof(field_args) / sizeof(field_args[0])),
        field_args);
    EXPECT_EQ(field.command, posest::runtime::DaemonCommand::ImportFieldLayout);
    EXPECT_EQ(field.import_field_layout.field_id, "reefscape");
    EXPECT_TRUE(field.import_field_layout.activate);

    const char* record_args[] = {
        "posest_daemon",
        "record-kalibr-dataset",
        "--config",
        "/tmp/robot.db",
        "--camera-id",
        "cam0",
        "--camera-id",
        "cam1",
        "--duration-s",
        "12.5",
        "--output-dir",
        "/tmp/dataset",
    };
    const auto record = posest::runtime::parseDaemonOptions(
        static_cast<int>(sizeof(record_args) / sizeof(record_args[0])),
        record_args);
    EXPECT_EQ(record.command, posest::runtime::DaemonCommand::RecordKalibrDataset);
    ASSERT_EQ(record.record_kalibr_dataset.camera_ids.size(), 2u);
    EXPECT_EQ(record.record_kalibr_dataset.camera_ids[1], "cam1");
    EXPECT_DOUBLE_EQ(record.record_kalibr_dataset.duration_s, 12.5);

    const char* imu_args[] = {
        "posest_daemon",
        "calibrate-camera-imu",
        "--config",
        "/tmp/robot.db",
        "--dataset",
        "/tmp/dataset",
        "--target",
        "/tmp/target.yaml",
        "--imu",
        "/tmp/imu.yaml",
        "--version",
        "imu-v1",
    };
    const auto imu = posest::runtime::parseDaemonOptions(
        static_cast<int>(sizeof(imu_args) / sizeof(imu_args[0])),
        imu_args);
    EXPECT_EQ(imu.command, posest::runtime::DaemonCommand::CalibrateCameraImu);
    EXPECT_EQ(imu.calibrate_camera_imu.version, "imu-v1");
}

TEST(DaemonOptions, BuildsKalibrDockerCommandWithExpectedMounts) {
    posest::runtime::CalibrateCameraOptions options;
    options.bag_path = "/home/team/calib/cam.bag";
    options.target_path = "/home/team/calib/target.yaml";
    options.output_dir = "/home/team/out";
    options.topic = "/cam/image_raw";
    options.docker_image = "kalibr:test";

    const auto command = posest::runtime::buildKalibrDockerCommand(options);
    EXPECT_NE(command.find("docker run --rm"), std::string::npos);
    EXPECT_NE(command.find("-v '/home/team/calib':/data:ro"), std::string::npos);
    EXPECT_NE(command.find("-v '/home/team/out':/output"), std::string::npos);
    EXPECT_NE(command.find("--bag '/data/cam.bag'"), std::string::npos);
    EXPECT_NE(command.find("--target '/target/target.yaml'"), std::string::npos);
    EXPECT_NE(command.find("--topics '/cam/image_raw'"), std::string::npos);
}

TEST(DaemonOptions, BuildsKalibrBagAndCameraImuDockerCommands) {
    posest::runtime::MakeKalibrBagOptions bag;
    bag.dataset_dir = "/home/team/dataset";
    bag.bag_path = "/home/team/out/kalibr.bag";
    const auto bag_command =
        posest::runtime::buildMakeKalibrBagDockerCommand(bag, "kalibr:test");
    EXPECT_NE(bag_command.find("make_rosbag.py"), std::string::npos);
    EXPECT_NE(bag_command.find("-v '/home/team/dataset':/dataset:ro"), std::string::npos);
    EXPECT_NE(bag_command.find("--bag /out/kalibr.bag"), std::string::npos);

    posest::runtime::CalibrateCameraImuOptions imu;
    imu.dataset_dir = "/home/team/dataset";
    imu.target_path = "/home/team/calib/target.yaml";
    imu.imu_path = "/home/team/calib/imu.yaml";
    const auto imu_command = posest::runtime::buildCalibrateCameraImuDockerCommand(
        imu,
        "/home/team/dataset/kalibr.bag",
        "/home/team/dataset/input-camchain.yaml",
        "kalibr:test");
    EXPECT_NE(imu_command.find("kalibr_calibrate_imu_camera"), std::string::npos);
    EXPECT_NE(imu_command.find("--bag /dataset/kalibr.bag"), std::string::npos);
    EXPECT_NE(imu_command.find("--imu /imu/imu.yaml"), std::string::npos);
}

TEST(DaemonHealth, HealthOnceJsonContainsExpectedFieldsForEmptyDb) {
    const auto path = tempDbPath("health_once");
    std::filesystem::remove(path);

    posest::runtime::DaemonOptions options;
    options.config_path = path;
    auto store = std::make_unique<posest::config::SqliteConfigStore>(path);
    FakeCameraFactory camera_factory;
    FakePipelineFactory pipeline_factory;
    posest::runtime::DaemonController daemon(
        options, std::move(store), camera_factory, pipeline_factory);

    daemon.loadAndBuild();
    const auto json = nlohmann::json::parse(posest::runtime::healthToJson(daemon.health()));

    EXPECT_EQ(json.at("state"), "built");
    EXPECT_EQ(json.at("camera_count"), 0);
    EXPECT_EQ(json.at("pipeline_count"), 0);
    EXPECT_EQ(json.at("has_latest_pose"), false);
    ASSERT_TRUE(json.contains("teensy"));
    EXPECT_EQ(json.at("teensy").at("enabled"), false);
    // §7.4 surface: the JSON always carries an `apriltag_pipelines` array
    // (empty for an empty config) so downstream consumers can rely on the key.
    ASSERT_TRUE(json.contains("apriltag_pipelines"));
    EXPECT_TRUE(json.at("apriltag_pipelines").is_array());
    EXPECT_TRUE(json.at("apriltag_pipelines").empty());

    std::filesystem::remove(path);
}

TEST(DaemonController, LoadsBuildsStartsAndStopsEmptyConfigWithFakeFactories) {
    const auto path = tempDbPath("lifecycle");
    std::filesystem::remove(path);

    posest::runtime::DaemonOptions options;
    options.config_path = path;
    auto store = std::make_unique<posest::config::SqliteConfigStore>(path);
    FakeCameraFactory camera_factory;
    FakePipelineFactory pipeline_factory;
    posest::runtime::DaemonController daemon(
        options, std::move(store), camera_factory, pipeline_factory);

    daemon.start();
    EXPECT_EQ(daemon.health().state, posest::runtime::DaemonState::Running);
    daemon.stop(15);
    const auto health = daemon.health();
    EXPECT_EQ(health.state, posest::runtime::DaemonState::Stopped);
    EXPECT_EQ(health.shutdown_signal, 15);
    EXPECT_FALSE(health.teensy.enabled);

    std::filesystem::remove(path);
}

TEST(DaemonController, StartupFailureRecordsFailedHealthAndError) {
    const auto path = tempDbPath("failure");
    std::filesystem::remove(path);

    {
        posest::config::SqliteConfigStore seed(path);
        posest::runtime::RuntimeConfig config;
        posest::runtime::PipelineConfig pipeline;
        pipeline.id = "tags";
        pipeline.type = "apriltag";
        pipeline.enabled = true;
        config.pipelines.push_back(pipeline);
        seed.saveRuntimeConfig(config);
    }

    posest::runtime::DaemonOptions options;
    options.config_path = path;
    auto store = std::make_unique<posest::config::SqliteConfigStore>(path);
    FakeCameraFactory camera_factory;
    FakePipelineFactory pipeline_factory;
    posest::runtime::DaemonController daemon(
        options, std::move(store), camera_factory, pipeline_factory);

    EXPECT_THROW(daemon.start(), std::runtime_error);
    const auto health = daemon.health();
    EXPECT_EQ(health.state, posest::runtime::DaemonState::Failed);
    EXPECT_NE(health.last_error.find("fake pipeline factory"), std::string::npos);

    std::filesystem::remove(path);
}

TEST(DaemonSignal, WaitLoopExitsWhenSignalIsRequested) {
    posest::runtime::ShutdownSignal signal;
    int ticks = 0;

    posest::runtime::waitUntilShutdownRequested(
        signal,
        1ms,
        [&] {
            ++ticks;
            if (ticks == 3) {
                signal.request(15);
            }
        });

    EXPECT_TRUE(signal.requested());
    EXPECT_EQ(signal.signalNumber(), 15);
    EXPECT_EQ(ticks, 3);
}

TEST(ProductionFactories, CameraFactoryCreatesV4L2OnLinuxAndRejectsUnknown) {
    posest::runtime::ProductionCameraFactory factory;

    posest::CameraConfig unknown;
    unknown.id = "cam";
    unknown.type = "unknown";
    EXPECT_THROW(factory.createCamera(unknown), std::runtime_error);

#if defined(__linux__)
    posest::CameraConfig v4l2;
    v4l2.id = "cam0";
    v4l2.type = "v4l2";
    v4l2.device = "/dev/video-does-not-open-in-constructor";
    auto producer = factory.createCamera(v4l2);
    ASSERT_TRUE(producer);
    EXPECT_EQ(producer->id(), "cam0");
#endif
}

TEST(ProductionFactories, PipelineFactoryCreatesImplementedAndPlaceholderPipelines) {
    posest::runtime::ProductionPipelineFactory factory;
    posest::MeasurementBus bus(4);

    posest::runtime::PipelineConfig tags;
    tags.id = "tags";
    tags.type = "apriltag";
    EXPECT_TRUE(factory.createPipeline(tags, bus));

    posest::runtime::PipelineConfig vio;
    vio.id = "vio";
    vio.type = "vio";
    EXPECT_TRUE(factory.createPipeline(vio, bus));

    posest::runtime::PipelineConfig unknown;
    unknown.id = "unknown";
    unknown.type = "unknown";
    EXPECT_THROW(factory.createPipeline(unknown, bus), std::runtime_error);
}
