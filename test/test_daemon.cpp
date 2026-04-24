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

TEST(ProductionFactories, PipelineFactoryRejectsUnimplementedPipelines) {
    posest::runtime::ProductionPipelineFactory factory;
    posest::MeasurementBus bus(4);

    posest::runtime::PipelineConfig tags;
    tags.id = "tags";
    tags.type = "apriltag";
    EXPECT_THROW(factory.createPipeline(tags, bus), std::runtime_error);

    posest::runtime::PipelineConfig vio;
    vio.id = "vio";
    vio.type = "vio";
    EXPECT_THROW(factory.createPipeline(vio, bus), std::runtime_error);
}
