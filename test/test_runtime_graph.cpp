#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "posest/IFrameProducer.h"
#include "posest/MeasurementBus.h"
#include "posest/runtime/RuntimeGraph.h"

namespace {

class RecordingProducer final : public posest::IFrameProducer {
public:
    explicit RecordingProducer(std::string id) : id_(std::move(id)) {}

    const std::string& id() const override { return id_; }

    void addConsumer(std::shared_ptr<posest::IFrameConsumer> consumer) override {
        consumers.push_back(std::move(consumer));
    }

    bool removeConsumer(const std::shared_ptr<posest::IFrameConsumer>& consumer) override {
        auto it = std::find(consumers.begin(), consumers.end(), consumer);
        if (it == consumers.end()) {
            return false;
        }
        consumers.erase(it);
        return true;
    }

    void start() override {
        started = true;
        state_ = posest::ProducerState::Running;
    }
    void stop() override {
        stopped = true;
        state_ = posest::ProducerState::Idle;
    }

    posest::ProducerState state() const override { return state_; }

    std::string id_;
    std::vector<std::shared_ptr<posest::IFrameConsumer>> consumers;
    bool started{false};
    bool stopped{false};
    posest::ProducerState state_{posest::ProducerState::Idle};
};

class RecordingPipeline final : public posest::runtime::IVisionPipeline {
public:
    RecordingPipeline(std::string id, std::string type)
        : id_(std::move(id)), type_(std::move(type)) {}

    const std::string& id() const override { return id_; }
    const std::string& type() const override { return type_; }
    void start() override { started = true; }
    void stop() override { stopped = true; }
    void deliver(posest::FramePtr /*frame*/) override {}

    std::string id_;
    std::string type_;
    bool started{false};
    bool stopped{false};
};

class RecordingCameraFactory final : public posest::runtime::ICameraBackendFactory {
public:
    std::shared_ptr<posest::IFrameProducer> createCamera(
        const posest::CameraConfig& config) override {
        auto camera = std::make_shared<RecordingProducer>(config.id);
        cameras.push_back(camera);
        return camera;
    }

    std::vector<std::shared_ptr<RecordingProducer>> cameras;
};

class RecordingPipelineFactory final : public posest::runtime::IPipelineFactory {
public:
    std::shared_ptr<posest::runtime::IVisionPipeline> createPipeline(
        const posest::runtime::PipelineConfig& config,
        posest::IMeasurementSink& /*measurement_sink*/) override {
        auto pipeline = std::make_shared<RecordingPipeline>(config.id, config.type);
        pipelines.push_back(pipeline);
        return pipeline;
    }

    std::vector<std::shared_ptr<RecordingPipeline>> pipelines;
};

}  // namespace

TEST(RuntimeGraph, BuildsBindingsAndControlsLifecycle) {
    posest::runtime::RuntimeConfig config;
    posest::CameraConfig camera;
    camera.id = "cam0";
    camera.type = "v4l2";
    config.cameras.push_back(camera);
    posest::runtime::PipelineConfig pipeline;
    pipeline.id = "tags";
    pipeline.type = "apriltag";
    pipeline.enabled = true;
    config.pipelines.push_back(pipeline);
    config.bindings.push_back(posest::runtime::CameraPipelineBinding{
        .camera_id = "cam0",
        .pipeline_id = "tags",
    });

    RecordingCameraFactory camera_factory;
    RecordingPipelineFactory pipeline_factory;
    posest::MeasurementBus bus(8);
    posest::runtime::RuntimeGraph graph(
        config, camera_factory, pipeline_factory, bus);

    graph.build();
    EXPECT_EQ(graph.cameraCount(), 1u);
    EXPECT_EQ(graph.pipelineCount(), 1u);
    ASSERT_EQ(camera_factory.cameras.size(), 1u);
    ASSERT_EQ(camera_factory.cameras[0]->consumers.size(), 1u);

    graph.start();
    EXPECT_TRUE(camera_factory.cameras[0]->started);
    ASSERT_EQ(pipeline_factory.pipelines.size(), 1u);
    EXPECT_TRUE(pipeline_factory.pipelines[0]->started);

    graph.stop();
    EXPECT_TRUE(camera_factory.cameras[0]->stopped);
    EXPECT_TRUE(pipeline_factory.pipelines[0]->stopped);
}

TEST(RuntimeGraph, RejectsUnknownBindingTarget) {
    posest::runtime::RuntimeConfig config;
    posest::CameraConfig camera;
    camera.id = "cam0";
    camera.type = "v4l2";
    config.cameras.push_back(camera);
    config.bindings.push_back(posest::runtime::CameraPipelineBinding{
        .camera_id = "cam0",
        .pipeline_id = "missing",
    });

    RecordingCameraFactory camera_factory;
    RecordingPipelineFactory pipeline_factory;
    posest::MeasurementBus bus(8);
    posest::runtime::RuntimeGraph graph(
        config, camera_factory, pipeline_factory, bus);

    EXPECT_THROW(graph.build(), std::runtime_error);
}
