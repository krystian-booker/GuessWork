#include <chrono>
#include <memory>
#include <thread>
#include <variant>

#include <apriltag/apriltag.h>
#include <apriltag/common/image_u8.h>
#include <apriltag/tag36h11.h>
#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include "posest/MeasurementBus.h"
#include "posest/MockProducer.h"
#include "posest/pipelines/AprilTagPipeline.h"
#include "posest/pipelines/PlaceholderPipelines.h"
#include "posest/pipelines/VisionPipelineBase.h"
#include "posest/runtime/ProductionFactories.h"
#include "posest/runtime/RuntimeGraph.h"

using namespace std::chrono_literals;

namespace {

posest::FramePtr makeFrame(cv::Mat image = cv::Mat(64, 64, CV_8UC1, cv::Scalar(255))) {
    auto frame = std::make_shared<posest::Frame>();
    frame->camera_id = "cam0";
    frame->sequence = 7;
    frame->capture_time = std::chrono::steady_clock::now();
    frame->image = std::move(image);
    return frame;
}

class SlowPipeline final : public posest::pipelines::VisionPipelineBase {
public:
    explicit SlowPipeline(posest::IMeasurementSink& sink)
        : VisionPipelineBase("slow", "slow", sink) {}

protected:
    void processFrame(const posest::Frame& /*frame*/) override {
        std::this_thread::sleep_for(50ms);
    }
};

class MockCameraFactory final : public posest::runtime::ICameraBackendFactory {
public:
    std::shared_ptr<posest::IFrameProducer> createCamera(
        const posest::CameraConfig& config) override {
        auto producer = std::make_shared<posest::mock::MockProducer>(
            posest::mock::MockProducerConfig{
                .id = config.id,
                .width = 64,
                .height = 64,
                .target_fps = 30.0,
            });
        producers.push_back(producer);
        return producer;
    }

    std::vector<std::shared_ptr<posest::mock::MockProducer>> producers;
};

cv::Mat makeRenderedTag36h11(int tag_id) {
    apriltag_family_t* family = tag36h11_create();
    image_u8_t* rendered = apriltag_to_image(family, static_cast<std::uint32_t>(tag_id));
    cv::Mat tag(
        rendered->height,
        rendered->width,
        CV_8UC1,
        rendered->buf,
        static_cast<std::size_t>(rendered->stride));
    cv::Mat scaled;
    cv::resize(tag, scaled, cv::Size(200, 200), 0.0, 0.0, cv::INTER_NEAREST);

    cv::Mat canvas(320, 320, CV_8UC1, cv::Scalar(255));
    scaled.copyTo(canvas(cv::Rect(60, 60, scaled.cols, scaled.rows)));

    image_u8_destroy(rendered);
    tag36h11_destroy(family);
    return canvas;
}

}  // namespace

TEST(Pipelines, PlaceholderAprilTagPublishesEmptyObservation) {
    posest::MeasurementBus bus(4);
    posest::pipelines::PlaceholderAprilTagPipeline pipeline("mock_tags", bus);

    pipeline.start();
    pipeline.deliver(makeFrame());
    auto measurement = bus.take();
    pipeline.stop();

    ASSERT_TRUE(measurement.has_value());
    ASSERT_TRUE(std::holds_alternative<posest::AprilTagObservation>(*measurement));
    const auto& observation = std::get<posest::AprilTagObservation>(*measurement);
    EXPECT_EQ(observation.camera_id, "cam0");
    EXPECT_EQ(observation.frame_sequence, 7u);
    EXPECT_TRUE(observation.detections.empty());
}

TEST(Pipelines, PlaceholderVioPublishesTrackingNotOkMeasurement) {
    posest::MeasurementBus bus(4);
    posest::pipelines::PlaceholderVioPipeline pipeline("vio", bus);

    pipeline.start();
    pipeline.deliver(makeFrame());
    auto measurement = bus.take();
    pipeline.stop();

    ASSERT_TRUE(measurement.has_value());
    ASSERT_TRUE(std::holds_alternative<posest::VioMeasurement>(*measurement));
    const auto& vio = std::get<posest::VioMeasurement>(*measurement);
    EXPECT_EQ(vio.camera_id, "cam0");
    EXPECT_FALSE(vio.tracking_ok);
    EXPECT_EQ(vio.backend_status, "placeholder");
}

TEST(Pipelines, DeliverIsNonBlockingWhileWorkerIsSlow) {
    posest::MeasurementBus bus(4);
    SlowPipeline pipeline(bus);
    pipeline.start();

    const auto before = std::chrono::steady_clock::now();
    for (int i = 0; i < 20; ++i) {
        pipeline.deliver(makeFrame());
    }
    const auto elapsed = std::chrono::steady_clock::now() - before;
    pipeline.stop();

    EXPECT_LT(elapsed, 20ms);
}

TEST(AprilTagPipeline, ConfigParserAppliesDefaultsAndRejectsInvalidValues) {
    posest::runtime::PipelineConfig config;
    config.id = "tags";
    config.type = "apriltag";

    const auto defaults = posest::pipelines::parseAprilTagPipelineConfig(config);
    EXPECT_EQ(defaults.family, "tag36h11");
    EXPECT_EQ(defaults.nthreads, 1);
    EXPECT_DOUBLE_EQ(defaults.quad_decimate, 2.0);
    EXPECT_DOUBLE_EQ(defaults.tag_size_m, 0.1651);
    EXPECT_TRUE(defaults.refine_edges);

    config.parameters_json =
        R"({"nthreads":2,"quad_decimate":1.5,"debug":true,"tag_size_m":0.2})";
    const auto custom = posest::pipelines::parseAprilTagPipelineConfig(config);
    EXPECT_EQ(custom.nthreads, 2);
    EXPECT_DOUBLE_EQ(custom.quad_decimate, 1.5);
    EXPECT_DOUBLE_EQ(custom.tag_size_m, 0.2);
    EXPECT_TRUE(custom.debug);

    config.parameters_json = R"({"family":"tagStandard41h12"})";
    EXPECT_THROW(posest::pipelines::parseAprilTagPipelineConfig(config), std::invalid_argument);

    config.parameters_json = R"({"nthreads":"two"})";
    EXPECT_THROW(posest::pipelines::parseAprilTagPipelineConfig(config), std::exception);

    config.parameters_json = R"({"tag_size_m":0.0})";
    EXPECT_THROW(posest::pipelines::parseAprilTagPipelineConfig(config), std::invalid_argument);
}

TEST(AprilTagPipeline, BlankImagePublishesEmptyObservation) {
    posest::MeasurementBus bus(4);
    posest::pipelines::AprilTagPipelineConfig config;
    config.quad_decimate = 1.0;
    posest::pipelines::AprilTagPipeline pipeline(
        "tags",
        bus,
        config);

    pipeline.start();
    pipeline.deliver(makeFrame(cv::Mat(240, 320, CV_8UC1, cv::Scalar(255))));
    auto measurement = bus.take();
    pipeline.stop();

    ASSERT_TRUE(measurement.has_value());
    ASSERT_TRUE(std::holds_alternative<posest::AprilTagObservation>(*measurement));
    const auto& observation = std::get<posest::AprilTagObservation>(*measurement);
    EXPECT_TRUE(observation.detections.empty());
}

TEST(AprilTagPipeline, DetectsRenderedTag36h11) {
    posest::MeasurementBus bus(4);
    posest::pipelines::AprilTagPipelineConfig config;
    config.quad_decimate = 1.0;
    posest::pipelines::AprilTagPipeline pipeline(
        "tags",
        bus,
        config);

    pipeline.start();
    pipeline.deliver(makeFrame(makeRenderedTag36h11(0)));
    auto measurement = bus.take();
    pipeline.stop();

    ASSERT_TRUE(measurement.has_value());
    ASSERT_TRUE(std::holds_alternative<posest::AprilTagObservation>(*measurement));
    const auto& observation = std::get<posest::AprilTagObservation>(*measurement);
    ASSERT_FALSE(observation.detections.empty());
    EXPECT_EQ(observation.detections[0].tag_id, 0);
    for (const auto& corner : observation.detections[0].image_corners_px) {
        EXPECT_GT(corner.x, 0.0);
        EXPECT_GT(corner.y, 0.0);
    }
    EXPECT_FALSE(observation.detections[0].camera_to_tag.has_value());
}

TEST(AprilTagPipeline, PopulatesCameraToTagWhenCalibrationIsAvailable) {
    posest::MeasurementBus bus(4);
    posest::pipelines::AprilTagPipelineConfig config;
    config.quad_decimate = 1.0;
    config.camera_calibrations.emplace(
        "cam0",
        posest::pipelines::AprilTagCameraCalibration{
            .fx = 500.0,
            .fy = 500.0,
            .cx = 160.0,
            .cy = 160.0,
        });
    posest::pipelines::AprilTagPipeline pipeline(
        "tags",
        bus,
        config);

    pipeline.start();
    pipeline.deliver(makeFrame(makeRenderedTag36h11(0)));
    auto measurement = bus.take();
    pipeline.stop();

    ASSERT_TRUE(measurement.has_value());
    ASSERT_TRUE(std::holds_alternative<posest::AprilTagObservation>(*measurement));
    const auto& observation = std::get<posest::AprilTagObservation>(*measurement);
    ASSERT_FALSE(observation.detections.empty());
    ASSERT_TRUE(observation.detections[0].camera_to_tag.has_value());
    EXPECT_GT(observation.detections[0].camera_to_tag->translation_m.z, 0.0);
    EXPECT_GE(observation.detections[0].reprojection_error_px, 0.0);
}

TEST(Pipelines, ProductionPipelineFactoryCreatesConfiguredPipelines) {
    posest::MeasurementBus bus(4);
    posest::runtime::ProductionPipelineFactory factory;

    posest::runtime::PipelineConfig tags;
    tags.id = "tags";
    tags.type = "apriltag";
    auto apriltag = factory.createPipeline(tags, bus);
    ASSERT_TRUE(apriltag);
    EXPECT_EQ(apriltag->type(), "apriltag");

    posest::runtime::PipelineConfig vio;
    vio.id = "vio";
    vio.type = "vio";
    auto vio_pipeline = factory.createPipeline(vio, bus);
    ASSERT_TRUE(vio_pipeline);
    EXPECT_EQ(vio_pipeline->type(), "vio");

    posest::runtime::PipelineConfig mock_tags;
    mock_tags.id = "mock_tags";
    mock_tags.type = "mock_apriltag";
    auto mock = factory.createPipeline(mock_tags, bus);
    ASSERT_TRUE(mock);
    EXPECT_EQ(mock->type(), "mock_apriltag");

    posest::runtime::PipelineConfig unknown;
    unknown.id = "unknown";
    unknown.type = "unknown";
    EXPECT_THROW(factory.createPipeline(unknown, bus), std::runtime_error);
}

TEST(Pipelines, ProductionPipelineFactoryPassesActiveCalibrationToAprilTagPipeline) {
    posest::MeasurementBus bus(4);
    posest::runtime::ProductionPipelineFactory factory;
    posest::runtime::RuntimeConfig runtime_config;
    runtime_config.calibrations.push_back({
        .camera_id = "cam0",
        .version = "v1",
        .active = true,
        .source_file_path = "cam0.yaml",
        .created_at = "now",
        .image_width = 320,
        .image_height = 320,
        .camera_model = "pinhole",
        .distortion_model = "radtan",
        .fx = 500.0,
        .fy = 500.0,
        .cx = 160.0,
        .cy = 160.0,
    });

    posest::runtime::PipelineConfig tags;
    tags.id = "tags";
    tags.type = "apriltag";
    tags.parameters_json = R"({"quad_decimate":1.0})";
    auto apriltag = factory.createPipeline(tags, bus, runtime_config);
    ASSERT_TRUE(apriltag);

    apriltag->start();
    apriltag->deliver(makeFrame(makeRenderedTag36h11(0)));
    auto measurement = bus.take();
    apriltag->stop();

    ASSERT_TRUE(measurement.has_value());
    ASSERT_TRUE(std::holds_alternative<posest::AprilTagObservation>(*measurement));
    const auto& observation = std::get<posest::AprilTagObservation>(*measurement);
    ASSERT_FALSE(observation.detections.empty());
    EXPECT_TRUE(observation.detections[0].camera_to_tag.has_value());
}

TEST(Pipelines, RuntimeGraphFlowsCameraFramesToMeasurementBus) {
    posest::runtime::RuntimeConfig config;
    posest::CameraConfig camera;
    camera.id = "cam0";
    camera.type = "mock";
    camera.device = "mock";
    config.cameras.push_back(camera);

    posest::runtime::PipelineConfig pipeline;
    pipeline.id = "mock_tags";
    pipeline.type = "mock_apriltag";
    config.pipelines.push_back(pipeline);
    config.bindings.push_back({"cam0", "mock_tags"});

    MockCameraFactory camera_factory;
    posest::runtime::ProductionPipelineFactory pipeline_factory;
    posest::MeasurementBus bus(8);
    posest::runtime::RuntimeGraph graph(config, camera_factory, pipeline_factory, bus);

    graph.start();
    auto measurement = bus.take();
    graph.stop();

    ASSERT_TRUE(measurement.has_value());
    ASSERT_TRUE(std::holds_alternative<posest::AprilTagObservation>(*measurement));
    const auto& observation = std::get<posest::AprilTagObservation>(*measurement);
    EXPECT_EQ(observation.camera_id, "cam0");
}
