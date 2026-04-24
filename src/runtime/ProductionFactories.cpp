#include "posest/runtime/ProductionFactories.h"

#include <stdexcept>

#include "posest/pipelines/AprilTagPipeline.h"
#include "posest/pipelines/PlaceholderPipelines.h"

#if defined(POSEST_HAS_V4L2)
#include "posest/V4L2Producer.h"
#endif

namespace posest::runtime {

std::shared_ptr<IFrameProducer> ProductionCameraFactory::createCamera(
    const CameraConfig& config) {
    if (config.type == "v4l2") {
#if defined(POSEST_HAS_V4L2)
        return std::make_shared<v4l2::V4L2Producer>(config);
#else
        throw std::runtime_error("Camera backend 'v4l2' is only available on Linux builds");
#endif
    }
    throw std::runtime_error("Unsupported camera backend: " + config.type);
}

std::shared_ptr<IVisionPipeline> ProductionPipelineFactory::createPipeline(
    const PipelineConfig& config,
    IMeasurementSink& measurement_sink) {
    if (config.type == "apriltag") {
        return std::make_shared<pipelines::AprilTagPipeline>(
            config.id,
            measurement_sink,
            pipelines::parseAprilTagPipelineConfig(config));
    }
    if (config.type == "vio") {
        return std::make_shared<pipelines::PlaceholderVioPipeline>(
            config.id,
            measurement_sink);
    }
    if (config.type == "mock_apriltag") {
        return std::make_shared<pipelines::PlaceholderAprilTagPipeline>(
            config.id,
            measurement_sink);
    }
    throw std::runtime_error("Unsupported pipeline type: " + config.type);
}

}  // namespace posest::runtime
