#include "posest/runtime/ProductionFactories.h"

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include "posest/pipelines/AprilTagPipeline.h"
#include "posest/pipelines/PlaceholderPipelines.h"
#include "posest/runtime/PipelineContextHelpers.h"
#include "posest/vio/FakeVioBackend.h"
#include "posest/vio/IVioBackend.h"
#include "posest/vio/KimeraVioConfigBuilder.h"
#include "posest/vio/KimeraVioConsumer.h"

#if defined(POSEST_HAS_V4L2)
#include "posest/V4L2DeviceEnumerator.h"
#include "posest/V4L2Producer.h"
#endif

#if defined(POSEST_BUILD_VIO)
namespace posest::vio {
// Forward-declared factory from src/vio/KimeraBackend.cpp. Compiled
// only when POSEST_BUILD_VIO is on; otherwise we fall back to
// FakeVioBackend so the daemon graph still builds on Linux dev/CI.
std::unique_ptr<IVioBackend> makeKimeraBackend(const std::string& param_dir);
}  // namespace posest::vio
#endif

namespace posest::runtime::detail {

void applyCameraCalibrationContext(
    const RuntimeConfig& runtime_config,
    pipelines::AprilTagPipelineConfig& pipeline_config) {
    std::unordered_map<std::string, std::string> active_versions;
    for (const auto& calibration : runtime_config.calibrations) {
        if (!calibration.active) {
            continue;
        }
        if (!pipeline_config.calibration_version.empty() &&
            calibration.version != pipeline_config.calibration_version) {
            continue;
        }
        active_versions[calibration.camera_id] = calibration.version;
        pipeline_config.camera_calibrations[calibration.camera_id] = {
            calibration.fx,
            calibration.fy,
            calibration.cx,
            calibration.cy,
            calibration.distortion_model,
            calibration.distortion_coefficients,
        };
    }
}

void applyFieldLayoutContext(
    const RuntimeConfig& runtime_config,
    pipelines::AprilTagPipelineConfig& pipeline_config) {
    const std::string& target_id = pipeline_config.field_layout_id.empty()
        ? runtime_config.active_field_layout_id
        : pipeline_config.field_layout_id;
    if (target_id.empty()) {
        return;
    }
    for (const auto& layout : runtime_config.field_layouts) {
        if (layout.id != target_id) {
            continue;
        }
        for (const auto& tag : layout.tags) {
            pipeline_config.field_to_tags[tag.tag_id] = tag.field_to_tag;
        }
        break;
    }
}

void applyCameraExtrinsicsContext(
    const RuntimeConfig& runtime_config,
    pipelines::AprilTagPipelineConfig& pipeline_config) {
    std::unordered_map<std::string, std::string> active_versions;
    for (const auto& calibration : runtime_config.calibrations) {
        if (!calibration.active) {
            continue;
        }
        if (!pipeline_config.calibration_version.empty() &&
            calibration.version != pipeline_config.calibration_version) {
            continue;
        }
        active_versions[calibration.camera_id] = calibration.version;
    }
    for (const auto& extrinsics : runtime_config.camera_extrinsics) {
        const auto version_it = active_versions.find(extrinsics.camera_id);
        if (version_it == active_versions.end()) {
            continue;
        }
        if (version_it->second != extrinsics.version) {
            continue;
        }
        pipeline_config.camera_to_robot[extrinsics.camera_id] = extrinsics.camera_to_robot;
    }
}

}  // namespace posest::runtime::detail

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

std::vector<CameraCapabilities> ProductionCameraFactory::enumerateAvailable() {
#if defined(POSEST_HAS_V4L2)
    return v4l2::enumerateDevices();
#else
    return {};
#endif
}

std::shared_ptr<IVisionPipeline> ProductionPipelineFactory::createPipeline(
    const PipelineConfig& config,
    IMeasurementSink& measurement_sink) {
    return createPipeline(config, measurement_sink, RuntimeConfig{});
}

std::shared_ptr<IVisionPipeline> ProductionPipelineFactory::createPipeline(
    const PipelineConfig& config,
    IMeasurementSink& measurement_sink,
    const RuntimeConfig& runtime_config) {
    if (config.type == "apriltag") {
        auto pipeline_config = pipelines::parseAprilTagPipelineConfig(config);
        detail::applyCameraCalibrationContext(runtime_config, pipeline_config);
        detail::applyCameraExtrinsicsContext(runtime_config, pipeline_config);
        detail::applyFieldLayoutContext(runtime_config, pipeline_config);
        return std::make_shared<pipelines::AprilTagPipeline>(
            config.id,
            measurement_sink,
            std::move(pipeline_config));
    }
    if (config.type == "vio") {
        if (imu_vio_bus_ == nullptr || !time_converter_) {
            throw std::runtime_error(
                "VIO pipeline requested but ProductionPipelineFactory was "
                "constructed without a VIO context — call setVioContext "
                "before building the pipeline graph.");
        }
        std::unique_ptr<vio::IVioBackend> backend;
#if defined(POSEST_BUILD_VIO)
        backend = vio::makeKimeraBackend(runtime_config.kimera_vio.param_dir);
#else
        // Linux dev/CI without Kimera installed: drop in the
        // deterministic fake so the daemon graph still builds and
        // wires end-to-end. The trajectory it emits is synthetic; do
        // not ship a build like this to a robot.
        backend = std::make_unique<vio::FakeVioBackend>();
#endif
        return std::make_shared<vio::KimeraVioConsumer>(
            config.id,
            *imu_vio_bus_,
            measurement_sink,
            time_converter_,
            std::move(backend),
            vio::buildKimeraVioConfig(runtime_config));
    }
    if (config.type == "mock_apriltag") {
        return std::make_shared<pipelines::PlaceholderAprilTagPipeline>(
            config.id,
            measurement_sink);
    }
    throw std::runtime_error("Unsupported pipeline type: " + config.type);
}

void ProductionPipelineFactory::setVioContext(
    MeasurementBus& imu_vio_bus,
    TeensyTimeConverter time_converter) {
    imu_vio_bus_ = &imu_vio_bus;
    time_converter_ = std::move(time_converter);
}

}  // namespace posest::runtime
