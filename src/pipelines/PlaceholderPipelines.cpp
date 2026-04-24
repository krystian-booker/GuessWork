#include "posest/pipelines/PlaceholderPipelines.h"

#include <utility>

namespace posest::pipelines {

PlaceholderAprilTagPipeline::PlaceholderAprilTagPipeline(
    std::string id,
    IMeasurementSink& sink)
    : VisionPipelineBase(std::move(id), "mock_apriltag", sink) {}

void PlaceholderAprilTagPipeline::processFrame(const Frame& frame) {
    AprilTagObservation observation;
    observation.camera_id = frame.camera_id;
    observation.frame_sequence = frame.sequence;
    observation.capture_time = frame.capture_time;
    measurementSink().publish(std::move(observation));
}

PlaceholderVioPipeline::PlaceholderVioPipeline(std::string id, IMeasurementSink& sink)
    : VisionPipelineBase(std::move(id), "vio", sink) {}

void PlaceholderVioPipeline::processFrame(const Frame& frame) {
    VioMeasurement measurement;
    measurement.camera_id = frame.camera_id;
    measurement.timestamp = frame.capture_time;
    measurement.tracking_ok = false;
    measurement.backend_status = "placeholder";
    measurementSink().publish(std::move(measurement));
}

}  // namespace posest::pipelines
