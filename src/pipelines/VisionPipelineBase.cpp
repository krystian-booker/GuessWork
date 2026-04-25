#include "posest/pipelines/VisionPipelineBase.h"

#include <utility>

namespace posest::pipelines {

VisionPipelineBase::VisionPipelineBase(
    std::string id,
    std::string type,
    IMeasurementSink& sink)
    : ConsumerBase(std::move(id)), type_(std::move(type)), sink_(sink) {}

}  // namespace posest::pipelines
