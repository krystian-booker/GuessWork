#pragma once

#include "posest/pipelines/AprilTagPipeline.h"
#include "posest/runtime/RuntimeConfig.h"

namespace posest::runtime::detail {

void applyCameraCalibrationContext(
    const RuntimeConfig& runtime_config,
    pipelines::AprilTagPipelineConfig& pipeline_config);

void applyFieldLayoutContext(
    const RuntimeConfig& runtime_config,
    pipelines::AprilTagPipelineConfig& pipeline_config);

void applyCameraExtrinsicsContext(
    const RuntimeConfig& runtime_config,
    pipelines::AprilTagPipelineConfig& pipeline_config);

}  // namespace posest::runtime::detail
