#pragma once

#include <memory>

#include "posest/runtime/Factories.h"

namespace posest::runtime {

class ProductionCameraFactory final : public ICameraBackendFactory {
public:
    std::shared_ptr<IFrameProducer> createCamera(const CameraConfig& config) override;
};

class ProductionPipelineFactory final : public IPipelineFactory {
public:
    std::shared_ptr<IVisionPipeline> createPipeline(
        const PipelineConfig& config,
        IMeasurementSink& measurement_sink) override;
};

}  // namespace posest::runtime
