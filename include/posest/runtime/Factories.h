#pragma once

#include <memory>

#include "posest/CameraConfig.h"
#include "posest/IFrameProducer.h"
#include "posest/MeasurementBus.h"
#include "posest/runtime/IVisionPipeline.h"
#include "posest/runtime/PipelineConfig.h"

namespace posest::runtime {

class ICameraBackendFactory {
public:
    virtual ~ICameraBackendFactory() = default;
    virtual std::shared_ptr<IFrameProducer> createCamera(const CameraConfig& config) = 0;
};

class IPipelineFactory {
public:
    virtual ~IPipelineFactory() = default;
    virtual std::shared_ptr<IVisionPipeline> createPipeline(
        const PipelineConfig& config,
        IMeasurementSink& measurement_sink) = 0;
};

}  // namespace posest::runtime
