#pragma once

#include <memory>
#include <vector>

#include "posest/CameraCapabilities.h"
#include "posest/CameraConfig.h"
#include "posest/IFrameProducer.h"
#include "posest/MeasurementBus.h"
#include "posest/runtime/IVisionPipeline.h"
#include "posest/runtime/PipelineConfig.h"
#include "posest/runtime/RuntimeConfig.h"

namespace posest::runtime {

class ICameraBackendFactory {
public:
    virtual ~ICameraBackendFactory() = default;
    virtual std::shared_ptr<IFrameProducer> createCamera(const CameraConfig& config) = 0;

    // Discover what cameras are physically available without requiring the
    // user to hand-edit config first. Default returns {} so backends that
    // don't support discovery (or test stubs) need not implement it.
    virtual std::vector<CameraCapabilities> enumerateAvailable() { return {}; }
};

class IPipelineFactory {
public:
    virtual ~IPipelineFactory() = default;
    virtual std::shared_ptr<IVisionPipeline> createPipeline(
        const PipelineConfig& config,
        IMeasurementSink& measurement_sink) = 0;
    virtual std::shared_ptr<IVisionPipeline> createPipeline(
        const PipelineConfig& config,
        IMeasurementSink& measurement_sink,
        const RuntimeConfig& runtime_config) {
        (void)runtime_config;
        return createPipeline(config, measurement_sink);
    }
};

}  // namespace posest::runtime
