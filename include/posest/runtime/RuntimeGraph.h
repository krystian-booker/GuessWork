#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "posest/MeasurementBus.h"
#include "posest/runtime/Factories.h"
#include "posest/runtime/RuntimeConfig.h"

namespace posest::runtime {

class RuntimeGraph final {
public:
    RuntimeGraph(
        RuntimeConfig config,
        ICameraBackendFactory& camera_factory,
        IPipelineFactory& pipeline_factory,
        IMeasurementSink& measurement_sink);
    ~RuntimeGraph();

    RuntimeGraph(const RuntimeGraph&) = delete;
    RuntimeGraph& operator=(const RuntimeGraph&) = delete;

    void build();
    void start();
    void stop();

    std::size_t cameraCount() const { return cameras_.size(); }
    std::size_t pipelineCount() const { return pipelines_.size(); }

private:
    RuntimeConfig config_;
    ICameraBackendFactory& camera_factory_;
    IPipelineFactory& pipeline_factory_;
    IMeasurementSink& measurement_sink_;

    std::unordered_map<std::string, std::shared_ptr<IFrameProducer>> cameras_;
    std::unordered_map<std::string, std::shared_ptr<IVisionPipeline>> pipelines_;
    std::vector<std::shared_ptr<IVisionPipeline>> pipeline_start_order_;
    std::vector<std::shared_ptr<IFrameProducer>> camera_start_order_;
    bool built_{false};
    bool running_{false};
};

}  // namespace posest::runtime
