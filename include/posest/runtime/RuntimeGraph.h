#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "posest/CameraProducer.h"
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

    // Returns the subset of producers that are CameraProducer subclasses so
    // callers (Daemon telemetry, future web facade) can query
    // capabilities()/setControl() without sprinkling dynamic_pointer_cast.
    std::vector<std::shared_ptr<CameraProducer>> cameraProducers() const;

    // Ordered list of pipelines as they were started. Used by the daemon's
    // health snapshot to enumerate per-pipeline stats via the polymorphic
    // IVisionPipeline::pipelineStats() hook. Safe to call concurrently with
    // a running graph: pipelines_ is built once in build() and immutable
    // for the rest of the graph's lifetime; each pipeline owns its own
    // stats lock.
    std::vector<std::shared_ptr<IVisionPipeline>> pipelines() const;

    // Ids of cameras whose capture loops have exited on their own
    // (EndOfStream or Failed). Empty when every camera is Idle or Running.
    // Cheap; intended for periodic health polling.
    std::vector<std::string> deadProducers() const;

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
