#pragma once

#include <atomic>
#include <string>
#include <thread>

#include "posest/Frame.h"
#include "posest/LatestFrameSlot.h"
#include "posest/MeasurementBus.h"
#include "posest/runtime/IVisionPipeline.h"

namespace posest::pipelines {

class VisionPipelineBase : public runtime::IVisionPipeline {
public:
    VisionPipelineBase(std::string id, std::string type, IMeasurementSink& sink);
    ~VisionPipelineBase() override;

    VisionPipelineBase(const VisionPipelineBase&) = delete;
    VisionPipelineBase& operator=(const VisionPipelineBase&) = delete;

    const std::string& id() const override { return id_; }
    const std::string& type() const override { return type_; }

    void start() override;
    void stop() override;
    void deliver(FramePtr frame) override;

protected:
    IMeasurementSink& measurementSink() { return sink_; }
    virtual void processFrame(const Frame& frame) = 0;

private:
    void runLoop();

    std::string id_;
    std::string type_;
    IMeasurementSink& sink_;
    LatestFrameSlot slot_;
    std::thread worker_;
    std::atomic<bool> running_{false};
};

}  // namespace posest::pipelines
