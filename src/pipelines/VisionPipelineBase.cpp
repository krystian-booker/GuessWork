#include "posest/pipelines/VisionPipelineBase.h"

#include <utility>

namespace posest::pipelines {

VisionPipelineBase::VisionPipelineBase(
    std::string id,
    std::string type,
    IMeasurementSink& sink)
    : id_(std::move(id)), type_(std::move(type)), sink_(sink) {}

VisionPipelineBase::~VisionPipelineBase() {
    stop();
}

void VisionPipelineBase::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return;
    }
    worker_ = std::thread(&VisionPipelineBase::runLoop, this);
}

void VisionPipelineBase::stop() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) {
        return;
    }
    slot_.shutdown();
    if (worker_.joinable()) {
        worker_.join();
    }
}

void VisionPipelineBase::deliver(FramePtr frame) {
    if (!running_.load(std::memory_order_acquire)) {
        return;
    }
    slot_.put(std::move(frame));
}

void VisionPipelineBase::runLoop() {
    while (true) {
        FramePtr frame = slot_.take();
        if (!frame) {
            return;
        }
        processFrame(*frame);
    }
}

}  // namespace posest::pipelines
