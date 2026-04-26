#pragma once

#include <atomic>
#include <cstdint>

#include "posest/pipelines/VisionPipelineBase.h"

namespace posest::pipelines {

class PlaceholderAprilTagPipeline final : public VisionPipelineBase {
public:
    PlaceholderAprilTagPipeline(std::string id, IMeasurementSink& sink);
    // Required by the ConsumerBase contract: stop the worker thread before
    // this leaf's members go out of scope (~ConsumerBase aborts otherwise).
    ~PlaceholderAprilTagPipeline() override { stop(); }

protected:
    void processFrame(const Frame& frame) override;
};

class PlaceholderVioPipeline final : public VisionPipelineBase {
public:
    PlaceholderVioPipeline(std::string id, IMeasurementSink& sink);
    ~PlaceholderVioPipeline() override { stop(); }

    // Observability for the firmware-side ToF plumbing. Real VIO
    // implementation will replace processFrame entirely; until then these
    // counters let integration tests verify that depth is actually arriving
    // on the joined Frame.
    std::uint64_t framesWithDepth() const {
        return frames_with_depth_.load(std::memory_order_relaxed);
    }
    std::uint64_t framesWithoutDepth() const {
        return frames_without_depth_.load(std::memory_order_relaxed);
    }

protected:
    void processFrame(const Frame& frame) override;

private:
    std::atomic<std::uint64_t> frames_with_depth_{0};
    std::atomic<std::uint64_t> frames_without_depth_{0};
};

}  // namespace posest::pipelines
