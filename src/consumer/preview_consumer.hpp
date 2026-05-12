#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include "consumer/consumer.hpp"
#include "core/frame_channel.hpp"

namespace gw {

struct PreviewConsumerStats {
    uint64_t total_received    = 0;   // frames pulled from the channel
    uint64_t total_dropped     = 0;   // skipped sequence numbers (latest-only behavior)
    uint64_t last_sequence     = 0;   // most recent frame.sequence the consumer saw
    uint64_t last_latency_ns   = 0;   // now_ns - frame.host_capture_ns for the latest frame
};

// Renders incoming Frames into a CAMetalLayer via a fullscreen textured quad.
//
// The constructor takes the layer as a type-erased void* so this header stays
// pure C++; the .mm impl bridges it back to CAMetalLayer*.
//
// PoC scope: assumes Mono8 (R8) frames. The fragment shader replicates the
// single channel across RGB for grayscale display.
class PreviewConsumer final : public IConsumer {
public:
    PreviewConsumer(std::string name, void* metal_layer);
    ~PreviewConsumer() override;

    PreviewConsumer(const PreviewConsumer&)            = delete;
    PreviewConsumer& operator=(const PreviewConsumer&) = delete;

    std::string_view name() const override;
    void             attach(FrameChannel& ch) override;
    void             detach() override;

    PreviewConsumerStats stats() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace gw
