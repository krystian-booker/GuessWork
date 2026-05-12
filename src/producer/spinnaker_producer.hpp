#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include "core/frame_channel.hpp"
#include "core/frame_format.hpp"
#include "producer/producer.hpp"

namespace gw {

// Producer driver for FLIR Spinnaker-compatible cameras (e.g. Chameleon3).
//
// Proof-of-concept scope:
//   - Grabs the first camera reported by Spinnaker.
//   - Forces NewestOnly stream-buffer mode and Mono8 pixel format.
//   - One CPU copy per frame from Spinnaker's host buffer into an IOSurface-
//     backed CVPixelBuffer; everything downstream is zero-copy.
//
// All Spinnaker types are hidden behind a Pimpl so downstream translation
// units don't need the SDK headers.
struct SpinnakerProducerStats {
    uint64_t total_published = 0;   // frames handed to the channel since start()
    uint64_t total_dropped   = 0;   // frames dropped because the pool was exhausted
    uint64_t total_incomplete = 0;  // images flagged incomplete by Spinnaker
};

class SpinnakerProducer final : public IProducer {
public:
    explicit SpinnakerProducer(std::string id);
    ~SpinnakerProducer() override;

    SpinnakerProducer(const SpinnakerProducer&)            = delete;
    SpinnakerProducer& operator=(const SpinnakerProducer&) = delete;

    std::string_view name()   const override;
    FrameFormat      format() const override;
    FrameChannel&    channel()      override;

    void start() override;
    void stop()  override;

    SpinnakerProducerStats stats() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace gw
