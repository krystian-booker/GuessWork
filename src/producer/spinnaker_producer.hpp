#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "core/frame_channel.hpp"
#include "core/frame_format.hpp"
#include "producer/producer.hpp"
#include "producer/spinnaker_video_modes.hpp"

namespace gw {

// Producer driver for FLIR Spinnaker-compatible cameras (e.g. Chameleon3).
//
// The Spinnaker SDK and CoreVideo types are hidden behind a Pimpl so downstream
// translation units don't need their headers. The owning supervisor passes in
// a resolved system + camera handle via bind_camera() before start().
struct SpinnakerProducerStats {
    uint64_t total_published  = 0;
    uint64_t total_dropped    = 0;
    uint64_t total_incomplete = 0;
};

// Opaque holder for the Spinnaker SystemPtr + CameraPtr. The concrete
// definition lives in spinnaker_producer_internal.hpp (which includes the SDK)
// and is only needed by callers that build the binding (the supervisor).
struct SpinnakerCameraBinding;

class SpinnakerProducer final : public IProducer {
public:
    SpinnakerProducer(std::string                name,
                      std::string                serial,
                      std::optional<std::string> mode = std::nullopt);
    ~SpinnakerProducer() override;

    SpinnakerProducer(const SpinnakerProducer&)            = delete;
    SpinnakerProducer& operator=(const SpinnakerProducer&) = delete;

    std::string_view name()   const override;
    std::string_view serial() const;
    FrameFormat      format() const override;
    FrameChannel&    channel()      override;

    // Must be called with a valid binding before start(). Ownership transfers
    // to the producer; the binding is released by stop()/destructor.
    void bind_camera(std::unique_ptr<SpinnakerCameraBinding> binding);

    void start() override;
    void stop()  override;

    SpinnakerProducerStats stats() const;

    // Modes enumerated during start(). Returns an empty {supported=false,...}
    // if start() hasn't run or failed before the cache was populated.
    const VideoModeList& cached_video_modes() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace gw
