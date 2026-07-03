// GW_STUB_SPINNAKER build of SpinnakerProducer: satisfies the public header
// without the SDK so gw_producer's dependents link on machines that don't
// have the license-gated FLIR install. Nothing constructs a producer in a
// stub build (camera_supervisor_stub.cpp never starts slots); every entry
// point throws in case that ever changes.
//
// enumerate_video_modes_* are deliberately NOT defined here — their
// signatures take Spinnaker::CameraPtr by value, which requires the SDK
// headers. No stub-build TU references them.

#include "producer/spinnaker_producer.hpp"

#include <stdexcept>

namespace gw {

namespace {
[[noreturn]] void stub_fail() {
    throw std::runtime_error(
        "SpinnakerProducer: built with GW_STUB_SPINNAKER — no camera "
        "hardware support in this binary");
}
}  // namespace

struct SpinnakerProducer::Impl {
    std::string name;
    std::string serial;
};

SpinnakerProducer::SpinnakerProducer(std::string name, std::string serial,
                                     std::optional<std::string> /*mode*/,
                                     CameraSettingsValues /*initial_settings*/,
                                     HardwareSyncConfig /*hw_sync*/)
    : impl_(std::make_unique<Impl>(Impl{std::move(name), std::move(serial)})) {}

SpinnakerProducer::~SpinnakerProducer() = default;

std::string_view SpinnakerProducer::name() const { return impl_->name; }
std::string_view SpinnakerProducer::serial() const { return impl_->serial; }

FrameFormat SpinnakerProducer::format() const { stub_fail(); }
FrameChannel& SpinnakerProducer::channel() { stub_fail(); }

void SpinnakerProducer::bind_camera(std::unique_ptr<SpinnakerCameraBinding>) {
    stub_fail();
}

void SpinnakerProducer::start() { stub_fail(); }
void SpinnakerProducer::stop() {}

SpinnakerProducerStats SpinnakerProducer::stats() const { return {}; }

const VideoModeList& SpinnakerProducer::cached_video_modes() const {
    static const VideoModeList kEmpty;
    return kEmpty;
}

const CameraSettingsLimits& SpinnakerProducer::cached_settings_limits() const {
    static const CameraSettingsLimits kEmpty;
    return kEmpty;
}

CameraSettingsValues
SpinnakerProducer::apply_settings_live(const CameraSettingsPatch&) {
    stub_fail();
}

CameraSettingsValues SpinnakerProducer::current_settings() { stub_fail(); }

}  // namespace gw
