#include "posest/CameraProducer.h"

#include <utility>

namespace posest {

CameraProducer::CameraProducer(CameraConfig config)
    : ProducerBase(config.id), config_(std::move(config)) {}

CameraProducer::~CameraProducer() {
    // Must call stop() here so the correct virtual overrides run
    // (base destructor would dispatch to ProducerBase::stop(), missing stream teardown).
    if (device_open_) {
        stop();
    }
}

void CameraProducer::start() {
    openDevice();
    device_open_ = true;
    try {
        applyFormat();
        applyControls();
        startStream();
    } catch (...) {
        closeDevice();
        device_open_ = false;
        throw;
    }
    ProducerBase::start();
}

void CameraProducer::stop() {
    ProducerBase::stop();
    if (device_open_) {
        stopStream();
        closeDevice();
        device_open_ = false;
    }
}

bool CameraProducer::captureOne(
    cv::Mat& out,
    std::optional<std::chrono::steady_clock::time_point>& out_capture_time) {
    return grabFrame(out, out_capture_time);
}

}  // namespace posest
