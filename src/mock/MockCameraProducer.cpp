#include "posest/MockCameraProducer.h"

#include <stdexcept>
#include <utility>

namespace posest::mock {

const char* cameraHookName(CameraHook hook) noexcept {
    switch (hook) {
        case CameraHook::OpenDevice:    return "openDevice";
        case CameraHook::ApplyFormat:   return "applyFormat";
        case CameraHook::ApplyControls: return "applyControls";
        case CameraHook::StartStream:   return "startStream";
        case CameraHook::GrabFrame:     return "grabFrame";
        case CameraHook::StopStream:    return "stopStream";
        case CameraHook::CloseDevice:   return "closeDevice";
    }
    return "?";
}

MockCameraProducer::MockCameraProducer(CameraConfig config)
    : CameraProducer(std::move(config)) {}

void MockCameraProducer::scriptGrabResults(std::vector<GrabResult> results) {
    std::lock_guard<std::mutex> g(mu_);
    scripted_results_.assign(results.begin(), results.end());
}

void MockCameraProducer::setDefaultGrabResult(GrabResult result) {
    std::lock_guard<std::mutex> g(mu_);
    default_result_ = result;
}

void MockCameraProducer::scriptOpenDeviceFailures(std::uint32_t failures,
                                                  std::string error) {
    std::lock_guard<std::mutex> g(mu_);
    open_failures_remaining_ = failures;
    open_failure_message_ = std::move(error);
}

void MockCameraProducer::scriptApplyControlsErrors(
    std::vector<ControlSetError> errors) {
    std::lock_guard<std::mutex> g(mu_);
    scripted_apply_errors_ = std::move(errors);
}

std::vector<CameraHook> MockCameraProducer::hookHistory() const {
    std::lock_guard<std::mutex> g(mu_);
    return history_;
}

std::uint32_t MockCameraProducer::openDeviceCount() const   { return open_count_.load(); }
std::uint32_t MockCameraProducer::closeDeviceCount() const  { return close_count_.load(); }
std::uint32_t MockCameraProducer::applyFormatCount() const  { return apply_format_count_.load(); }
std::uint32_t MockCameraProducer::applyControlsCount() const{ return apply_controls_count_.load(); }
std::uint32_t MockCameraProducer::startStreamCount() const  { return start_stream_count_.load(); }
std::uint32_t MockCameraProducer::stopStreamCount() const   { return stop_stream_count_.load(); }
std::uint32_t MockCameraProducer::grabFrameCount() const    { return grab_frame_count_.load(); }

void MockCameraProducer::recordHook(CameraHook hook) {
    history_.push_back(hook);
}

void MockCameraProducer::openDevice() {
    open_count_.fetch_add(1);
    std::string err;
    bool fail = false;
    {
        std::lock_guard<std::mutex> g(mu_);
        recordHook(CameraHook::OpenDevice);
        if (open_failures_remaining_ > 0) {
            --open_failures_remaining_;
            err = open_failure_message_;
            fail = true;
        }
    }
    if (fail) {
        throw std::runtime_error(err);
    }
}

void MockCameraProducer::applyFormat() {
    apply_format_count_.fetch_add(1);
    {
        std::lock_guard<std::mutex> g(mu_);
        recordHook(CameraHook::ApplyFormat);
    }
    setCurrentFormat(config().format);
}

std::vector<ControlSetError> MockCameraProducer::applyControls() {
    apply_controls_count_.fetch_add(1);
    std::lock_guard<std::mutex> g(mu_);
    recordHook(CameraHook::ApplyControls);
    return scripted_apply_errors_;
}

void MockCameraProducer::startStream() {
    start_stream_count_.fetch_add(1);
    std::lock_guard<std::mutex> g(mu_);
    recordHook(CameraHook::StartStream);
}

GrabResult MockCameraProducer::grabFrame(
    cv::Mat& out,
    std::optional<std::chrono::steady_clock::time_point>& out_capture_time) {
    grab_frame_count_.fetch_add(1);
    GrabResult result;
    {
        std::lock_guard<std::mutex> g(mu_);
        recordHook(CameraHook::GrabFrame);
        if (!scripted_results_.empty()) {
            result = scripted_results_.front();
            scripted_results_.pop_front();
        } else {
            result = default_result_;
        }
    }
    if (result == GrabResult::Ok) {
        out.create(4, 4, CV_8UC1);
        out.setTo(cv::Scalar(0));
        out_capture_time = std::chrono::steady_clock::now();
    }
    return result;
}

void MockCameraProducer::stopStream() {
    stop_stream_count_.fetch_add(1);
    std::lock_guard<std::mutex> g(mu_);
    recordHook(CameraHook::StopStream);
}

void MockCameraProducer::closeDevice() {
    close_count_.fetch_add(1);
    std::lock_guard<std::mutex> g(mu_);
    recordHook(CameraHook::CloseDevice);
}

}  // namespace posest::mock
