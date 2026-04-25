#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "posest/CameraConfig.h"
#include "posest/CameraProducer.h"

namespace posest::mock {

// Records the order of lifecycle hook invocations so reconnect tests can
// assert the canonical "stop -> close -> open -> format -> controls -> start"
// sequence.
enum class CameraHook : std::uint8_t {
    OpenDevice,
    ApplyFormat,
    ApplyControls,
    StartStream,
    GrabFrame,
    StopStream,
    CloseDevice,
};

const char* cameraHookName(CameraHook hook) noexcept;

// Test fixture for CameraProducer. Tests script grabFrame() return values
// (and optional applyControls() failures, openDevice failures, etc.) so the
// reconnect ladder, capability defaults, and live control surface can be
// exercised without a real device.
class MockCameraProducer : public CameraProducer {
public:
    explicit MockCameraProducer(CameraConfig config);

    // Test-side knobs. All accessors lock mu_; safe to call from any thread.

    // Push a sequence of GrabResult to be returned in order. Once the queue
    // is drained, the default action (kept_default) is returned indefinitely.
    void scriptGrabResults(std::vector<GrabResult> results);

    // Default GrabResult once the scripted queue is exhausted. Defaults to
    // GrabResult::Stopping so tests don't accidentally spin forever.
    void setDefaultGrabResult(GrabResult result);

    // Make the next N openDevice() calls throw. Useful for testing
    // attemptReconnect's exhaustion behavior.
    void scriptOpenDeviceFailures(std::uint32_t failures, std::string error);

    // Make applyControls() return these failures (instead of {}) on every call.
    void scriptApplyControlsErrors(std::vector<ControlSetError> errors);

    // Snapshot of recorded hook invocation order.
    std::vector<CameraHook> hookHistory() const;

    // Counts of individual hook invocations (cheap shorthand).
    std::uint32_t openDeviceCount() const;
    std::uint32_t closeDeviceCount() const;
    std::uint32_t applyFormatCount() const;
    std::uint32_t applyControlsCount() const;
    std::uint32_t startStreamCount() const;
    std::uint32_t stopStreamCount() const;
    std::uint32_t grabFrameCount() const;

protected:
    void openDevice() override;
    void applyFormat() override;
    std::vector<ControlSetError> applyControls() override;
    void startStream() override;
    GrabResult grabFrame(
        cv::Mat& out,
        std::optional<std::chrono::steady_clock::time_point>& out_capture_time) override;
    void stopStream() override;
    void closeDevice() override;

private:
    void recordHook(CameraHook hook);

    mutable std::mutex mu_;
    std::vector<CameraHook> history_;
    std::deque<GrabResult> scripted_results_;
    GrabResult default_result_{GrabResult::Stopping};
    std::uint32_t open_failures_remaining_{0};
    std::string open_failure_message_{"openDevice scripted failure"};
    std::vector<ControlSetError> scripted_apply_errors_;

    std::atomic<std::uint32_t> open_count_{0};
    std::atomic<std::uint32_t> close_count_{0};
    std::atomic<std::uint32_t> apply_format_count_{0};
    std::atomic<std::uint32_t> apply_controls_count_{0};
    std::atomic<std::uint32_t> start_stream_count_{0};
    std::atomic<std::uint32_t> stop_stream_count_{0};
    std::atomic<std::uint32_t> grab_frame_count_{0};
};

}  // namespace posest::mock
