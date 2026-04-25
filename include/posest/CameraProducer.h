#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "posest/CameraCapabilities.h"
#include "posest/CameraConfig.h"
#include "posest/ProducerBase.h"

namespace posest {

// Returned by grabFrame() to distinguish four cases. The base captureOne
// translates these into either deliver-the-frame, exit cleanly, or run the
// reconnect ladder.
enum class GrabResult : std::uint8_t {
    Ok = 0,             // frame written to out + (optional) timestamp; deliver it
    EndOfStream = 1,    // permanent EOS (file-backed sources, replay)
    TransientError = 2, // device hiccup; CameraProducer attempts reconnect
    Stopping = 3,       // isRunning() flipped during the call; exit without reconnect
};

// Thrown by base default implementations of optional capability methods
// (setControl, getControl, setTriggerMode for non-FreeRun) when the concrete
// backend does not support that capability.
class NotSupportedError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Reported by applyControls() when individual controls fail. Per-entry
// failures are non-fatal — the camera continues streaming, but the daemon
// can surface these via the capability descriptor / telemetry.
struct ControlSetError {
    std::string name;
    std::int32_t requested_value = 0;
    std::string error;
};

// Abstract intermediate between ProducerBase and concrete camera backends.
//
// Enforces a standard lifecycle:
//   start():  openDevice -> applyFormat -> applyControls -> startStream -> capture thread
//   stop():   capture thread joined -> stopStream -> closeDevice
//
// Subclasses implement the hooks below plus grabFrame(). captureOne() is final
// and additionally drives the reconnect ladder when grabFrame reports a
// transient I/O error.
class CameraProducer : public ProducerBase {
public:
    explicit CameraProducer(CameraConfig config);
    ~CameraProducer() override;

    CameraProducer(const CameraProducer&) = delete;
    CameraProducer& operator=(const CameraProducer&) = delete;

    const CameraConfig& config() const { return config_; }

    void start() override;
    void stop() override;

    // --- Capability descriptor -------------------------------------------
    // Default implementation reports camera_id / current_format_ / live_stats_
    // / current_trigger_mode and trigger_modes={FreeRun}. Backends override
    // to fill pixel_formats / controls / supports_* flags.
    virtual CameraCapabilities capabilities() const;

    // --- Live (runtime) control ------------------------------------------
    // Defaults throw NotSupportedError. V4L2 backend implements both with
    // VIDIOC_S_CTRL / VIDIOC_G_CTRL under fd_mu_.
    virtual void setControl(const std::string& name, std::int32_t value);
    virtual std::optional<std::int32_t> getControl(const std::string& name) const;

    // --- Trigger mode -----------------------------------------------------
    // Default accepts FreeRun (no-op), throws NotSupportedError otherwise.
    virtual void setTriggerMode(TriggerMode mode);

    // Read-only accessors for snapshots taken under caps_mu_. Useful for tests.
    LiveStats liveStats() const;
    std::optional<CameraFormatConfig> currentFormat() const;

    // Failures from the most recent applyControls() call (cleared each
    // start/reconnect cycle). Empty means every entry succeeded.
    std::vector<ControlSetError> lastApplyErrors() const;

protected:
    virtual void openDevice() = 0;
    virtual void applyFormat() = 0;

    // Returns the list of per-control failures. An empty vector means every
    // entry in config().controls was applied successfully. Failures here are
    // non-fatal: the camera continues streaming.
    virtual std::vector<ControlSetError> applyControls() = 0;

    virtual void startStream() = 0;

    // Returns one of the four GrabResult variants. On Ok, fills out (and
    // optionally out_capture_time). On the other variants, out is unused.
    virtual GrabResult grabFrame(
        cv::Mat& out,
        std::optional<std::chrono::steady_clock::time_point>& out_capture_time) = 0;

    virtual void stopStream() = 0;
    virtual void closeDevice() = 0;

    bool captureOne(
        cv::Mat& out,
        std::optional<std::chrono::steady_clock::time_point>& out_capture_time) final;

    // Update the cached negotiated format. Called by subclasses after
    // applyFormat resolves the kernel-coerced values. Surfaces in
    // capabilities().current_format.
    void setCurrentFormat(CameraFormatConfig fmt);

    // Update LiveStats fields atomically under caps_mu_. Subclasses (V4L2)
    // call recordFrameDelivered() after a successful grab; the reconnect
    // ladder updates state/disconnect_count/etc. internally.
    void recordFrameDelivered(std::chrono::steady_clock::time_point ts);
    void recordLastError(std::string error);
    void setConnectionState(ConnectionState state);

    // --- Reconnect ladder primitives -------------------------------------
    // attemptReconnect runs stopStream -> closeDevice -> sleep(interruptible)
    // -> openDevice -> applyFormat -> applyControls -> startStream until
    // success, until config().reconnect.max_attempts is reached, or until
    // stop() is called. Returns true on a successful reconnect, false on
    // exhaustion or interruption.
    bool attemptReconnect();

private:
    void resetReconnectSignal();
    void signalReconnectStop();

    CameraConfig config_;
    bool device_open_{false};

    mutable std::mutex caps_mu_;
    std::optional<CameraFormatConfig> current_format_;
    LiveStats live_stats_;
    std::vector<ControlSetError> last_apply_errors_;

    // Interruptible sleep used by attemptReconnect.
    std::mutex reconnect_mu_;
    std::condition_variable reconnect_cv_;
    bool stop_signaled_{false};

    // EWMA of inter-frame intervals for capabilities().live.measured_fps.
    bool fps_warmed_{false};
    std::chrono::steady_clock::time_point fps_last_ts_{};
    double fps_ewma_period_s_{0.0};
};

}  // namespace posest
