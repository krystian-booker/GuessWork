#include "posest/CameraProducer.h"

#include <algorithm>
#include <chrono>
#include <utility>

namespace posest {

namespace {

// EWMA smoothing factor for measured_fps. ~0.1 gives ~10-frame settling time
// without being so reactive that single-frame jitter dominates.
constexpr double kFpsEwmaAlpha = 0.1;

}  // namespace

CameraProducer::CameraProducer(CameraConfig config)
    : ProducerBase(config.id), config_(std::move(config)) {}

CameraProducer::~CameraProducer() {
    if (device_open_) {
        stop();
    }
}

void CameraProducer::start() {
    resetReconnectSignal();
    {
        std::scoped_lock g(caps_mu_);
        live_stats_.state = ConnectionState::Connecting;
    }

    openDevice();
    device_open_ = true;
    try {
        applyFormat();
        auto errors = applyControls();
        {
            std::scoped_lock g(caps_mu_);
            last_apply_errors_ = std::move(errors);
            if (!last_apply_errors_.empty()) {
                live_stats_.last_error =
                    "applyControls reported " +
                    std::to_string(last_apply_errors_.size()) + " failure(s)";
            }
        }
        startStream();
    } catch (...) {
        closeDevice();
        device_open_ = false;
        std::scoped_lock g(caps_mu_);
        live_stats_.state = ConnectionState::Failed;
        throw;
    }

    {
        std::scoped_lock g(caps_mu_);
        live_stats_.state = ConnectionState::Streaming;
        ++live_stats_.successful_connects;
    }

    ProducerBase::start();
}

void CameraProducer::stop() {
    signalReconnectStop();
    ProducerBase::stop();
    if (device_open_) {
        try { stopStream(); } catch (...) {}
        try { closeDevice(); } catch (...) {}
        device_open_ = false;
    }
    std::scoped_lock g(caps_mu_);
    if (live_stats_.state != ConnectionState::Failed) {
        live_stats_.state = ConnectionState::Disconnected;
    }
}

CameraCapabilities CameraProducer::capabilities() const {
    CameraCapabilities caps;
    caps.camera_id = config_.id;
    caps.backend = config_.type;
    caps.trigger_modes = {TriggerMode::FreeRun};
    caps.supports_set_trigger_mode = false;
    caps.supports_set_control = false;
    caps.supports_get_control = false;
    caps.supports_reconnect = config_.reconnect.interval_ms > 0;
    caps.current_trigger_mode = config_.trigger_mode;
    {
        std::scoped_lock g(caps_mu_);
        caps.current_format = current_format_;
        caps.live = live_stats_;
    }
    return caps;
}

void CameraProducer::setControl(const std::string& name, std::int32_t value) {
    throw NotSupportedError("setControl(" + name + "=" + std::to_string(value) +
                            ") not supported by " + config_.type + " backend");
}

std::optional<std::int32_t> CameraProducer::getControl(const std::string& name) const {
    throw NotSupportedError("getControl not supported by " + config_.type +
                            " backend (control: " + name + ")");
}

void CameraProducer::setTriggerMode(TriggerMode mode) {
    if (mode == TriggerMode::FreeRun) {
        return;
    }
    throw NotSupportedError(
        std::string("setTriggerMode(") + triggerModeToString(mode) +
        ") not supported by " + config_.type + " backend");
}

LiveStats CameraProducer::liveStats() const {
    std::scoped_lock g(caps_mu_);
    return live_stats_;
}

std::optional<CameraFormatConfig> CameraProducer::currentFormat() const {
    std::scoped_lock g(caps_mu_);
    return current_format_;
}

void CameraProducer::setCurrentFormat(CameraFormatConfig fmt) {
    std::scoped_lock g(caps_mu_);
    current_format_ = std::move(fmt);
}

void CameraProducer::recordFrameDelivered(
    std::chrono::steady_clock::time_point ts) {
    std::scoped_lock g(caps_mu_);
    if (fps_warmed_) {
        const auto delta_s =
            std::chrono::duration<double>(ts - fps_last_ts_).count();
        if (delta_s > 0.0) {
            fps_ewma_period_s_ =
                kFpsEwmaAlpha * delta_s +
                (1.0 - kFpsEwmaAlpha) * fps_ewma_period_s_;
            live_stats_.measured_fps =
                fps_ewma_period_s_ > 0.0 ? 1.0 / fps_ewma_period_s_ : 0.0;
        }
    } else {
        fps_warmed_ = true;
    }
    fps_last_ts_ = ts;
    live_stats_.last_frame_time = ts;
}

void CameraProducer::recordLastError(std::string error) {
    std::scoped_lock g(caps_mu_);
    live_stats_.last_error = std::move(error);
}

void CameraProducer::setConnectionState(ConnectionState state) {
    std::scoped_lock g(caps_mu_);
    live_stats_.state = state;
}

std::vector<ControlSetError> CameraProducer::lastApplyErrors() const {
    std::scoped_lock g(caps_mu_);
    return last_apply_errors_;
}

bool CameraProducer::captureOne(
    cv::Mat& out,
    std::optional<std::chrono::steady_clock::time_point>& out_capture_time) {
    while (isRunning()) {
        const auto result = grabFrame(out, out_capture_time);
        switch (result) {
            using enum GrabResult;
            case Ok: {
                const auto ts =
                    out_capture_time.value_or(std::chrono::steady_clock::now());
                recordFrameDelivered(ts);
                return true;
            }
            case Stopping:
                return false;
            case EndOfStream:
                return false;
            case TransientError:
                if (!attemptReconnect()) {
                    return false;
                }
                continue;
        }
    }
    return false;
}

bool CameraProducer::attemptReconnect() {
    const auto& policy = config_.reconnect;
    if (policy.interval_ms == 0) {
        // Reconnect disabled — preserve pre-feature behavior.
        return false;
    }

    setConnectionState(ConnectionState::Connecting);

    // Tear down current device best-effort.
    if (device_open_) {
        try { stopStream(); } catch (...) {}
        try { closeDevice(); } catch (...) {}
        device_open_ = false;
    }

    std::uint32_t attempt = 0;
    while (isRunning()) {
        ++attempt;
        {
            std::scoped_lock g(caps_mu_);
            ++live_stats_.reconnect_attempts;
        }

        // Interruptible sleep.
        {
            std::unique_lock<std::mutex> lk(reconnect_mu_);
            reconnect_cv_.wait_for(
                lk,
                std::chrono::milliseconds(policy.interval_ms),
                [&] { return stop_signaled_ || !isRunning(); });
            if (stop_signaled_ || !isRunning()) {
                return false;
            }
        }

        try {
            openDevice();
            device_open_ = true;
            applyFormat();
            auto errors = applyControls();
            {
                std::scoped_lock g(caps_mu_);
                last_apply_errors_ = std::move(errors);
            }
            startStream();
            std::scoped_lock g(caps_mu_);
            live_stats_.state = ConnectionState::Streaming;
            ++live_stats_.successful_connects;
            live_stats_.last_error.clear();
            return true;
        } catch (const std::exception& e) {
            try { closeDevice(); } catch (...) {}
            device_open_ = false;
            std::scoped_lock g(caps_mu_);
            live_stats_.last_error = e.what();
            ++live_stats_.disconnect_count;
        } catch (...) {
            try { closeDevice(); } catch (...) {}
            device_open_ = false;
            std::scoped_lock g(caps_mu_);
            live_stats_.last_error = "unknown reconnect failure";
            ++live_stats_.disconnect_count;
        }

        if (policy.max_attempts > 0 && attempt >= policy.max_attempts) {
            std::scoped_lock g(caps_mu_);
            live_stats_.state = ConnectionState::Failed;
            return false;
        }
    }
    return false;
}

void CameraProducer::resetReconnectSignal() {
    std::scoped_lock lk(reconnect_mu_);
    stop_signaled_ = false;
}

void CameraProducer::signalReconnectStop() {
    {
        std::scoped_lock lk(reconnect_mu_);
        stop_signaled_ = true;
    }
    reconnect_cv_.notify_all();
}

}  // namespace posest
