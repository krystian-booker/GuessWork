#pragma once

#include <chrono>
#include <optional>

#include <opencv2/core/mat.hpp>

#include "posest/CameraConfig.h"
#include "posest/ProducerBase.h"

namespace posest {

// Abstract intermediate between ProducerBase and concrete camera backends.
//
// Enforces a standard lifecycle:
//   start():  openDevice → applyFormat → applyControls → startStream → capture thread
//   stop():   capture thread joined → stopStream → closeDevice
//
// Subclasses implement the hooks below plus grabFrame(). captureOne() is final.
class CameraProducer : public ProducerBase {
public:
    explicit CameraProducer(CameraConfig config);
    ~CameraProducer() override;

    CameraProducer(const CameraProducer&) = delete;
    CameraProducer& operator=(const CameraProducer&) = delete;

    const CameraConfig& config() const { return config_; }

    void start() override;
    void stop() override;

protected:
    virtual void openDevice() = 0;
    virtual void applyFormat() = 0;
    virtual void applyControls() = 0;
    virtual void startStream() = 0;

    virtual bool grabFrame(
        cv::Mat& out,
        std::optional<std::chrono::steady_clock::time_point>& out_capture_time) = 0;

    virtual void stopStream() = 0;
    virtual void closeDevice() = 0;

    bool captureOne(
        cv::Mat& out,
        std::optional<std::chrono::steady_clock::time_point>& out_capture_time) final;

private:
    CameraConfig config_;
    bool device_open_{false};
};

}  // namespace posest
