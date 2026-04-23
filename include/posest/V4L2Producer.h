#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "posest/CameraProducer.h"

namespace posest::v4l2 {

class V4L2Producer final : public CameraProducer {
public:
    explicit V4L2Producer(CameraConfig config);
    ~V4L2Producer() override;

    // Exposed for unit testing.
    static std::int32_t controlNameToCid(const std::string& name);
    static std::chrono::steady_clock::time_point
    timevalToSteadyClock(long tv_sec, long tv_usec);

protected:
    void openDevice() override;
    void applyFormat() override;
    void applyControls() override;
    void startStream() override;
    bool grabFrame(
        cv::Mat& out,
        std::optional<std::chrono::steady_clock::time_point>& out_capture_time) override;
    void stopStream() override;
    void closeDevice() override;

private:
    int fd_{-1};

    static constexpr std::size_t kBufferCount = 4;
    struct MmapBuffer {
        void* start{nullptr};
        std::size_t length{0};
    };
    std::vector<MmapBuffer> buffers_;

    std::uint32_t negotiated_pixfmt_{0};
    int negotiated_width_{0};
    int negotiated_height_{0};
};

}  // namespace posest::v4l2
