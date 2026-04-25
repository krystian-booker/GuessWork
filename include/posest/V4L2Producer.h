#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "posest/CameraCapabilities.h"
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

    // Returns the V4L2 fourcc for a canonical pixel format name; throws
    // std::runtime_error if the name is not in the dispatch table. Exposed
    // so tests can assert the catalog contents without opening a device.
    static std::uint32_t pixelFormatFourcc(const std::string& name);
    static bool isPixelFormatSupported(const std::string& name);

    // Live control surface.
    void setControl(const std::string& name, std::int32_t value) override;
    std::optional<std::int32_t> getControl(const std::string& name) const override;

    // V4L2 / UVC backends are free-run only — explicit reject for non-FreeRun.
    void setTriggerMode(TriggerMode mode) override;

    // Capability descriptor populated from the live device when fd_ is open;
    // otherwise reports the static set of declared trigger modes plus
    // current_format/live_stats. Safe to call from any thread.
    CameraCapabilities capabilities() const override;

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
    // Serializes ALL ioctls on fd_ (including DQBUF/QBUF in grabFrame). NOT
    // held across poll() so live setControl can run while the capture thread
    // is sleeping. Held during open/close so the reconnect path's fd swap is
    // serialized against in-flight setControl/getControl calls.
    mutable std::mutex fd_mu_;
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
