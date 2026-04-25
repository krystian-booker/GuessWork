#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "posest/CameraCapabilities.h"
#include "posest/CameraProducer.h"

namespace posest::v4l2 {

// RAII wrapper for a POSIX file descriptor. Move-only; closes on destruction.
class FdGuard {
public:
    FdGuard() = default;
    explicit FdGuard(int fd) noexcept : fd_(fd) {}
    ~FdGuard() noexcept { reset(); }

    FdGuard(const FdGuard&) = delete;
    FdGuard& operator=(const FdGuard&) = delete;

    FdGuard(FdGuard&& other) noexcept : fd_(other.fd_) { other.fd_ = -1; }
    FdGuard& operator=(FdGuard&& other) noexcept {
        if (this != &other) {
            reset();
            fd_ = other.fd_;
            other.fd_ = -1;
        }
        return *this;
    }

    void reset() noexcept;
    [[nodiscard]] int get() const noexcept { return fd_; }
    [[nodiscard]] int release() noexcept { int f = fd_; fd_ = -1; return f; }
    [[nodiscard]] explicit operator bool() const noexcept { return fd_ >= 0; }

private:
    int fd_{-1};
};

// RAII wrapper for an mmap region. Move-only; munmaps on destruction.
class MappedBuffer {
public:
    MappedBuffer() = default;
    MappedBuffer(void* start, std::size_t length) noexcept
        : start_(start), length_(length) {}
    ~MappedBuffer() noexcept { reset(); }

    MappedBuffer(const MappedBuffer&) = delete;
    MappedBuffer& operator=(const MappedBuffer&) = delete;

    MappedBuffer(MappedBuffer&& other) noexcept
        : start_(other.start_), length_(other.length_) {
        other.start_ = nullptr;
        other.length_ = 0;
    }
    MappedBuffer& operator=(MappedBuffer&& other) noexcept {
        if (this != &other) {
            reset();
            start_ = other.start_;
            length_ = other.length_;
            other.start_ = nullptr;
            other.length_ = 0;
        }
        return *this;
    }

    void reset() noexcept;
    [[nodiscard]] void* data() const noexcept { return start_; }
    [[nodiscard]] std::size_t size() const noexcept { return length_; }
    [[nodiscard]] bool valid() const noexcept;

private:
    void* start_{nullptr};
    std::size_t length_{0};
};

class V4L2Producer final : public CameraProducer {
public:
    explicit V4L2Producer(CameraConfig config);
    ~V4L2Producer() override;

    // Exposed for unit testing.
    [[nodiscard]] static std::int32_t controlNameToCid(std::string_view name) noexcept;
    [[nodiscard]] static std::chrono::steady_clock::time_point
    timevalToSteadyClock(long tv_sec, long tv_usec);

    // Returns the V4L2 fourcc for a canonical pixel format name; throws
    // std::runtime_error if the name is not in the dispatch table. Exposed
    // so tests can assert the catalog contents without opening a device.
    [[nodiscard]] static std::uint32_t pixelFormatFourcc(const std::string& name);
    [[nodiscard]] static bool isPixelFormatSupported(const std::string& name);

    // Live control surface.
    void setControl(const std::string& name, std::int32_t value) override;
    [[nodiscard]] std::optional<std::int32_t>
    getControl(const std::string& name) const override;

    // V4L2 / UVC backends are free-run only — explicit reject for non-FreeRun.
    void setTriggerMode(TriggerMode mode) override;

    // Capability descriptor populated from the live device when fd_ is open;
    // otherwise reports the static set of declared trigger modes plus
    // current_format/live_stats. Static parts (formats, sizes, intervals,
    // control min/max/step/default/menu) are cached and only re-enumerated
    // when applyFormat() or closeDevice() invalidate the cache.
    [[nodiscard]] CameraCapabilities capabilities() const override;

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
    // Re-enumerates pixel formats / controls / menus from the live device.
    // Caller must hold fd_mu_. Used to populate the capabilities cache.
    CameraCapabilities enumerateStaticCapabilities() const;
    void invalidateCapabilitiesCache() const noexcept;

    // Lock order across the subsystem (acquire outer → inner; never invert):
    //   1. fd_mu_       (this class) — serializes ALL ioctls on fd_ + open/close
    //   2. caps_mu_     (CameraProducer) — guards live_stats_/current_format_
    //   3. caps_cache_mu_ (this class) — guards the static-capabilities cache
    // Never take fd_mu_ while holding caps_mu_ or caps_cache_mu_. Use the
    // CameraProducer record* helpers AFTER releasing fd_mu_.
    //
    // fd_mu_ is NOT held across poll() so live setControl can run while the
    // capture thread is sleeping. It IS held during DQBUF and QBUF (and the
    // open/close sequence) so the reconnect path's fd swap is serialized
    // against in-flight setControl/getControl calls.
    mutable std::mutex fd_mu_;
    FdGuard fd_;

    static constexpr std::size_t kBufferCount = 4;
    std::vector<MappedBuffer> buffers_;

    std::uint32_t negotiated_pixfmt_{0};
    int negotiated_width_{0};
    int negotiated_height_{0};

    // Static-capabilities cache. Built lazily on first capabilities() call
    // when fd_ is open; invalidated by applyFormat() and closeDevice().
    // Holds only the fields that don't change during a stream — pixel
    // formats, control descriptors, etc.
    mutable std::mutex caps_cache_mu_;
    mutable std::optional<CameraCapabilities> caps_cache_;
};

}  // namespace posest::v4l2
