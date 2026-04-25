#include "posest/V4L2Producer.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

static_assert(std::chrono::steady_clock::is_steady,
              "steady_clock must be monotonic for V4L2 timestamp conversion");

namespace {

// poll() timeout — bounds how long the capture thread waits before
// re-checking isRunning() / stop_signaled_. Not a frame-rate parameter.
constexpr int kPollTimeoutMs = 500;

int xioctl(int fd, unsigned long request, void* arg) {
    int r;
    do {
        r = ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

// ---------------- Pixel format dispatch table ----------------

using DecodeFn = void (*)(const std::uint8_t* src,
                          std::size_t bytes_used,
                          int width,
                          int height,
                          cv::Mat& dst);

void decodeMjpeg(const std::uint8_t* src, std::size_t bytes_used,
                 int /*width*/, int /*height*/, cv::Mat& dst) {
    cv::Mat raw(1, static_cast<int>(bytes_used), CV_8UC1,
                const_cast<std::uint8_t*>(src));
    dst = cv::imdecode(raw, cv::IMREAD_COLOR);
}

void decodeYuyv(const std::uint8_t* src, std::size_t /*bytes_used*/,
                int width, int height, cv::Mat& dst) {
    cv::Mat yuyv(height, width, CV_8UC2, const_cast<std::uint8_t*>(src));
    cv::cvtColor(yuyv, dst, cv::COLOR_YUV2BGR_YUYV);
}

void decodeGrey(const std::uint8_t* src, std::size_t /*bytes_used*/,
                int width, int height, cv::Mat& dst) {
    cv::Mat grey(height, width, CV_8UC1, const_cast<std::uint8_t*>(src));
    grey.copyTo(dst);   // own the bytes; kernel reclaims the mmap buffer
}

void decodeNv12(const std::uint8_t* src, std::size_t /*bytes_used*/,
                int width, int height, cv::Mat& dst) {
    // NV12: Y plane (height x width) followed by interleaved CbCr at half res.
    cv::Mat nv12(height + height / 2, width, CV_8UC1,
                 const_cast<std::uint8_t*>(src));
    cv::cvtColor(nv12, dst, cv::COLOR_YUV2BGR_NV12);
}

void decodeBgr24(const std::uint8_t* src, std::size_t /*bytes_used*/,
                 int width, int height, cv::Mat& dst) {
    cv::Mat bgr(height, width, CV_8UC3, const_cast<std::uint8_t*>(src));
    bgr.copyTo(dst);
}

void decodeBayerRggb8(const std::uint8_t* src, std::size_t /*bytes_used*/,
                      int width, int height, cv::Mat& dst) {
    cv::Mat bayer(height, width, CV_8UC1, const_cast<std::uint8_t*>(src));
    cv::cvtColor(bayer, dst, cv::COLOR_BayerRG2BGR);
}

void decodeBayerGrbg8(const std::uint8_t* src, std::size_t /*bytes_used*/,
                      int width, int height, cv::Mat& dst) {
    cv::Mat bayer(height, width, CV_8UC1, const_cast<std::uint8_t*>(src));
    cv::cvtColor(bayer, dst, cv::COLOR_BayerGR2BGR);
}

void decodeBayerGbrg8(const std::uint8_t* src, std::size_t /*bytes_used*/,
                      int width, int height, cv::Mat& dst) {
    cv::Mat bayer(height, width, CV_8UC1, const_cast<std::uint8_t*>(src));
    cv::cvtColor(bayer, dst, cv::COLOR_BayerGB2BGR);
}

void decodeBayerBggr8(const std::uint8_t* src, std::size_t /*bytes_used*/,
                      int width, int height, cv::Mat& dst) {
    cv::Mat bayer(height, width, CV_8UC1, const_cast<std::uint8_t*>(src));
    cv::cvtColor(bayer, dst, cv::COLOR_BayerBG2BGR);
}

struct PixelFormatBinding {
    std::string_view name;
    std::uint32_t fourcc;
    DecodeFn decode;
};

constexpr std::array<PixelFormatBinding, 9> kPixelFormats = {{
    {"mjpeg",       V4L2_PIX_FMT_MJPEG,  &decodeMjpeg},
    {"yuyv",        V4L2_PIX_FMT_YUYV,   &decodeYuyv},
    {"grey",        V4L2_PIX_FMT_GREY,   &decodeGrey},
    {"nv12",        V4L2_PIX_FMT_NV12,   &decodeNv12},
    {"bgr3",        V4L2_PIX_FMT_BGR24,  &decodeBgr24},
    {"bayer_rggb8", V4L2_PIX_FMT_SRGGB8, &decodeBayerRggb8},
    {"bayer_grbg8", V4L2_PIX_FMT_SGRBG8, &decodeBayerGrbg8},
    {"bayer_gbrg8", V4L2_PIX_FMT_SGBRG8, &decodeBayerGbrg8},
    {"bayer_bggr8", V4L2_PIX_FMT_SBGGR8, &decodeBayerBggr8},
}};

const PixelFormatBinding* findPixelFormatByName(std::string_view name) {
    for (const auto& entry : kPixelFormats) {
        if (entry.name == name) return &entry;
    }
    return nullptr;
}

const PixelFormatBinding* findPixelFormatByFourcc(std::uint32_t fourcc) {
    for (const auto& entry : kPixelFormats) {
        if (entry.fourcc == fourcc) return &entry;
    }
    return nullptr;
}

std::uint32_t pixelFormatFromString(const std::string& name) {
    if (const auto* binding = findPixelFormatByName(name)) {
        return binding->fourcc;
    }
    // Accept upper-case spellings of the canonical names for backwards compat.
    std::string lower;
    lower.reserve(name.size());
    for (char ch : name) {
        lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
    }
    if (const auto* binding = findPixelFormatByName(lower)) {
        return binding->fourcc;
    }
    throw std::runtime_error("Unsupported pixel format: " + name);
}

// Canonical control-name → V4L2 CID table. Sorted by name to keep the
// linear scan branch-predictor-friendly; 13 entries is faster than an
// unordered_map at this size and avoids per-call string allocation when
// the input is std::string_view.
constexpr std::array<std::pair<std::string_view, std::int32_t>, 13>
    kControlNameCids = {{
        {"backlight_compensation",     V4L2_CID_BACKLIGHT_COMPENSATION},
        {"brightness",                 V4L2_CID_BRIGHTNESS},
        {"contrast",                   V4L2_CID_CONTRAST},
        {"exposure_absolute",          V4L2_CID_EXPOSURE_ABSOLUTE},
        {"exposure_auto",              V4L2_CID_EXPOSURE_AUTO},
        {"focus_absolute",             V4L2_CID_FOCUS_ABSOLUTE},
        {"focus_auto",                 V4L2_CID_FOCUS_AUTO},
        {"gain",                       V4L2_CID_GAIN},
        {"power_line_frequency",       V4L2_CID_POWER_LINE_FREQUENCY},
        {"saturation",                 V4L2_CID_SATURATION},
        {"sharpness",                  V4L2_CID_SHARPNESS},
        {"white_balance_auto",         V4L2_CID_AUTO_WHITE_BALANCE},
        {"white_balance_temperature",  V4L2_CID_WHITE_BALANCE_TEMPERATURE},
    }};

}  // namespace

namespace posest::v4l2 {

// ---------------- FdGuard / MappedBuffer ----------------

void FdGuard::reset() noexcept {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void MappedBuffer::reset() noexcept {
    if (start_ && start_ != MAP_FAILED) {
        ::munmap(start_, length_);
    }
    start_ = nullptr;
    length_ = 0;
}

bool MappedBuffer::valid() const noexcept {
    return start_ != nullptr && start_ != MAP_FAILED;
}

// ---------------- V4L2Producer ----------------

V4L2Producer::V4L2Producer(CameraConfig config)
    : CameraProducer(std::move(config)) {}

V4L2Producer::~V4L2Producer() {
    stop();
}

std::uint32_t V4L2Producer::pixelFormatFourcc(const std::string& name) {
    return pixelFormatFromString(name);
}

bool V4L2Producer::isPixelFormatSupported(const std::string& name) {
    return findPixelFormatByName(name) != nullptr;
}

std::int32_t V4L2Producer::controlNameToCid(std::string_view name) noexcept {
    for (const auto& [n, cid] : kControlNameCids) {
        if (n == name) return cid;
    }
    return -1;
}

std::chrono::steady_clock::time_point
V4L2Producer::timevalToSteadyClock(long tv_sec, long tv_usec) {
    auto dur = std::chrono::seconds(tv_sec) + std::chrono::microseconds(tv_usec);
    return std::chrono::steady_clock::time_point(
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(dur));
}

void V4L2Producer::invalidateCapabilitiesCache() const noexcept {
    std::scoped_lock g(caps_cache_mu_);
    caps_cache_.reset();
}

void V4L2Producer::openDevice() {
    // Open + validate using a local FdGuard so a throw at any validation
    // step closes the descriptor automatically. Only publish to fd_ once
    // every check has passed.
    FdGuard guard{::open(config().device.c_str(), O_RDWR)};
    if (!guard) {
        throw std::runtime_error(
            "Failed to open " + config().device + ": " + std::strerror(errno));
    }

    v4l2_capability cap{};
    if (xioctl(guard.get(), VIDIOC_QUERYCAP, &cap) < 0) {
        throw std::runtime_error(
            config().device + ": VIDIOC_QUERYCAP failed: " + std::strerror(errno));
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        throw std::runtime_error(config().device + " does not support video capture");
    }
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        throw std::runtime_error(config().device + " does not support streaming");
    }

    {
        std::scoped_lock g(fd_mu_);
        fd_ = std::move(guard);
    }
}

void V4L2Producer::applyFormat() {
    CameraFormatConfig negotiated;
    {
        std::scoped_lock g(fd_mu_);

        v4l2_format fmt{};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = static_cast<__u32>(config().format.width);
        fmt.fmt.pix.height = static_cast<__u32>(config().format.height);
        fmt.fmt.pix.pixelformat = pixelFormatFromString(config().format.pixel_format);
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (xioctl(fd_.get(), VIDIOC_S_FMT, &fmt) < 0) {
            throw std::runtime_error(
                config().device + ": VIDIOC_S_FMT failed: " + std::strerror(errno));
        }

        negotiated_pixfmt_ = fmt.fmt.pix.pixelformat;
        negotiated_width_ = static_cast<int>(fmt.fmt.pix.width);
        negotiated_height_ = static_cast<int>(fmt.fmt.pix.height);

        // Best-effort framerate request — many UVC drivers ignore this.
        v4l2_streamparm parm{};
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator = 1;
        parm.parm.capture.timeperframe.denominator =
            static_cast<__u32>(config().format.fps);
        xioctl(fd_.get(), VIDIOC_S_PARM, &parm);

        negotiated.width = negotiated_width_;
        negotiated.height = negotiated_height_;
        negotiated.pixel_format = config().format.pixel_format;
        if (const auto* binding = findPixelFormatByFourcc(negotiated_pixfmt_)) {
            negotiated.pixel_format = std::string(binding->name);
        }
        if (parm.parm.capture.timeperframe.numerator != 0 &&
            parm.parm.capture.timeperframe.denominator != 0) {
            negotiated.fps =
                static_cast<double>(parm.parm.capture.timeperframe.denominator) /
                static_cast<double>(parm.parm.capture.timeperframe.numerator);
        } else {
            negotiated.fps = config().format.fps;
        }
    }
    // Lock-order discipline: release fd_mu_ BEFORE taking caps_mu_ via
    // setCurrentFormat. Static capability cache is also stale once the
    // negotiated format changes.
    invalidateCapabilitiesCache();
    setCurrentFormat(std::move(negotiated));
}

std::vector<ControlSetError> V4L2Producer::applyControls() {
    std::vector<ControlSetError> errors;
    std::scoped_lock g(fd_mu_);

    for (const auto& entry : config().controls) {
        const auto cid = controlNameToCid(entry.name);
        if (cid < 0) {
            errors.push_back({entry.name, entry.value, "unknown control"});
            continue;
        }

        v4l2_control ctrl{};
        ctrl.id = static_cast<__u32>(cid);
        ctrl.value = entry.value;
        if (xioctl(fd_.get(), VIDIOC_S_CTRL, &ctrl) < 0) {
            errors.push_back({entry.name, entry.value, std::strerror(errno)});
        }
    }
    return errors;
}

void V4L2Producer::startStream() {
    std::scoped_lock g(fd_mu_);

    v4l2_requestbuffers req{};
    req.count = static_cast<__u32>(kBufferCount);
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd_.get(), VIDIOC_REQBUFS, &req) < 0) {
        throw std::runtime_error(
            config().device + ": VIDIOC_REQBUFS failed: " + std::strerror(errno));
    }

    // Fresh ring on every startStream; any previous buffers are dropped
    // (their MappedBuffer destructors munmap).
    buffers_.clear();
    buffers_.reserve(req.count);

    for (__u32 i = 0; i < req.count; ++i) {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(fd_.get(), VIDIOC_QUERYBUF, &buf) < 0) {
            throw std::runtime_error(
                config().device + ": VIDIOC_QUERYBUF failed: " + std::strerror(errno));
        }

        void* mapped = ::mmap(
            nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
            fd_.get(), buf.m.offset);

        if (mapped == MAP_FAILED) {
            throw std::runtime_error(
                config().device + ": mmap failed: " + std::strerror(errno));
        }

        // Push-back BEFORE QBUF so the RAII guard owns the mmap immediately;
        // a throw from QBUF below now unwinds through ~MappedBuffer.
        buffers_.emplace_back(mapped, buf.length);

        if (xioctl(fd_.get(), VIDIOC_QBUF, &buf) < 0) {
            throw std::runtime_error(
                config().device + ": VIDIOC_QBUF failed: " + std::strerror(errno));
        }
    }

    auto type = static_cast<v4l2_buf_type>(V4L2_BUF_TYPE_VIDEO_CAPTURE);
    if (xioctl(fd_.get(), VIDIOC_STREAMON, &type) < 0) {
        throw std::runtime_error(
            config().device + ": VIDIOC_STREAMON failed: " + std::strerror(errno));
    }
}

GrabResult V4L2Producer::grabFrame(
    cv::Mat& out,
    std::optional<std::chrono::steady_clock::time_point>& out_capture_time) {

    // S7: do not carry over a stale timestamp from a prior call. Callers
    // contract is that a non-empty value means "the backend supplied it".
    out_capture_time.reset();

    while (isRunning()) {
        // Snapshot fd outside the lock so poll() does not block setControl
        // for up to kPollTimeoutMs.
        int local_fd;
        {
            std::scoped_lock g(fd_mu_);
            local_fd = fd_.get();
        }
        if (local_fd < 0) [[unlikely]] {
            return GrabResult::TransientError;
        }

        pollfd pfd{};
        pfd.fd = local_fd;
        pfd.events = POLLIN;

        int ret = ::poll(&pfd, 1, kPollTimeoutMs);
        if (!isRunning()) [[unlikely]] return GrabResult::Stopping;
        if (ret < 0) [[unlikely]] {
            if (errno == EINTR) continue;
            return GrabResult::TransientError;
        }
        if (ret == 0) continue;
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) [[unlikely]] {
            return GrabResult::TransientError;
        }
        if (!(pfd.revents & POLLIN)) [[unlikely]] continue;

        // Dequeue + snapshot decode inputs under the lock; release before
        // decode so a slow MJPEG decode (~5ms) does not gate setControl.
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        const std::uint8_t* src = nullptr;
        std::size_t bytes_used = 0;
        std::uint32_t pixfmt = 0;
        int width = 0, height = 0;
        std::optional<std::chrono::steady_clock::time_point> capture_ts;

        {
            std::scoped_lock g(fd_mu_);
            // C2: detect mid-poll fd swap (reconnect happened during poll).
            if (fd_.get() != local_fd) [[unlikely]] {
                return GrabResult::TransientError;
            }

            if (xioctl(fd_.get(), VIDIOC_DQBUF, &buf) < 0) [[unlikely]] {
                if (errno == EAGAIN) continue;
                return GrabResult::TransientError;
            }

            // Bounds-check buf.index against the buffer ring; a malformed
            // driver could otherwise corrupt memory.
            if (buf.index >= buffers_.size()) [[unlikely]] {
                xioctl(fd_.get(), VIDIOC_QBUF, &buf);
                return GrabResult::TransientError;
            }

            if ((buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) ==
                V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
                capture_ts = timevalToSteadyClock(
                    buf.timestamp.tv_sec, buf.timestamp.tv_usec);
            }

            src = static_cast<const std::uint8_t*>(buffers_[buf.index].data());
            bytes_used = buf.bytesused;
            pixfmt = negotiated_pixfmt_;
            width = negotiated_width_;
            height = negotiated_height_;
        }

        // P3: decode without fd_mu_ held. The mmap region stays valid until
        // we re-queue via QBUF below — the kernel cannot reclaim it. This
        // window is also safe against closeDevice() because closeDevice is
        // only called from the capture thread itself (via attemptReconnect)
        // or after the capture thread has joined (via stop()).
        bool decoded = false;
        if (const auto* binding = findPixelFormatByFourcc(pixfmt)) {
            try {
                binding->decode(src, bytes_used, width, height, out);
                decoded = !out.empty();
            } catch (const cv::Exception&) {
                decoded = false;
            }
        }

        // Re-acquire to re-queue the buffer. C1: check the QBUF return —
        // a silent failure here drains the 4-buffer ring and stalls the
        // capture thread at poll() forever.
        {
            std::scoped_lock g(fd_mu_);
            if (fd_.get() != local_fd) [[unlikely]] {
                return GrabResult::TransientError;
            }
            if (xioctl(fd_.get(), VIDIOC_QBUF, &buf) < 0) [[unlikely]] {
                return GrabResult::TransientError;
            }
        }

        if (decoded) {
            out_capture_time = capture_ts;
            return GrabResult::Ok;
        }
        // Decode failed (corrupt MJPEG, unknown format) — try the next frame.
    }

    return GrabResult::Stopping;
}

void V4L2Producer::stopStream() {
    std::scoped_lock g(fd_mu_);
    if (!fd_) return;
    auto type = static_cast<v4l2_buf_type>(V4L2_BUF_TYPE_VIDEO_CAPTURE);
    xioctl(fd_.get(), VIDIOC_STREAMOFF, &type);
}

void V4L2Producer::closeDevice() {
    {
        std::scoped_lock g(fd_mu_);
        buffers_.clear();   // each MappedBuffer munmaps
        fd_.reset();        // FdGuard closes
    }
    invalidateCapabilitiesCache();
}

void V4L2Producer::setControl(const std::string& name, std::int32_t value) {
    const auto cid = controlNameToCid(name);
    if (cid < 0) {
        throw std::invalid_argument("unknown control: " + name);
    }
    std::scoped_lock g(fd_mu_);
    if (!fd_) {
        throw std::runtime_error("camera not open: " + config().device);
    }
    v4l2_control ctrl{};
    ctrl.id = static_cast<__u32>(cid);
    ctrl.value = value;
    if (xioctl(fd_.get(), VIDIOC_S_CTRL, &ctrl) < 0) {
        throw std::runtime_error(
            "VIDIOC_S_CTRL " + name + ": " + std::strerror(errno));
    }
}

std::optional<std::int32_t>
V4L2Producer::getControl(const std::string& name) const {
    const auto cid = controlNameToCid(name);
    if (cid < 0) return std::nullopt;
    std::scoped_lock g(fd_mu_);
    if (!fd_) return std::nullopt;
    v4l2_control ctrl{};
    ctrl.id = static_cast<__u32>(cid);
    if (xioctl(fd_.get(), VIDIOC_G_CTRL, &ctrl) < 0) return std::nullopt;
    return ctrl.value;
}

void V4L2Producer::setTriggerMode(TriggerMode mode) {
    if (mode == TriggerMode::FreeRun) {
        return;
    }
    throw NotSupportedError(
        std::string("V4L2/UVC backend supports only FreeRun (requested: ") +
        triggerModeToString(mode) + ")");
}

namespace {

// Build the auto-companion mapping for capability descriptors so the UI knows
// which controls are gated by an auto-mode toggle.
const char* autoCompanionFor(std::string_view name) {
    if (name == "exposure_auto")              return "exposure_absolute";
    if (name == "exposure_absolute")          return "exposure_auto";
    if (name == "white_balance_auto")         return "white_balance_temperature";
    if (name == "white_balance_temperature")  return "white_balance_auto";
    if (name == "focus_auto")                 return "focus_absolute";
    if (name == "focus_absolute")             return "focus_auto";
    return "";
}

bool isAutoModeControl(std::string_view name) {
    return name == "exposure_auto" || name == "white_balance_auto" ||
           name == "focus_auto";
}

ControlValueType valueTypeFromQctrl(__u32 v4l2_type) {
    switch (v4l2_type) {
        case V4L2_CTRL_TYPE_BOOLEAN: return ControlValueType::Boolean;
        case V4L2_CTRL_TYPE_MENU:
        case V4L2_CTRL_TYPE_INTEGER_MENU: return ControlValueType::Menu;
        default: return ControlValueType::Integer;
    }
}

// Snapshot of the canonical control vocabulary (also exposed via
// controlNameToCid). Iterating once per capabilities() rebuild is cheap.
constexpr std::array<std::string_view, 13> kCanonicalControlNames = {{
    "exposure_auto", "exposure_absolute", "gain", "brightness", "contrast",
    "saturation", "sharpness", "white_balance_auto", "white_balance_temperature",
    "backlight_compensation", "focus_auto", "focus_absolute", "power_line_frequency",
}};

}  // namespace

CameraCapabilities V4L2Producer::enumerateStaticCapabilities() const {
    // Caller must hold fd_mu_ and have verified fd_ is open.
    CameraCapabilities caps;

    // Enumerate pixel formats -> sizes -> intervals.
    for (__u32 fi = 0; ; ++fi) {
        v4l2_fmtdesc fmtdesc{};
        fmtdesc.index = fi;
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(fd_.get(), VIDIOC_ENUM_FMT, &fmtdesc) < 0) break;

        const auto* binding = findPixelFormatByFourcc(fmtdesc.pixelformat);
        if (!binding) continue;   // skip formats we cannot decode

        PixelFormatOption fmt_option;
        fmt_option.name = std::string(binding->name);
        fmt_option.fourcc = fmtdesc.pixelformat;

        for (__u32 si = 0; ; ++si) {
            v4l2_frmsizeenum frmsize{};
            frmsize.index = si;
            frmsize.pixel_format = fmtdesc.pixelformat;
            if (xioctl(fd_.get(), VIDIOC_ENUM_FRAMESIZES, &frmsize) < 0) break;

            FrameSizeOption size_option;
            if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
                size_option.width = static_cast<int>(frmsize.discrete.width);
                size_option.height = static_cast<int>(frmsize.discrete.height);
            } else {
                size_option.width = static_cast<int>(frmsize.stepwise.max_width);
                size_option.height = static_cast<int>(frmsize.stepwise.max_height);
            }

            for (__u32 ii = 0; ; ++ii) {
                v4l2_frmivalenum frmival{};
                frmival.index = ii;
                frmival.pixel_format = fmtdesc.pixelformat;
                frmival.width = static_cast<__u32>(size_option.width);
                frmival.height = static_cast<__u32>(size_option.height);
                if (xioctl(fd_.get(), VIDIOC_ENUM_FRAMEINTERVALS, &frmival) < 0) break;

                FrameRateRange range;
                if (frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
                    if (frmival.discrete.numerator != 0) {
                        const double fps =
                            static_cast<double>(frmival.discrete.denominator) /
                            static_cast<double>(frmival.discrete.numerator);
                        range.discrete.push_back(fps);
                        range.min_fps = fps;
                        range.max_fps = fps;
                    }
                } else {
                    auto interval_to_fps = [](const v4l2_fract& f) {
                        return f.numerator == 0 ? 0.0
                            : static_cast<double>(f.denominator) /
                              static_cast<double>(f.numerator);
                    };
                    range.min_fps = interval_to_fps(frmival.stepwise.max);
                    range.max_fps = interval_to_fps(frmival.stepwise.min);
                    range.step_fps = interval_to_fps(frmival.stepwise.step);
                }
                size_option.rates.push_back(std::move(range));
                if (frmival.type != V4L2_FRMIVAL_TYPE_DISCRETE) break;
            }
            fmt_option.sizes.push_back(std::move(size_option));
            if (frmsize.type != V4L2_FRMSIZE_TYPE_DISCRETE) break;
        }
        caps.pixel_formats.push_back(std::move(fmt_option));
    }

    // Enumerate canonical controls (static metadata only — current_value is
    // refreshed per capabilities() call below since it can change live).
    for (const auto name : kCanonicalControlNames) {
        const auto cid = controlNameToCid(name);
        if (cid < 0) continue;

        v4l2_queryctrl qctrl{};
        qctrl.id = static_cast<__u32>(cid);
        if (xioctl(fd_.get(), VIDIOC_QUERYCTRL, &qctrl) < 0) continue;
        if (qctrl.flags & V4L2_CTRL_FLAG_DISABLED) continue;

        ControlDescriptor desc;
        desc.name = std::string(name);
        desc.type = valueTypeFromQctrl(qctrl.type);
        desc.min = qctrl.minimum;
        desc.max = qctrl.maximum;
        desc.step = qctrl.step != 0 ? static_cast<std::int32_t>(qctrl.step) : 1;
        desc.default_value = qctrl.default_value;
        desc.read_only = (qctrl.flags & V4L2_CTRL_FLAG_READ_ONLY) != 0;
        desc.auto_mode = isAutoModeControl(name);
        desc.auto_companion = autoCompanionFor(name);

        if (desc.type == ControlValueType::Menu) {
            for (__s32 mi = qctrl.minimum; mi <= qctrl.maximum; ++mi) {
                if (mi < 0) continue;
                v4l2_querymenu qmenu{};
                qmenu.id = static_cast<__u32>(cid);
                qmenu.index = static_cast<__u32>(mi);
                if (xioctl(fd_.get(), VIDIOC_QUERYMENU, &qmenu) < 0) continue;
                ControlMenuEntry entry;
                entry.value = mi;
                entry.label = reinterpret_cast<const char*>(qmenu.name);
                desc.menu_entries.push_back(std::move(entry));
            }
        }

        caps.controls.push_back(std::move(desc));
    }

    return caps;
}

CameraCapabilities V4L2Producer::capabilities() const {
    CameraCapabilities caps = CameraProducer::capabilities();
    caps.backend = "v4l2";
    caps.device_hint = {DeviceHintKind::DevicePath,
                        "Path under /dev/v4l/by-id or /dev/video*"};
    caps.trigger_modes = {TriggerMode::FreeRun};
    caps.supports_set_control = true;
    caps.supports_get_control = true;
    caps.supports_reconnect = config().reconnect.interval_ms > 0;
    caps.supports_set_trigger_mode = false;

    // Take fd_mu_ for the entire enumeration / refresh; release before
    // returning so the caller never holds the lock through their copy.
    std::scoped_lock g(fd_mu_);
    if (!fd_) {
        return caps;
    }

    // P2: build the static cache lazily and reuse it. Invalidated by
    // applyFormat() and closeDevice(); first call after either pays the
    // 50-ioctl enumeration cost, subsequent calls reuse it.
    {
        std::scoped_lock cache_g(caps_cache_mu_);
        if (!caps_cache_) {
            caps_cache_ = enumerateStaticCapabilities();
        }
        caps.pixel_formats = caps_cache_->pixel_formats;
        caps.controls = caps_cache_->controls;
    }

    // Refresh current_value per control (~13 ioctls — the dynamic portion).
    for (auto& desc : caps.controls) {
        const auto cid = controlNameToCid(desc.name);
        if (cid < 0) {
            desc.current_value.reset();
            continue;
        }
        v4l2_control current{};
        current.id = static_cast<__u32>(cid);
        if (xioctl(fd_.get(), VIDIOC_G_CTRL, &current) >= 0) {
            desc.current_value = current.value;
        } else {
            desc.current_value.reset();
        }
    }

    return caps;
}

}  // namespace posest::v4l2
