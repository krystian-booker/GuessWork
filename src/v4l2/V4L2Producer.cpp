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
#include <unordered_map>
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

const PixelFormatBinding* findPixelFormatByName(const std::string& name) {
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

}  // namespace

namespace posest::v4l2 {

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

std::int32_t V4L2Producer::controlNameToCid(const std::string& name) {
    static const std::unordered_map<std::string, std::int32_t> table = {
        {"exposure_auto",              V4L2_CID_EXPOSURE_AUTO},
        {"exposure_absolute",          V4L2_CID_EXPOSURE_ABSOLUTE},
        {"gain",                       V4L2_CID_GAIN},
        {"brightness",                 V4L2_CID_BRIGHTNESS},
        {"contrast",                   V4L2_CID_CONTRAST},
        {"saturation",                 V4L2_CID_SATURATION},
        {"sharpness",                  V4L2_CID_SHARPNESS},
        {"white_balance_auto",         V4L2_CID_AUTO_WHITE_BALANCE},
        {"white_balance_temperature",  V4L2_CID_WHITE_BALANCE_TEMPERATURE},
        {"backlight_compensation",     V4L2_CID_BACKLIGHT_COMPENSATION},
        {"focus_auto",                 V4L2_CID_FOCUS_AUTO},
        {"focus_absolute",             V4L2_CID_FOCUS_ABSOLUTE},
        {"power_line_frequency",       V4L2_CID_POWER_LINE_FREQUENCY},
    };
    auto it = table.find(name);
    return it != table.end() ? it->second : -1;
}

std::chrono::steady_clock::time_point
V4L2Producer::timevalToSteadyClock(long tv_sec, long tv_usec) {
    auto dur = std::chrono::seconds(tv_sec) + std::chrono::microseconds(tv_usec);
    return std::chrono::steady_clock::time_point(
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(dur));
}

void V4L2Producer::openDevice() {
    std::lock_guard<std::mutex> g(fd_mu_);

    fd_ = open(config().device.c_str(), O_RDWR);
    if (fd_ < 0) {
        throw std::runtime_error(
            "Failed to open " + config().device + ": " + std::strerror(errno));
    }

    v4l2_capability cap{};
    if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
        close(fd_);
        fd_ = -1;
        throw std::runtime_error(
            config().device + ": VIDIOC_QUERYCAP failed: " + std::strerror(errno));
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        close(fd_);
        fd_ = -1;
        throw std::runtime_error(config().device + " does not support video capture");
    }
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        close(fd_);
        fd_ = -1;
        throw std::runtime_error(config().device + " does not support streaming");
    }
}

void V4L2Producer::applyFormat() {
    std::lock_guard<std::mutex> g(fd_mu_);

    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = static_cast<__u32>(config().format.width);
    fmt.fmt.pix.height = static_cast<__u32>(config().format.height);
    fmt.fmt.pix.pixelformat = pixelFormatFromString(config().format.pixel_format);
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
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
    xioctl(fd_, VIDIOC_S_PARM, &parm);

    // Cache negotiated format for capabilities() readback.
    CameraFormatConfig negotiated;
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
    setCurrentFormat(negotiated);
}

std::vector<ControlSetError> V4L2Producer::applyControls() {
    std::vector<ControlSetError> errors;
    std::lock_guard<std::mutex> g(fd_mu_);

    for (const auto& entry : config().controls) {
        auto cid = controlNameToCid(entry.name);
        if (cid < 0) {
            errors.push_back({entry.name, entry.value, "unknown control"});
            continue;
        }

        v4l2_control ctrl{};
        ctrl.id = static_cast<__u32>(cid);
        ctrl.value = entry.value;
        if (xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
            errors.push_back({entry.name, entry.value, std::strerror(errno)});
        }
    }
    return errors;
}

void V4L2Producer::startStream() {
    std::lock_guard<std::mutex> g(fd_mu_);

    v4l2_requestbuffers req{};
    req.count = static_cast<__u32>(kBufferCount);
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
        throw std::runtime_error(
            config().device + ": VIDIOC_REQBUFS failed: " + std::strerror(errno));
    }

    buffers_.resize(req.count);

    for (__u32 i = 0; i < req.count; ++i) {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
            throw std::runtime_error(
                config().device + ": VIDIOC_QUERYBUF failed: " + std::strerror(errno));
        }

        buffers_[i].length = buf.length;
        buffers_[i].start = mmap(
            nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
            fd_, buf.m.offset);

        if (buffers_[i].start == MAP_FAILED) {
            throw std::runtime_error(
                config().device + ": mmap failed: " + std::strerror(errno));
        }

        if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            throw std::runtime_error(
                config().device + ": VIDIOC_QBUF failed: " + std::strerror(errno));
        }
    }

    auto type = static_cast<v4l2_buf_type>(V4L2_BUF_TYPE_VIDEO_CAPTURE);
    if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        throw std::runtime_error(
            config().device + ": VIDIOC_STREAMON failed: " + std::strerror(errno));
    }
}

GrabResult V4L2Producer::grabFrame(
    cv::Mat& out,
    std::optional<std::chrono::steady_clock::time_point>& out_capture_time) {

    while (isRunning()) {
        // Snapshot fd_ outside the lock — we don't want to hold fd_mu_ across
        // poll(), which can sleep up to 500ms.
        int local_fd;
        {
            std::lock_guard<std::mutex> g(fd_mu_);
            local_fd = fd_;
        }
        if (local_fd < 0) {
            return GrabResult::TransientError;
        }

        pollfd pfd{};
        pfd.fd = local_fd;
        pfd.events = POLLIN;

        int ret = poll(&pfd, 1, 500);
        if (!isRunning()) return GrabResult::Stopping;
        if (ret < 0) {
            if (errno == EINTR) continue;
            return GrabResult::TransientError;
        }
        if (ret == 0) continue;
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            return GrabResult::TransientError;
        }
        if (!(pfd.revents & POLLIN)) {
            continue;
        }

        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        std::lock_guard<std::mutex> g(fd_mu_);
        if (fd_ < 0) {
            return GrabResult::TransientError;
        }

        if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) continue;
            return GrabResult::TransientError;
        }

        // Timestamp: convert kernel CLOCK_MONOTONIC to steady_clock if the
        // driver flagged it; otherwise leave nullopt and let ProducerBase
        // stamp on return.
        if ((buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) ==
            V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
            out_capture_time = timevalToSteadyClock(
                buf.timestamp.tv_sec, buf.timestamp.tv_usec);
        }

        // Decode frame from MMAP buffer (zero-copy input).
        const auto* src = static_cast<const std::uint8_t*>(
            buffers_[buf.index].start);
        bool decoded = false;

        if (const auto* binding = findPixelFormatByFourcc(negotiated_pixfmt_)) {
            try {
                binding->decode(src, buf.bytesused,
                                negotiated_width_, negotiated_height_, out);
                decoded = !out.empty();
            } catch (const cv::Exception&) {
                decoded = false;
            }
        }

        // Re-enqueue immediately after decode so the kernel reclaims the buffer.
        xioctl(fd_, VIDIOC_QBUF, &buf);

        if (decoded) return GrabResult::Ok;
        // Decode failed (corrupt MJPEG, unknown format) — try the next frame.
    }

    return GrabResult::Stopping;
}

void V4L2Producer::stopStream() {
    std::lock_guard<std::mutex> g(fd_mu_);
    if (fd_ < 0) return;
    auto type = static_cast<v4l2_buf_type>(V4L2_BUF_TYPE_VIDEO_CAPTURE);
    xioctl(fd_, VIDIOC_STREAMOFF, &type);
}

void V4L2Producer::closeDevice() {
    std::lock_guard<std::mutex> g(fd_mu_);
    for (auto& b : buffers_) {
        if (b.start && b.start != MAP_FAILED) {
            munmap(b.start, b.length);
        }
    }
    buffers_.clear();

    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

void V4L2Producer::setControl(const std::string& name, std::int32_t value) {
    auto cid = controlNameToCid(name);
    if (cid < 0) {
        throw std::invalid_argument("unknown control: " + name);
    }
    std::lock_guard<std::mutex> g(fd_mu_);
    if (fd_ < 0) {
        throw std::runtime_error("camera not open: " + config().device);
    }
    v4l2_control ctrl{};
    ctrl.id = static_cast<__u32>(cid);
    ctrl.value = value;
    if (xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
        throw std::runtime_error(
            "VIDIOC_S_CTRL " + name + ": " + std::strerror(errno));
    }
}

std::optional<std::int32_t>
V4L2Producer::getControl(const std::string& name) const {
    auto cid = controlNameToCid(name);
    if (cid < 0) return std::nullopt;
    std::lock_guard<std::mutex> g(fd_mu_);
    if (fd_ < 0) return std::nullopt;
    v4l2_control ctrl{};
    ctrl.id = static_cast<__u32>(cid);
    if (xioctl(fd_, VIDIOC_G_CTRL, &ctrl) < 0) return std::nullopt;
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
const char* autoCompanionFor(const std::string& name) {
    if (name == "exposure_auto")              return "exposure_absolute";
    if (name == "exposure_absolute")          return "exposure_auto";
    if (name == "white_balance_auto")         return "white_balance_temperature";
    if (name == "white_balance_temperature")  return "white_balance_auto";
    if (name == "focus_auto")                 return "focus_absolute";
    if (name == "focus_absolute")             return "focus_auto";
    return "";
}

bool isAutoModeControl(const std::string& name) {
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
// controlNameToCid). Iterating once per capabilities() call is cheap.
constexpr std::array<std::string_view, 13> kCanonicalControlNames = {{
    "exposure_auto", "exposure_absolute", "gain", "brightness", "contrast",
    "saturation", "sharpness", "white_balance_auto", "white_balance_temperature",
    "backlight_compensation", "focus_auto", "focus_absolute", "power_line_frequency",
}};

}  // namespace

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

    std::lock_guard<std::mutex> g(fd_mu_);
    if (fd_ < 0) {
        return caps;
    }

    // Enumerate pixel formats -> sizes -> intervals.
    for (__u32 fi = 0; ; ++fi) {
        v4l2_fmtdesc fmtdesc{};
        fmtdesc.index = fi;
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(fd_, VIDIOC_ENUM_FMT, &fmtdesc) < 0) break;

        const auto* binding = findPixelFormatByFourcc(fmtdesc.pixelformat);
        if (!binding) continue;   // skip formats we cannot decode

        PixelFormatOption fmt_option;
        fmt_option.name = std::string(binding->name);
        fmt_option.fourcc = fmtdesc.pixelformat;

        for (__u32 si = 0; ; ++si) {
            v4l2_frmsizeenum frmsize{};
            frmsize.index = si;
            frmsize.pixel_format = fmtdesc.pixelformat;
            if (xioctl(fd_, VIDIOC_ENUM_FRAMESIZES, &frmsize) < 0) break;

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
                if (xioctl(fd_, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) < 0) break;

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

    // Enumerate canonical controls.
    for (const auto& name_view : kCanonicalControlNames) {
        const std::string name(name_view);
        const auto cid = controlNameToCid(name);
        if (cid < 0) continue;

        v4l2_queryctrl qctrl{};
        qctrl.id = static_cast<__u32>(cid);
        if (xioctl(fd_, VIDIOC_QUERYCTRL, &qctrl) < 0) continue;
        if (qctrl.flags & V4L2_CTRL_FLAG_DISABLED) continue;

        ControlDescriptor desc;
        desc.name = name;
        desc.type = valueTypeFromQctrl(qctrl.type);
        desc.min = qctrl.minimum;
        desc.max = qctrl.maximum;
        desc.step = qctrl.step != 0 ? static_cast<std::int32_t>(qctrl.step) : 1;
        desc.default_value = qctrl.default_value;
        desc.read_only = (qctrl.flags & V4L2_CTRL_FLAG_READ_ONLY) != 0;
        desc.auto_mode = isAutoModeControl(name);
        desc.auto_companion = autoCompanionFor(name);

        v4l2_control current{};
        current.id = static_cast<__u32>(cid);
        if (xioctl(fd_, VIDIOC_G_CTRL, &current) >= 0) {
            desc.current_value = current.value;
        }

        if (desc.type == ControlValueType::Menu) {
            for (__s32 mi = qctrl.minimum; mi <= qctrl.maximum; ++mi) {
                if (mi < 0) continue;
                v4l2_querymenu qmenu{};
                qmenu.id = static_cast<__u32>(cid);
                qmenu.index = static_cast<__u32>(mi);
                if (xioctl(fd_, VIDIOC_QUERYMENU, &qmenu) < 0) continue;
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

}  // namespace posest::v4l2
