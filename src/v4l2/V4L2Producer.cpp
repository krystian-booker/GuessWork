#include "posest/V4L2Producer.h"

#include <cerrno>
#include <cstring>
#include <stdexcept>
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

std::uint32_t pixelFormatFromString(const std::string& fmt) {
    if (fmt == "mjpeg" || fmt == "MJPEG") return V4L2_PIX_FMT_MJPEG;
    if (fmt == "yuyv" || fmt == "YUYV") return V4L2_PIX_FMT_YUYV;
    throw std::runtime_error("Unsupported pixel format: " + fmt);
}

}  // namespace

namespace posest::v4l2 {

V4L2Producer::V4L2Producer(CameraConfig config)
    : CameraProducer(std::move(config)) {}

V4L2Producer::~V4L2Producer() {
    stop();
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

    // Set framerate via VIDIOC_S_PARM
    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator =
        static_cast<__u32>(config().format.fps);
    // Best-effort — some drivers ignore this
    xioctl(fd_, VIDIOC_S_PARM, &parm);
}

void V4L2Producer::applyControls() {
    for (const auto& entry : config().controls) {
        auto cid = controlNameToCid(entry.name);
        if (cid < 0) {
            fprintf(stderr, "%s: unknown control '%s', skipping\n",
                    config().device.c_str(), entry.name.c_str());
            continue;
        }

        v4l2_control ctrl{};
        ctrl.id = static_cast<__u32>(cid);
        ctrl.value = entry.value;
        if (xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
            fprintf(stderr, "%s: failed to set %s=%d: %s\n",
                    config().device.c_str(), entry.name.c_str(),
                    entry.value, std::strerror(errno));
        }
    }
}

void V4L2Producer::startStream() {
    // Request MMAP buffers
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

bool V4L2Producer::grabFrame(
    cv::Mat& out,
    std::optional<std::chrono::steady_clock::time_point>& out_capture_time) {

    while (isRunning()) {
        // Poll with timeout so we can check isRunning() periodically
        pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN;

        int ret = poll(&pfd, 1, 500);
        if (ret < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (ret == 0) continue;

        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) continue;
            return false;
        }

        // Convert V4L2 kernel timestamp to steady_clock
        if ((buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) ==
            V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
            out_capture_time = timevalToSteadyClock(
                buf.timestamp.tv_sec, buf.timestamp.tv_usec);
        }

        // Decode frame from MMAP buffer (zero-copy input)
        void* ptr = buffers_[buf.index].start;
        bool decoded = false;

        if (negotiated_pixfmt_ == V4L2_PIX_FMT_YUYV) {
            cv::Mat yuyv(negotiated_height_, negotiated_width_, CV_8UC2, ptr);
            cv::cvtColor(yuyv, out, cv::COLOR_YUV2BGR_YUYV);
            decoded = true;
        } else if (negotiated_pixfmt_ == V4L2_PIX_FMT_MJPEG) {
            cv::Mat raw(1, static_cast<int>(buf.bytesused), CV_8UC1, ptr);
            out = cv::imdecode(raw, cv::IMREAD_COLOR);
            decoded = !out.empty();
        }

        // Re-enqueue the buffer immediately after conversion
        xioctl(fd_, VIDIOC_QBUF, &buf);

        if (decoded) return true;
        // Decode failed (corrupt MJPEG or unsupported format) — retry
    }

    return false;
}

void V4L2Producer::stopStream() {
    if (fd_ < 0) return;
    auto type = static_cast<v4l2_buf_type>(V4L2_BUF_TYPE_VIDEO_CAPTURE);
    xioctl(fd_, VIDIOC_STREAMOFF, &type);
}

void V4L2Producer::closeDevice() {
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

}  // namespace posest::v4l2
