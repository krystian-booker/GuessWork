// camera_config_tool [--generate <output.xml>]
//
// Enumerates V4L2 cameras, shows capabilities (formats, resolutions,
// framerates, controls), and optionally generates an XML config file.

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/videodev2.h>

namespace {

int xioctl(int fd, unsigned long request, void* arg) {
    int r;
    do {
        r = ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

struct FrameSize {
    int width;
    int height;
};

struct FrameRate {
    int numerator;
    int denominator;
};

struct FormatInfo {
    std::string fourcc;
    std::string description;
    __u32 pixelformat;
    std::vector<std::pair<FrameSize, std::vector<FrameRate>>> sizes;
};

struct ControlInfo {
    std::string name;
    __u32 id;
    __s32 minimum;
    __s32 maximum;
    __s32 step;
    __s32 default_value;
    __u32 type;
};

struct CameraInfo {
    std::string device_path;
    std::string by_id_path;
    std::string by_path_path;
    std::string card;
    std::string driver;
    std::string bus_info;
    std::vector<FormatInfo> formats;
    std::vector<ControlInfo> controls;
};

std::string fourccToString(__u32 pf) {
    char buf[5];
    buf[0] = static_cast<char>(pf & 0xFF);
    buf[1] = static_cast<char>((pf >> 8) & 0xFF);
    buf[2] = static_cast<char>((pf >> 16) & 0xFF);
    buf[3] = static_cast<char>((pf >> 24) & 0xFF);
    buf[4] = '\0';
    return buf;
}

std::string controlTypeStr(__u32 type) {
    switch (type) {
        case V4L2_CTRL_TYPE_INTEGER: return "int";
        case V4L2_CTRL_TYPE_BOOLEAN: return "bool";
        case V4L2_CTRL_TYPE_MENU: return "menu";
        case V4L2_CTRL_TYPE_INTEGER_MENU: return "int_menu";
        case V4L2_CTRL_TYPE_BUTTON: return "button";
        default: return "other";
    }
}

// Map V4L2 CID to the config name used in cameras.xml
const char* cidToConfigName(__u32 cid) {
    switch (cid) {
        case V4L2_CID_EXPOSURE_AUTO: return "exposure_auto";
        case V4L2_CID_EXPOSURE_ABSOLUTE: return "exposure_absolute";
        case V4L2_CID_GAIN: return "gain";
        case V4L2_CID_BRIGHTNESS: return "brightness";
        case V4L2_CID_CONTRAST: return "contrast";
        case V4L2_CID_SATURATION: return "saturation";
        case V4L2_CID_SHARPNESS: return "sharpness";
        case V4L2_CID_AUTO_WHITE_BALANCE: return "white_balance_auto";
        case V4L2_CID_WHITE_BALANCE_TEMPERATURE: return "white_balance_temperature";
        case V4L2_CID_BACKLIGHT_COMPENSATION: return "backlight_compensation";
        case V4L2_CID_FOCUS_AUTO: return "focus_auto";
        case V4L2_CID_FOCUS_ABSOLUTE: return "focus_absolute";
        case V4L2_CID_POWER_LINE_FREQUENCY: return "power_line_frequency";
        default: return nullptr;
    }
}

std::vector<FrameRate> enumFrameIntervals(int fd, __u32 pixfmt, int w, int h) {
    std::vector<FrameRate> rates;
    v4l2_frmivalenum fival{};
    fival.pixel_format = pixfmt;
    fival.width = static_cast<__u32>(w);
    fival.height = static_cast<__u32>(h);

    for (fival.index = 0; xioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &fival) == 0; ++fival.index) {
        if (fival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
            rates.push_back({
                static_cast<int>(fival.discrete.numerator),
                static_cast<int>(fival.discrete.denominator),
            });
        } else {
            // Stepwise/continuous — report min interval (max fps)
            rates.push_back({
                static_cast<int>(fival.stepwise.min.numerator),
                static_cast<int>(fival.stepwise.min.denominator),
            });
            break;
        }
    }
    return rates;
}

std::vector<std::pair<FrameSize, std::vector<FrameRate>>>
enumFrameSizes(int fd, __u32 pixfmt) {
    std::vector<std::pair<FrameSize, std::vector<FrameRate>>> sizes;
    v4l2_frmsizeenum fsize{};
    fsize.pixel_format = pixfmt;

    for (fsize.index = 0; xioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fsize) == 0; ++fsize.index) {
        if (fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
            FrameSize sz{static_cast<int>(fsize.discrete.width),
                         static_cast<int>(fsize.discrete.height)};
            auto rates = enumFrameIntervals(fd, pixfmt, sz.width, sz.height);
            sizes.push_back({sz, std::move(rates)});
        } else {
            // Stepwise — report max resolution
            FrameSize sz{static_cast<int>(fsize.stepwise.max_width),
                         static_cast<int>(fsize.stepwise.max_height)};
            auto rates = enumFrameIntervals(fd, pixfmt, sz.width, sz.height);
            sizes.push_back({sz, std::move(rates)});
            break;
        }
    }
    return sizes;
}

std::vector<FormatInfo> enumFormats(int fd) {
    std::vector<FormatInfo> formats;
    v4l2_fmtdesc fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    for (fmt.index = 0; xioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0; ++fmt.index) {
        FormatInfo info;
        info.fourcc = fourccToString(fmt.pixelformat);
        info.description = reinterpret_cast<const char*>(fmt.description);
        info.pixelformat = fmt.pixelformat;
        info.sizes = enumFrameSizes(fd, fmt.pixelformat);
        formats.push_back(std::move(info));
    }
    return formats;
}

std::vector<ControlInfo> enumControls(int fd) {
    std::vector<ControlInfo> controls;

    // Standard controls
    v4l2_queryctrl qctrl{};
    qctrl.id = V4L2_CID_BASE;
    while (qctrl.id < V4L2_CID_LASTP1) {
        if (xioctl(fd, VIDIOC_QUERYCTRL, &qctrl) == 0) {
            if (!(qctrl.flags & V4L2_CTRL_FLAG_DISABLED)) {
                controls.push_back({
                    reinterpret_cast<const char*>(qctrl.name),
                    qctrl.id,
                    qctrl.minimum,
                    qctrl.maximum,
                    qctrl.step,
                    qctrl.default_value,
                    qctrl.type,
                });
            }
        }
        ++qctrl.id;
    }

    // Camera class controls
    qctrl.id = V4L2_CID_CAMERA_CLASS_BASE;
    while (qctrl.id < V4L2_CID_CAMERA_CLASS_BASE + 64) {
        if (xioctl(fd, VIDIOC_QUERYCTRL, &qctrl) == 0) {
            if (!(qctrl.flags & V4L2_CTRL_FLAG_DISABLED)) {
                controls.push_back({
                    reinterpret_cast<const char*>(qctrl.name),
                    qctrl.id,
                    qctrl.minimum,
                    qctrl.maximum,
                    qctrl.step,
                    qctrl.default_value,
                    qctrl.type,
                });
            }
        }
        ++qctrl.id;
    }

    return controls;
}

// Resolve /dev/video* to /dev/v4l/by-id/ and /dev/v4l/by-path/ symlinks
void resolveStablePaths(CameraInfo& cam) {
    namespace fs = std::filesystem;
    auto real = fs::canonical(cam.device_path);

    auto scan = [&](const fs::path& dir, std::string& out) {
        if (!fs::exists(dir)) return;
        for (auto& entry : fs::directory_iterator(dir)) {
            if (fs::is_symlink(entry)) {
                auto target = fs::canonical(entry.path());
                if (target == real) {
                    out = entry.path().string();
                    return;
                }
            }
        }
    };

    scan("/dev/v4l/by-id", cam.by_id_path);
    scan("/dev/v4l/by-path", cam.by_path_path);
}

CameraInfo probeDevice(const std::string& device_path) {
    CameraInfo cam;
    cam.device_path = device_path;

    int fd = open(device_path.c_str(), O_RDWR);
    if (fd < 0) return cam;

    v4l2_capability cap{};
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
        cam.card = reinterpret_cast<const char*>(cap.card);
        cam.driver = reinterpret_cast<const char*>(cap.driver);
        cam.bus_info = reinterpret_cast<const char*>(cap.bus_info);

        if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) {
            cam.formats = enumFormats(fd);
            cam.controls = enumControls(fd);
        }
    }

    close(fd);
    resolveStablePaths(cam);
    return cam;
}

std::vector<CameraInfo> discoverCameras() {
    namespace fs = std::filesystem;
    std::vector<CameraInfo> cameras;
    std::vector<std::string> devices;

    // Find /dev/video* devices
    for (int i = 0; i < 64; ++i) {
        std::string path = "/dev/video" + std::to_string(i);
        if (fs::exists(path)) {
            devices.push_back(path);
        }
    }

    for (auto& path : devices) {
        auto cam = probeDevice(path);
        if (!cam.card.empty() && !cam.formats.empty()) {
            cameras.push_back(std::move(cam));
        }
    }

    return cameras;
}

void printCamera(const CameraInfo& cam, int index) {
    std::printf("\n  %d. %s\n", index, cam.card.c_str());
    std::printf("     Driver:  %s\n", cam.driver.c_str());
    std::printf("     Bus:     %s\n", cam.bus_info.c_str());
    std::printf("     Device:  %s\n", cam.device_path.c_str());
    if (!cam.by_id_path.empty())
        std::printf("     By-ID:   %s\n", cam.by_id_path.c_str());
    if (!cam.by_path_path.empty())
        std::printf("     By-Path: %s\n", cam.by_path_path.c_str());

    std::printf("     Formats:\n");
    for (auto& fmt : cam.formats) {
        std::printf("       %s (%s):\n", fmt.fourcc.c_str(), fmt.description.c_str());
        for (auto& [sz, rates] : fmt.sizes) {
            std::printf("         %dx%d @", sz.width, sz.height);
            for (auto& r : rates) {
                if (r.numerator > 0) {
                    double fps = static_cast<double>(r.denominator) /
                                 static_cast<double>(r.numerator);
                    std::printf(" %.0f", fps);
                }
            }
            std::printf(" fps\n");
        }
    }

    std::printf("     Controls:\n");
    for (auto& ctrl : cam.controls) {
        auto* config_name = cidToConfigName(ctrl.id);
        if (ctrl.type == V4L2_CTRL_TYPE_BOOLEAN) {
            std::printf("       %-32s [%s] : 0..1 (default: %d)",
                        ctrl.name.c_str(), controlTypeStr(ctrl.type).c_str(),
                        ctrl.default_value);
        } else if (ctrl.type == V4L2_CTRL_TYPE_MENU ||
                   ctrl.type == V4L2_CTRL_TYPE_INTEGER_MENU) {
            std::printf("       %-32s [%s] : %d..%d (default: %d)",
                        ctrl.name.c_str(), controlTypeStr(ctrl.type).c_str(),
                        ctrl.minimum, ctrl.maximum, ctrl.default_value);
        } else {
            std::printf("       %-32s [%s] : %d..%d step %d (default: %d)",
                        ctrl.name.c_str(), controlTypeStr(ctrl.type).c_str(),
                        ctrl.minimum, ctrl.maximum, ctrl.step,
                        ctrl.default_value);
        }
        if (config_name) {
            std::printf("  -> xml: %s", config_name);
        }
        std::printf("\n");
    }
}

// Pick best format for XML generation: prefer MJPEG, then YUYV
const FormatInfo* pickBestFormat(const CameraInfo& cam) {
    for (auto& f : cam.formats) {
        if (f.pixelformat == V4L2_PIX_FMT_MJPEG) return &f;
    }
    for (auto& f : cam.formats) {
        if (f.pixelformat == V4L2_PIX_FMT_YUYV) return &f;
    }
    return cam.formats.empty() ? nullptr : &cam.formats[0];
}

void generateXml(const std::vector<CameraInfo>& cameras, const char* output_path) {
    FILE* f = std::fopen(output_path, "w");
    if (!f) {
        std::fprintf(stderr, "Failed to open %s: %s\n", output_path, std::strerror(errno));
        return;
    }

    std::fprintf(f, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    std::fprintf(f, "<cameras>\n");

    for (std::size_t i = 0; i < cameras.size(); ++i) {
        auto& cam = cameras[i];
        std::string cam_id = "cam" + std::to_string(i);

        // Prefer by-id path for stability, fall back to by-path, then raw device
        std::string device = cam.by_id_path;
        if (device.empty()) device = cam.by_path_path;
        if (device.empty()) device = cam.device_path;

        auto* fmt = pickBestFormat(cam);
        if (!fmt) continue;

        // Pick the largest resolution
        int best_w = 0, best_h = 0;
        double best_fps = 30.0;
        for (auto& [sz, rates] : fmt->sizes) {
            if (sz.width * sz.height > best_w * best_h) {
                best_w = sz.width;
                best_h = sz.height;
                // Find fps closest to 30
                double closest = 0;
                for (auto& r : rates) {
                    if (r.numerator > 0) {
                        double fps = static_cast<double>(r.denominator) /
                                     static_cast<double>(r.numerator);
                        if (std::abs(fps - 30.0) < std::abs(closest - 30.0)) {
                            closest = fps;
                        }
                    }
                }
                if (closest > 0) best_fps = closest;
            }
        }

        std::string pf_str;
        if (fmt->pixelformat == V4L2_PIX_FMT_MJPEG) pf_str = "mjpeg";
        else if (fmt->pixelformat == V4L2_PIX_FMT_YUYV) pf_str = "yuyv";
        else pf_str = fmt->fourcc;

        std::fprintf(f, "  <!-- %s (%s) -->\n", cam.card.c_str(), cam.bus_info.c_str());
        std::fprintf(f, "  <camera id=\"%s\" type=\"v4l2\"\n", cam_id.c_str());
        std::fprintf(f, "          device=\"%s\">\n", device.c_str());
        std::fprintf(f, "    <format width=\"%d\" height=\"%d\" fps=\"%.0f\" pixel_format=\"%s\" />\n",
                     best_w, best_h, best_fps, pf_str.c_str());
        std::fprintf(f, "    <controls>\n");

        for (auto& ctrl : cam.controls) {
            auto* config_name = cidToConfigName(ctrl.id);
            if (config_name) {
                std::fprintf(f, "      <control name=\"%s\" value=\"%d\" />\n",
                             config_name, ctrl.default_value);
            }
        }

        std::fprintf(f, "    </controls>\n");
        std::fprintf(f, "  </camera>\n");
    }

    std::fprintf(f, "</cameras>\n");
    std::fclose(f);

    std::printf("Written %s with %zu camera(s).\n", output_path, cameras.size());
}

}  // namespace

int main(int argc, char** argv) {
    const char* generate_path = nullptr;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--generate") == 0) {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "--generate requires an output path\n");
                return 2;
            }
            generate_path = argv[++i];
        } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            std::printf("usage: camera_config_tool [--generate <output.xml>]\n\n"
                        "Enumerates V4L2 cameras and shows capabilities.\n"
                        "With --generate, writes an XML config file usable by V4L2-based pipelines.\n");
            return 0;
        } else {
            std::fprintf(stderr, "unknown arg: %s\n", argv[i]);
            return 2;
        }
    }

    auto cameras = discoverCameras();

    if (cameras.empty()) {
        std::printf("No V4L2 cameras detected.\n");
        return 1;
    }

    std::printf("Detected %zu camera(s):\n", cameras.size());
    for (std::size_t i = 0; i < cameras.size(); ++i) {
        printCamera(cameras[i], static_cast<int>(i + 1));
    }

    if (generate_path) {
        std::printf("\nGenerating config...\n");
        generateXml(cameras, generate_path);
    } else {
        std::printf("\nTo generate a config file, run:\n"
                    "  camera_config_tool --generate config/cameras.xml\n");
    }

    return 0;
}
