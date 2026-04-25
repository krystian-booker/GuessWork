#include "posest/V4L2DeviceEnumerator.h"

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/videodev2.h>

namespace posest::v4l2 {

namespace {

namespace fs = std::filesystem;

int xioctl(int fd, unsigned long request, void* arg) {
    int r;
    do {
        r = ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

// Map /dev/videoN -> stable /dev/v4l/by-id/* path when one exists.
std::unordered_map<std::string, std::string> resolveStablePaths() {
    std::unordered_map<std::string, std::string> out;
    const fs::path by_id{"/dev/v4l/by-id"};
    std::error_code ec;
    if (!fs::exists(by_id, ec)) return out;
    for (auto it = fs::directory_iterator(by_id, ec);
         it != fs::directory_iterator(); ++it) {
        std::error_code link_ec;
        const auto target = fs::canonical(it->path(), link_ec);
        if (link_ec) continue;
        out.emplace(target.string(), it->path().string());
    }
    return out;
}

std::string readNameFromSysfs(const std::string& dev_basename) {
    const fs::path name_path =
        fs::path("/sys/class/video4linux") / dev_basename / "name";
    std::ifstream in(name_path);
    std::string line;
    if (in && std::getline(in, line)) {
        return line;
    }
    return {};
}

bool deviceUsable(const std::string& device_path, std::string& human_name) {
    const int fd = open(device_path.c_str(), O_RDWR);
    if (fd < 0) return false;
    v4l2_capability cap{};
    const bool query_ok = xioctl(fd, VIDIOC_QUERYCAP, &cap) == 0;
    close(fd);
    if (!query_ok) return false;
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) return false;
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) return false;
    human_name = reinterpret_cast<const char*>(cap.card);
    return true;
}

}  // namespace

std::vector<CameraCapabilities> enumerateDevices() {
    std::vector<CameraCapabilities> out;
    const fs::path sys_root{"/sys/class/video4linux"};
    std::error_code ec;
    if (!fs::exists(sys_root, ec)) {
        return out;
    }

    const auto stable_paths = resolveStablePaths();

    for (const auto& entry : fs::directory_iterator(sys_root, ec)) {
        const auto basename = entry.path().filename().string();
        if (basename.rfind("video", 0) != 0) continue;
        const std::string dev_path = "/dev/" + basename;
        if (!fs::exists(dev_path, ec)) continue;

        std::string human_name;
        if (!deviceUsable(dev_path, human_name)) continue;

        std::string stable = dev_path;
        if (auto it = stable_paths.find(dev_path); it != stable_paths.end()) {
            stable = it->second;
        }

        const std::string sysfs_name = readNameFromSysfs(basename);

        CameraCapabilities caps;
        caps.backend = "v4l2";
        caps.device_hint.kind = DeviceHintKind::DevicePath;
        caps.device_hint.description = stable;
        caps.trigger_modes = {TriggerMode::FreeRun};
        caps.supports_set_control = true;
        caps.supports_get_control = true;
        caps.supports_reconnect = true;
        caps.supports_set_trigger_mode = false;
        if (!sysfs_name.empty()) {
            caps.camera_id = sysfs_name;
        } else if (!human_name.empty()) {
            caps.camera_id = human_name;
        }
        out.push_back(std::move(caps));
    }

    return out;
}

}  // namespace posest::v4l2
