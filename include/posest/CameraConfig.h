#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace posest {

struct CameraFormatConfig {
    int width = 1920;
    int height = 1080;
    double fps = 30.0;
    std::string pixel_format = "mjpeg";
};

struct CameraControlEntry {
    std::string name;
    std::int32_t value;
};

struct CameraConfig {
    std::string id;
    std::string type;
    std::string device;
    CameraFormatConfig format;
    std::vector<CameraControlEntry> controls;
};

}  // namespace posest
