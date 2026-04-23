#pragma once

#include <filesystem>
#include <string>

#include "posest/ICameraConfigSource.h"

namespace posest {

class XmlCameraConfigSource final : public ICameraConfigSource {
public:
    explicit XmlCameraConfigSource(std::filesystem::path xml_path);

    static XmlCameraConfigSource fromString(std::string xml_content);

    std::vector<CameraConfig> loadAll() const override;

private:
    explicit XmlCameraConfigSource(std::string xml_content, bool from_string);

    std::filesystem::path path_;
    std::string content_;
    bool from_string_{false};
};

}  // namespace posest
