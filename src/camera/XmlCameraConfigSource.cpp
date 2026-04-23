#include "posest/XmlCameraConfigSource.h"

#include <stdexcept>
#include <utility>

#include <pugixml.hpp>

namespace posest {

XmlCameraConfigSource::XmlCameraConfigSource(std::filesystem::path xml_path)
    : path_(std::move(xml_path)) {}

XmlCameraConfigSource::XmlCameraConfigSource(std::string xml_content, bool /*from_string*/)
    : content_(std::move(xml_content)), from_string_(true) {}

XmlCameraConfigSource XmlCameraConfigSource::fromString(std::string xml_content) {
    return XmlCameraConfigSource(std::move(xml_content), true);
}

std::vector<CameraConfig> XmlCameraConfigSource::loadAll() const {
    pugi::xml_document doc;
    pugi::xml_parse_result result;

    if (from_string_) {
        result = doc.load_string(content_.c_str());
    } else {
        result = doc.load_file(path_.c_str());
    }

    if (!result) {
        throw std::runtime_error(
            std::string("Failed to parse camera config XML: ") + result.description());
    }

    auto cameras_node = doc.child("cameras");
    if (!cameras_node) {
        throw std::runtime_error("Missing <cameras> root element");
    }

    std::vector<CameraConfig> configs;

    for (auto camera_node : cameras_node.children("camera")) {
        auto id_attr = camera_node.attribute("id");
        auto type_attr = camera_node.attribute("type");
        auto device_attr = camera_node.attribute("device");

        if (!id_attr) {
            throw std::runtime_error("Camera element missing required 'id' attribute");
        }
        if (!type_attr) {
            throw std::runtime_error(
                std::string("Camera '") + id_attr.value() +
                "' missing required 'type' attribute");
        }
        if (!device_attr) {
            throw std::runtime_error(
                std::string("Camera '") + id_attr.value() +
                "' missing required 'device' attribute");
        }

        CameraConfig cfg;
        cfg.id = id_attr.value();
        cfg.type = type_attr.value();
        cfg.device = device_attr.value();

        if (auto fmt = camera_node.child("format")) {
            cfg.format.width = fmt.attribute("width").as_int(cfg.format.width);
            cfg.format.height = fmt.attribute("height").as_int(cfg.format.height);
            cfg.format.fps = fmt.attribute("fps").as_double(cfg.format.fps);
            auto pf = fmt.attribute("pixel_format");
            if (pf) {
                cfg.format.pixel_format = pf.value();
            }
        }

        if (auto controls_node = camera_node.child("controls")) {
            for (auto ctrl : controls_node.children("control")) {
                auto name_attr = ctrl.attribute("name");
                auto value_attr = ctrl.attribute("value");
                if (name_attr && value_attr) {
                    cfg.controls.push_back({
                        name_attr.value(),
                        static_cast<std::int32_t>(value_attr.as_int()),
                    });
                }
            }
        }

        configs.push_back(std::move(cfg));
    }

    return configs;
}

}  // namespace posest
