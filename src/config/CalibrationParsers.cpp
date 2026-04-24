#include "posest/config/CalibrationParsers.h"

#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace posest::config {

namespace {

double yamlDouble(const YAML::Node& node, const char* field) {
    if (!node) {
        throw std::invalid_argument(std::string("Kalibr calibration missing ") + field);
    }
    return node.as<double>();
}

std::string yamlStringOr(const YAML::Node& node, const char* field, std::string fallback) {
    const auto child = node[field];
    return child ? child.as<std::string>() : std::move(fallback);
}

std::vector<double> yamlDoubleVector(const YAML::Node& node, const char* field) {
    const auto child = node[field];
    if (!child || !child.IsSequence()) {
        throw std::invalid_argument(std::string("Kalibr calibration missing sequence ") + field);
    }
    std::vector<double> values;
    values.reserve(child.size());
    for (const auto& value : child) {
        values.push_back(value.as<double>());
    }
    return values;
}

const YAML::Node chooseKalibrCameraNode(
    const YAML::Node& root,
    const std::optional<std::string>& topic) {
    if (!root || !root.IsMap()) {
        throw std::invalid_argument("Kalibr calibration YAML must be a map");
    }

    YAML::Node fallback;
    for (const auto& entry : root) {
        const YAML::Node camera = entry.second;
        if (!camera || !camera.IsMap()) {
            continue;
        }
        if (!fallback) {
            fallback = camera;
        }
        if (topic) {
            const auto rostopic = camera["rostopic"];
            if (rostopic && rostopic.as<std::string>() == *topic) {
                return camera;
            }
        }
    }

    if (topic) {
        throw std::invalid_argument("Kalibr calibration does not contain requested topic: " + *topic);
    }
    if (!fallback) {
        throw std::invalid_argument("Kalibr calibration contains no camera entries");
    }
    return fallback;
}

double jsonDouble(const nlohmann::json& json, const char* field) {
    if (!json.contains(field) || !json.at(field).is_number()) {
        throw std::invalid_argument(std::string("field layout JSON missing numeric ") + field);
    }
    return json.at(field).get<double>();
}

int jsonIntEither(const nlohmann::json& json, const char* lower, const char* upper) {
    if (json.contains(lower)) {
        return json.at(lower).get<int>();
    }
    if (json.contains(upper)) {
        return json.at(upper).get<int>();
    }
    throw std::invalid_argument("field layout tag missing id");
}

const nlohmann::json& jsonObject(const nlohmann::json& json, const char* field) {
    if (!json.contains(field) || !json.at(field).is_object()) {
        throw std::invalid_argument(std::string("field layout JSON missing object ") + field);
    }
    return json.at(field);
}

std::array<double, 4> readQuaternion(const nlohmann::json& rotation) {
    const auto* q = &rotation;
    if (rotation.contains("quaternion")) {
        q = &rotation.at("quaternion");
    }

    auto value = [q](const char* lower, const char* upper) {
        if (q->contains(lower)) {
            return q->at(lower).get<double>();
        }
        if (q->contains(upper)) {
            return q->at(upper).get<double>();
        }
        throw std::invalid_argument("field layout rotation quaternion is incomplete");
    };

    return {value("w", "W"), value("x", "X"), value("y", "Y"), value("z", "Z")};
}

Vec3 quaternionToRpy(const std::array<double, 4>& q) {
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];

    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);

    const double sinp = 2.0 * (w * y - z * x);
    double pitch = 0.0;
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(std::acos(-1.0) / 2.0, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);

    return {
        std::atan2(sinr_cosp, cosr_cosp),
        pitch,
        std::atan2(siny_cosp, cosy_cosp),
    };
}

Pose3d parseWpilibPose(const nlohmann::json& pose) {
    const auto& translation = jsonObject(pose, "translation");
    const auto& rotation = jsonObject(pose, "rotation");
    Pose3d out;
    out.translation_m = {
        jsonDouble(translation, "x"),
        jsonDouble(translation, "y"),
        jsonDouble(translation, "z"),
    };
    if (rotation.contains("rpy")) {
        const auto& rpy = jsonObject(rotation, "rpy");
        out.rotation_rpy_rad = {
            jsonDouble(rpy, "roll"),
            jsonDouble(rpy, "pitch"),
            jsonDouble(rpy, "yaw"),
        };
    } else {
        out.rotation_rpy_rad = quaternionToRpy(readQuaternion(rotation));
    }
    return out;
}

}  // namespace

runtime::CameraCalibrationConfig parseKalibrCameraCalibration(
    const std::filesystem::path& path,
    const std::string& camera_id,
    const std::string& version,
    bool active,
    const std::string& created_at,
    const std::optional<std::string>& topic) {
    const YAML::Node camera = chooseKalibrCameraNode(YAML::LoadFile(path.string()), topic);
    const auto intrinsics = yamlDoubleVector(camera, "intrinsics");
    const auto resolution = yamlDoubleVector(camera, "resolution");
    if (intrinsics.size() != 4) {
        throw std::invalid_argument("Kalibr intrinsics must contain fx, fy, cx, cy");
    }
    if (resolution.size() != 2) {
        throw std::invalid_argument("Kalibr resolution must contain width and height");
    }

    runtime::CameraCalibrationConfig out;
    out.camera_id = camera_id;
    out.version = version;
    out.active = active;
    out.source_file_path = path.string();
    out.created_at = created_at;
    out.image_width = static_cast<int>(resolution[0]);
    out.image_height = static_cast<int>(resolution[1]);
    out.camera_model = yamlStringOr(camera, "camera_model", "pinhole");
    out.distortion_model = yamlStringOr(camera, "distortion_model", "none");
    out.fx = yamlDouble(camera["intrinsics"][0], "intrinsics[0]");
    out.fy = yamlDouble(camera["intrinsics"][1], "intrinsics[1]");
    out.cx = yamlDouble(camera["intrinsics"][2], "intrinsics[2]");
    out.cy = yamlDouble(camera["intrinsics"][3], "intrinsics[3]");
    out.distortion_coefficients = yamlDoubleVector(camera, "distortion_coeffs");
    return out;
}

runtime::FieldLayoutConfig parseWpilibFieldLayout(
    const std::filesystem::path& path,
    std::string id,
    std::string name) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("failed to open field layout JSON: " + path.string());
    }

    const auto json = nlohmann::json::parse(in);
    const auto& field = jsonObject(json, "field");
    if (!json.contains("tags") || !json.at("tags").is_array()) {
        throw std::invalid_argument("field layout JSON missing tags array");
    }

    runtime::FieldLayoutConfig out;
    out.id = std::move(id);
    out.name = std::move(name);
    out.source_file_path = path.string();
    out.field_length_m = jsonDouble(field, "length");
    out.field_width_m = jsonDouble(field, "width");

    std::unordered_set<int> tag_ids;
    for (const auto& tag : json.at("tags")) {
        const int tag_id = jsonIntEither(tag, "id", "ID");
        if (!tag_ids.insert(tag_id).second) {
            throw std::invalid_argument("field layout JSON contains duplicate tag id");
        }
        out.tags.push_back({tag_id, parseWpilibPose(jsonObject(tag, "pose"))});
    }
    return out;
}

Pose3d parsePoseCsv(const std::string& value) {
    std::array<double, 6> values{};
    std::stringstream in(value);
    std::string token;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (!std::getline(in, token, ',')) {
            throw std::invalid_argument("pose must be x,y,z,roll,pitch,yaw");
        }
        std::size_t consumed = 0;
        values[i] = std::stod(token, &consumed);
        if (consumed != token.size()) {
            throw std::invalid_argument("pose contains a non-numeric value");
        }
    }
    if (std::getline(in, token, ',')) {
        throw std::invalid_argument("pose must contain exactly six values");
    }
    return {{values[0], values[1], values[2]}, {values[3], values[4], values[5]}};
}

}  // namespace posest::config
