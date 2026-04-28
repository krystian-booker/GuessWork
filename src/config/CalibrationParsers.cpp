#include "posest/config/CalibrationParsers.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace posest::config {

namespace {

std::string trimLeft(const std::string& line) {
    std::size_t i = 0;
    while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i]))) {
        ++i;
    }
    return line.substr(i);
}

// Try to read "cam<N>" off the start of a (left-trimmed) line. Returns the
// label ("cam0", "cam12", ...) when matched, otherwise an empty string.
// Anything may follow — Kalibr writes "cam0:", "cam0 (/topic):", etc.
std::string detectCamLabel(const std::string& trimmed) {
    if (trimmed.size() < 4) return {};
    if (trimmed.compare(0, 3, "cam") != 0) return {};
    std::size_t i = 3;
    while (i < trimmed.size() &&
           std::isdigit(static_cast<unsigned char>(trimmed[i]))) {
        ++i;
    }
    if (i == 3) return {};  // "cam" with no digits — not a label.
    if (i < trimmed.size()) {
        const char next = trimmed[i];
        if (next != ':' && next != ' ' && next != '\t' && next != '(') {
            return {};
        }
    }
    return trimmed.substr(0, i);
}

// Pull the second "[a, b]" pair out of a line shaped
//     reprojection error: [-0.0, 0.0] +- [0.18, 0.19]
// Returns {a, b} on success, std::nullopt otherwise.
std::optional<std::pair<double, double>> extractStdPair(const std::string& line) {
    const auto plus_minus = line.find("+-");
    if (plus_minus == std::string::npos) return std::nullopt;
    const auto open = line.find('[', plus_minus);
    if (open == std::string::npos) return std::nullopt;
    const auto comma = line.find(',', open);
    if (comma == std::string::npos) return std::nullopt;
    const auto close = line.find(']', comma);
    if (close == std::string::npos) return std::nullopt;
    try {
        const double a = std::stod(line.substr(open + 1, comma - open - 1));
        const double b = std::stod(line.substr(comma + 1, close - comma - 1));
        return std::make_pair(a, b);
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

// Pull the first floating-point literal out of a "std:" / "mean:" line.
std::optional<double> extractFirstDouble(const std::string& line) {
    std::size_t i = 0;
    while (i < line.size()) {
        const char ch = line[i];
        if (std::isdigit(static_cast<unsigned char>(ch)) || ch == '-' || ch == '+') {
            try {
                std::size_t consumed = 0;
                const double value = std::stod(line.substr(i), &consumed);
                if (consumed > 0) return value;
            } catch (const std::exception&) {
                return std::nullopt;
            }
        }
        ++i;
    }
    return std::nullopt;
}

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

// Shared YAML → CameraCalibrationConfig conversion. `source_file_path` is
// set by the caller because the camchain path is a single-file convention
// shared by every cam<N> entry the multi-camera parser walks.
runtime::CameraCalibrationConfig cameraConfigFromYamlNode(
    const YAML::Node& camera,
    const std::string& camera_id,
    const std::string& version,
    bool active,
    const std::string& created_at,
    const std::string& source_file_path) {
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
    out.source_file_path = source_file_path;
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

Pose3d poseFromKalibrMatrix(const YAML::Node& matrix, const char* field_name) {
    if (!matrix || !matrix.IsSequence() || matrix.size() != 4u) {
        throw std::invalid_argument(std::string("Kalibr result missing matrix ") + field_name);
    }
    double m[4][4]{};
    for (std::size_t row = 0; row < 4u; ++row) {
        const auto yaml_row = matrix[row];
        if (!yaml_row || !yaml_row.IsSequence() || yaml_row.size() != 4u) {
            throw std::invalid_argument(std::string("Kalibr matrix has invalid row: ") +
                                        field_name);
        }
        for (std::size_t col = 0; col < 4u; ++col) {
            m[row][col] = yaml_row[col].as<double>();
        }
    }

    Pose3d out;
    out.translation_m = {m[0][3], m[1][3], m[2][3]};
    const double sy = std::sqrt(m[0][0] * m[0][0] + m[1][0] * m[1][0]);
    const bool singular = sy < 1e-9;
    if (!singular) {
        out.rotation_rpy_rad = {
            std::atan2(m[2][1], m[2][2]),
            std::atan2(-m[2][0], sy),
            std::atan2(m[1][0], m[0][0]),
        };
    } else {
        out.rotation_rpy_rad = {
            std::atan2(-m[1][2], m[1][1]),
            std::atan2(-m[2][0], sy),
            0.0,
        };
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
    return cameraConfigFromYamlNode(
        camera, camera_id, version, active, created_at, path.string());
}

KalibrCalibrationBundle parseKalibrAllCameras(
    const std::filesystem::path& camchain_path,
    const std::unordered_map<std::string, std::string>& topic_to_camera_id,
    const std::string& version,
    const std::string& created_at) {
    const YAML::Node root = YAML::LoadFile(camchain_path.string());
    if (!root || !root.IsMap()) {
        throw std::invalid_argument(
            "Kalibr camchain YAML must be a map: " + camchain_path.string());
    }

    // Walk root entries in cam-key sort order so iteration is deterministic
    // and matches Kalibr's internal cam0/cam1/... numbering. yaml-cpp's
    // root iteration order is implementation-defined; sort by string key.
    std::map<std::string, YAML::Node> ordered;
    for (const auto& entry : root) {
        const auto key = entry.first.as<std::string>();
        if (key.rfind("cam", 0) != 0) continue;
        ordered.emplace(key, entry.second);
    }

    KalibrCalibrationBundle bundle;
    bundle.cameras.reserve(ordered.size());
    std::string previous_camera_id;
    for (const auto& [kalibr_label, camera_node] : ordered) {
        if (!camera_node || !camera_node.IsMap()) continue;
        const auto rostopic = camera_node["rostopic"];
        if (!rostopic) {
            throw std::invalid_argument(
                "Kalibr camchain entry " + kalibr_label + " has no rostopic");
        }
        const auto topic_key = rostopic.as<std::string>();
        const auto map_it = topic_to_camera_id.find(topic_key);
        if (map_it == topic_to_camera_id.end()) {
            throw std::invalid_argument(
                "Kalibr camchain references rostopic with no GuessWork "
                "camera id mapping: " + topic_key);
        }
        const std::string camera_id = map_it->second;

        bundle.cameras.push_back(cameraConfigFromYamlNode(
            camera_node, camera_id, version, /*active=*/true, created_at,
            camchain_path.string()));

        const auto t_cn_cnm1 = camera_node["T_cn_cnm1"];
        if (t_cn_cnm1) {
            if (previous_camera_id.empty()) {
                throw std::invalid_argument(
                    "Kalibr camchain entry " + kalibr_label +
                    " has T_cn_cnm1 but no preceding camera");
            }
            runtime::CameraToCameraExtrinsicsConfig edge;
            edge.reference_camera_id = previous_camera_id;
            edge.target_camera_id = camera_id;
            edge.version = version;
            edge.target_in_reference =
                poseFromKalibrMatrix(t_cn_cnm1, "T_cn_cnm1");
            bundle.cam_to_cam.push_back(std::move(edge));
        }
        previous_camera_id = camera_id;
    }

    if (bundle.cameras.size() != topic_to_camera_id.size()) {
        throw std::runtime_error(
            "Kalibr camchain returned " + std::to_string(bundle.cameras.size()) +
            " camera entries but " +
            std::to_string(topic_to_camera_id.size()) +
            " were requested — partial result rejected");
    }
    return bundle;
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

runtime::CameraImuCalibrationConfig parseKalibrCameraImuCalibration(
    const std::filesystem::path& path,
    const std::string& camera_id,
    const std::string& version,
    bool active,
    const std::string& created_at,
    const std::optional<std::string>& topic) {
    const YAML::Node camera = chooseKalibrCameraNode(YAML::LoadFile(path.string()), topic);
    const auto t_cam_imu = camera["T_cam_imu"];
    if (!t_cam_imu) {
        throw std::invalid_argument("Kalibr camera-IMU result missing T_cam_imu");
    }

    runtime::CameraImuCalibrationConfig out;
    out.camera_id = camera_id;
    out.version = version;
    out.active = active;
    out.source_file_path = path.string();
    out.created_at = created_at;
    out.camera_to_imu = poseFromKalibrMatrix(t_cam_imu, "T_cam_imu");
    const auto t_imu_cam = camera["T_imu_cam"];
    out.imu_to_camera = t_imu_cam ? poseFromKalibrMatrix(t_imu_cam, "T_imu_cam") : Pose3d{};
    const auto timeshift = camera["timeshift_cam_imu"];
    out.time_shift_s = timeshift ? timeshift.as<double>() : 0.0;
    return out;
}

runtime::CalibrationTargetConfig parseKalibrTargetYaml(
    const std::filesystem::path& path,
    std::string id) {
    const YAML::Node root = YAML::LoadFile(path.string());
    if (!root || !root.IsMap()) {
        throw std::invalid_argument("Kalibr target YAML must be a map: " + path.string());
    }
    const auto type_node = root["target_type"];
    if (!type_node) {
        throw std::invalid_argument("Kalibr target YAML missing target_type: " + path.string());
    }

    runtime::CalibrationTargetConfig out;
    out.id = std::move(id);
    out.type = type_node.as<std::string>();

    if (out.type == "aprilgrid") {
        out.cols = root["tagCols"].as<int>();
        out.rows = root["tagRows"].as<int>();
        out.tag_size_m = root["tagSize"].as<double>();
        out.tag_spacing_ratio = root["tagSpacing"].as<double>();
        const auto family = root["tagFamily"];
        out.tag_family = family ? family.as<std::string>() : std::string("tag36h11");
    } else if (out.type == "checkerboard") {
        out.cols = root["targetCols"].as<int>();
        out.rows = root["targetRows"].as<int>();
        const auto row_spacing = root["rowSpacingMeters"];
        const auto col_spacing = root["colSpacingMeters"];
        if (!row_spacing || !col_spacing) {
            throw std::invalid_argument(
                "Kalibr checkerboard target missing row/col spacing: " + path.string());
        }
        out.square_size_m = row_spacing.as<double>();
        // Kalibr accepts row != col spacing, but our schema collapses them; keep
        // the row value and ignore col when they disagree to avoid silently
        // losing a non-square target.
        if (std::abs(row_spacing.as<double>() - col_spacing.as<double>()) > 1e-9) {
            throw std::invalid_argument(
                "non-square checkerboard targets are not supported: " + path.string());
        }
    } else if (out.type == "circlegrid") {
        out.cols = root["targetCols"].as<int>();
        out.rows = root["targetRows"].as<int>();
        out.square_size_m = root["spacingMeters"].as<double>();
    } else {
        throw std::invalid_argument(
            "unsupported Kalibr target_type: " + out.type);
    }

    return out;
}

std::unordered_map<std::string, KalibrCameraQualityMetrics>
parseKalibrCameraResults(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("failed to open Kalibr results-cam file: " +
                                 path.string());
    }

    std::unordered_map<std::string, KalibrCameraQualityMetrics> out;
    std::string current_cam;
    std::string line;
    while (std::getline(in, line)) {
        const std::string trimmed = trimLeft(line);
        if (trimmed.empty()) {
            continue;
        }
        const std::string label = detectCamLabel(trimmed);
        if (!label.empty()) {
            current_cam = label;
            // Touch the entry so that callers can distinguish "section
            // existed but had no metric" from "section was never seen".
            out.try_emplace(current_cam);
            continue;
        }
        if (current_cam.empty()) continue;
        if (trimmed.find("reprojection error") != std::string::npos) {
            if (const auto pair = extractStdPair(trimmed); pair) {
                out[current_cam].reprojection_rms_px =
                    std::hypot(pair->first, pair->second);
            }
        }
    }
    return out;
}

std::unordered_map<std::string, KalibrCameraImuQualityMetrics>
parseKalibrCameraImuResults(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("failed to open Kalibr results-imucam file: " +
                                 path.string());
    }

    std::unordered_map<std::string, KalibrCameraImuQualityMetrics> out;
    enum class Section { None, CameraReproj, Gyro, Accel };
    Section section = Section::None;
    std::string current_cam;
    double gyro_std = 0.0;
    double accel_std = 0.0;
    bool saw_gyro = false;
    bool saw_accel = false;

    std::string line;
    while (std::getline(in, line)) {
        const std::string trimmed = trimLeft(line);
        if (trimmed.empty()) {
            continue;
        }

        // Section headers reset state. Order matters: check the most
        // specific (camera-keyed) header first.
        if (trimmed.find("Reprojection error") != std::string::npos &&
            trimmed.find("(cam") != std::string::npos) {
            const auto open = trimmed.find('(');
            const auto close = trimmed.find(')', open);
            if (open != std::string::npos && close != std::string::npos &&
                close > open + 1) {
                current_cam = trimmed.substr(open + 1, close - open - 1);
                out.try_emplace(current_cam);
                section = Section::CameraReproj;
            }
            continue;
        }
        if (trimmed.find("Gyroscope error") != std::string::npos) {
            section = Section::Gyro;
            continue;
        }
        if (trimmed.find("Accelerometer error") != std::string::npos) {
            section = Section::Accel;
            continue;
        }

        if (trimmed.find("std") == std::string::npos) continue;
        const auto value = extractFirstDouble(trimmed);
        if (!value) continue;

        switch (section) {
            case Section::CameraReproj:
                if (!current_cam.empty()) {
                    out[current_cam].reprojection_rms_px = *value;
                }
                break;
            case Section::Gyro:
                gyro_std = *value;
                saw_gyro = true;
                break;
            case Section::Accel:
                accel_std = *value;
                saw_accel = true;
                break;
            case Section::None:
                break;
        }
    }

    // Broadcast the IMU error to every discovered camera.
    if (saw_gyro || saw_accel) {
        for (auto& [cam, metrics] : out) {
            (void)cam;
            if (saw_gyro) metrics.gyro_rms_radps = gyro_std;
            if (saw_accel) metrics.accel_rms_mps2 = accel_std;
        }
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
