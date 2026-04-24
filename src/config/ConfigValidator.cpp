#include "posest/config/ConfigValidator.h"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <nlohmann/json.hpp>

namespace posest::config {

namespace {

constexpr std::size_t kMaxEnabledCameraTriggers = 6;

bool isObjectJson(const std::string& value) {
    if (value.empty()) {
        return true;
    }
    try {
        return nlohmann::json::parse(value).is_object();
    } catch (const nlohmann::json::parse_error&) {
        return false;
    }
}

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::invalid_argument("Invalid runtime config: " + message);
    }
}

}  // namespace

void validateRuntimeConfig(const runtime::RuntimeConfig& config) {
    std::unordered_map<std::string, bool> cameras_enabled;
    for (const auto& camera : config.cameras) {
        require(!camera.id.empty(), "camera id is empty");
        require(cameras_enabled.emplace(camera.id, camera.enabled).second,
                "duplicate camera id: " + camera.id);
        require(!camera.type.empty(), "camera '" + camera.id + "' has empty type");
        require(!camera.device.empty(), "camera '" + camera.id + "' has empty device");
        require(camera.format.width > 0, "camera '" + camera.id + "' width must be > 0");
        require(camera.format.height > 0, "camera '" + camera.id + "' height must be > 0");
        require(camera.format.fps > 0.0, "camera '" + camera.id + "' fps must be > 0");
        require(!camera.format.pixel_format.empty(),
                "camera '" + camera.id + "' pixel format is empty");

        std::unordered_set<std::string> controls;
        for (const auto& control : camera.controls) {
            require(!control.name.empty(), "camera '" + camera.id + "' has empty control name");
            require(controls.insert(control.name).second,
                    "camera '" + camera.id + "' has duplicate control: " + control.name);
        }
    }

    std::unordered_map<std::string, bool> pipelines_enabled;
    for (const auto& pipeline : config.pipelines) {
        require(!pipeline.id.empty(), "pipeline id is empty");
        require(pipelines_enabled.emplace(pipeline.id, pipeline.enabled).second,
                "duplicate pipeline id: " + pipeline.id);
        require(!pipeline.type.empty(), "pipeline '" + pipeline.id + "' has empty type");
        require(isObjectJson(pipeline.parameters_json),
                "pipeline '" + pipeline.id + "' parameters_json must be empty or a JSON object");
    }

    std::unordered_set<std::string> bindings;
    for (const auto& binding : config.bindings) {
        require(!binding.camera_id.empty(), "binding has empty camera id");
        require(!binding.pipeline_id.empty(), "binding has empty pipeline id");
        const auto camera_it = cameras_enabled.find(binding.camera_id);
        require(camera_it != cameras_enabled.end(),
                "binding references unknown camera: " + binding.camera_id);
        require(camera_it->second,
                "binding references disabled camera: " + binding.camera_id);
        const auto pipeline_it = pipelines_enabled.find(binding.pipeline_id);
        require(pipeline_it != pipelines_enabled.end(),
                "binding references unknown pipeline: " + binding.pipeline_id);
        require(pipeline_it->second,
                "binding references disabled pipeline: " + binding.pipeline_id);
        require(bindings.insert(binding.camera_id + "\n" + binding.pipeline_id).second,
                "duplicate binding: " + binding.camera_id + " -> " + binding.pipeline_id);
    }

    std::unordered_set<std::string> calibrations;
    for (const auto& calibration : config.calibrations) {
        require(!calibration.camera_id.empty(), "calibration has empty camera id");
        require(cameras_enabled.find(calibration.camera_id) != cameras_enabled.end(),
                "calibration references unknown camera: " + calibration.camera_id);
        require(!calibration.file_path.empty(),
                "calibration for camera '" + calibration.camera_id + "' has empty file path");
        require(!calibration.version.empty(),
                "calibration for camera '" + calibration.camera_id + "' has empty version");
        require(calibrations.insert(calibration.camera_id + "\n" + calibration.version).second,
                "duplicate calibration for camera/version: " + calibration.camera_id + "/" +
                    calibration.version);
    }

    std::unordered_set<std::string> trigger_cameras;
    std::unordered_set<std::int32_t> enabled_trigger_pins;
    std::size_t enabled_trigger_count = 0;
    for (const auto& trigger : config.camera_triggers) {
        require(!trigger.camera_id.empty(), "camera trigger has empty camera id");
        require(cameras_enabled.find(trigger.camera_id) != cameras_enabled.end(),
                "camera trigger references unknown camera: " + trigger.camera_id);
        require(trigger_cameras.insert(trigger.camera_id).second,
                "duplicate camera trigger for camera: " + trigger.camera_id);
        require(trigger.teensy_pin >= 0,
                "camera trigger for camera '" + trigger.camera_id + "' has invalid Teensy pin");
        require(trigger.rate_hz > 0.0,
                "camera trigger for camera '" + trigger.camera_id + "' rate must be > 0");
        require(trigger.pulse_width_us > 0,
                "camera trigger for camera '" + trigger.camera_id +
                    "' pulse width must be > 0");
        if (trigger.enabled) {
            ++enabled_trigger_count;
            require(enabled_trigger_pins.insert(trigger.teensy_pin).second,
                    "duplicate enabled camera trigger pin: " +
                        std::to_string(trigger.teensy_pin));
        }
    }
    require(enabled_trigger_count <= kMaxEnabledCameraTriggers,
            "enabled camera trigger count must be <= 6");

    require(config.teensy.baud_rate > 0, "Teensy baud rate must be > 0");
    require(config.teensy.reconnect_interval_ms > 0,
            "Teensy reconnect interval must be > 0");
    require(config.teensy.read_timeout_ms > 0, "Teensy read timeout must be > 0");
    require(config.teensy.pose_publish_hz > 0.0, "Teensy pose publish rate must be > 0");
}

}  // namespace posest::config
