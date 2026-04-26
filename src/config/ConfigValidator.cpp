#include "posest/config/ConfigValidator.h"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cmath>
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

bool isFinite(double value) {
    return std::isfinite(value);
}

bool isFinitePose(const Pose3d& pose) {
    return isFinite(pose.translation_m.x) && isFinite(pose.translation_m.y) &&
           isFinite(pose.translation_m.z) && isFinite(pose.rotation_rpy_rad.x) &&
           isFinite(pose.rotation_rpy_rad.y) && isFinite(pose.rotation_rpy_rad.z);
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

        // trigger_mode is a typed enum so any in-range value is acceptable;
        // round-tripping through the canonical string vocabulary catches a
        // bit-pattern that would not survive serialization.
        require(triggerModeFromString(triggerModeToString(camera.trigger_mode))
                    .has_value(),
                "camera '" + camera.id + "' has invalid trigger_mode");
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
    std::unordered_set<std::string> active_calibration_cameras;
    for (const auto& calibration : config.calibrations) {
        require(!calibration.camera_id.empty(), "calibration has empty camera id");
        require(cameras_enabled.find(calibration.camera_id) != cameras_enabled.end(),
                "calibration references unknown camera: " + calibration.camera_id);
        require(!calibration.source_file_path.empty(),
                "calibration for camera '" + calibration.camera_id + "' has empty source file path");
        require(!calibration.version.empty(),
                "calibration for camera '" + calibration.camera_id + "' has empty version");
        require(calibrations.insert(calibration.camera_id + "\n" + calibration.version).second,
                "duplicate calibration for camera/version: " + calibration.camera_id + "/" +
                    calibration.version);
        require(calibration.image_width > 0,
                "calibration for camera '" + calibration.camera_id + "' width must be > 0");
        require(calibration.image_height > 0,
                "calibration for camera '" + calibration.camera_id + "' height must be > 0");
        require(!calibration.camera_model.empty(),
                "calibration for camera '" + calibration.camera_id + "' has empty camera model");
        require(!calibration.distortion_model.empty(),
                "calibration for camera '" + calibration.camera_id + "' has empty distortion model");
        require(calibration.fx > 0.0 && isFinite(calibration.fx),
                "calibration for camera '" + calibration.camera_id + "' fx must be finite and > 0");
        require(calibration.fy > 0.0 && isFinite(calibration.fy),
                "calibration for camera '" + calibration.camera_id + "' fy must be finite and > 0");
        require(isFinite(calibration.cx) && isFinite(calibration.cy),
                "calibration for camera '" + calibration.camera_id + "' principal point must be finite");
        require(std::all_of(
                    calibration.distortion_coefficients.begin(),
                    calibration.distortion_coefficients.end(),
                    isFinite),
                "calibration for camera '" + calibration.camera_id +
                    "' has non-finite distortion coefficient");
        if (calibration.active) {
            require(active_calibration_cameras.insert(calibration.camera_id).second,
                    "multiple active calibrations for camera: " + calibration.camera_id);
        }
    }

    std::unordered_set<std::string> active_camera_imu_cameras;
    std::unordered_set<std::string> camera_imu_calibrations;
    for (const auto& calibration : config.camera_imu_calibrations) {
        require(!calibration.camera_id.empty(), "camera-IMU calibration has empty camera id");
        require(cameras_enabled.find(calibration.camera_id) != cameras_enabled.end(),
                "camera-IMU calibration references unknown camera: " + calibration.camera_id);
        require(!calibration.version.empty(),
                "camera-IMU calibration for camera '" + calibration.camera_id +
                    "' has empty version");
        require(!calibration.source_file_path.empty(),
                "camera-IMU calibration for camera '" + calibration.camera_id +
                    "' has empty source file path");
        require(camera_imu_calibrations
                    .insert(calibration.camera_id + "\n" + calibration.version)
                    .second,
                "duplicate camera-IMU calibration for camera/version: " +
                    calibration.camera_id + "/" + calibration.version);
        require(isFinitePose(calibration.camera_to_imu) &&
                    isFinitePose(calibration.imu_to_camera) &&
                    isFinite(calibration.time_shift_s),
                "camera-IMU calibration for camera '" + calibration.camera_id +
                    "' has non-finite value");
        if (calibration.active) {
            require(active_camera_imu_cameras.insert(calibration.camera_id).second,
                    "multiple active camera-IMU calibrations for camera: " +
                        calibration.camera_id);
        }
    }

    std::unordered_set<std::string> dataset_ids;
    for (const auto& dataset : config.kalibr_datasets) {
        require(!dataset.id.empty(), "Kalibr dataset id is empty");
        require(dataset_ids.insert(dataset.id).second,
                "duplicate Kalibr dataset id: " + dataset.id);
        require(!dataset.path.empty(), "Kalibr dataset '" + dataset.id + "' has empty path");
        require(dataset.duration_s >= 0.0 && isFinite(dataset.duration_s),
                "Kalibr dataset '" + dataset.id + "' has invalid duration");
        std::unordered_set<std::string> dataset_cameras;
        for (const auto& camera_id : dataset.camera_ids) {
            require(cameras_enabled.find(camera_id) != cameras_enabled.end(),
                    "Kalibr dataset references unknown camera: " + camera_id);
            require(dataset_cameras.insert(camera_id).second,
                    "Kalibr dataset has duplicate camera id: " + camera_id);
        }
    }
    require(!config.calibration_tools.docker_image.empty(),
            "Kalibr Docker image default is empty");

    std::unordered_set<std::string> extrinsics;
    for (const auto& entry : config.camera_extrinsics) {
        require(!entry.camera_id.empty(), "camera extrinsics has empty camera id");
        require(!entry.version.empty(),
                "camera extrinsics for camera '" + entry.camera_id + "' has empty version");
        const std::string key = entry.camera_id + "\n" + entry.version;
        require(calibrations.find(key) != calibrations.end(),
                "camera extrinsics references unknown calibration: " + entry.camera_id + "/" +
                    entry.version);
        require(extrinsics.insert(key).second,
                "duplicate camera extrinsics for camera/version: " + entry.camera_id + "/" +
                    entry.version);
        require(isFinitePose(entry.camera_to_robot),
                "camera extrinsics for camera '" + entry.camera_id + "' has non-finite pose");
    }
    for (const auto& calibration : config.calibrations) {
        if (calibration.active) {
            const std::string key = calibration.camera_id + "\n" + calibration.version;
            require(extrinsics.find(key) != extrinsics.end(),
                    "active calibration missing camera-to-robot extrinsics: " +
                        calibration.camera_id + "/" + calibration.version);
        }
    }

    std::unordered_set<std::string> field_layout_ids;
    bool saw_active_field_layout = config.active_field_layout_id.empty();
    for (const auto& layout : config.field_layouts) {
        require(!layout.id.empty(), "field layout id is empty");
        require(field_layout_ids.insert(layout.id).second,
                "duplicate field layout id: " + layout.id);
        require(!layout.name.empty(), "field layout '" + layout.id + "' has empty name");
        require(!layout.source_file_path.empty(),
                "field layout '" + layout.id + "' has empty source file path");
        require(layout.field_length_m > 0.0 && isFinite(layout.field_length_m),
                "field layout '" + layout.id + "' length must be finite and > 0");
        require(layout.field_width_m > 0.0 && isFinite(layout.field_width_m),
                "field layout '" + layout.id + "' width must be finite and > 0");
        std::unordered_set<int> tag_ids;
        for (const auto& tag : layout.tags) {
            require(tag.tag_id > 0,
                    "field layout '" + layout.id + "' has invalid tag id");
            require(tag_ids.insert(tag.tag_id).second,
                    "field layout '" + layout.id + "' has duplicate tag id: " +
                        std::to_string(tag.tag_id));
            require(isFinitePose(tag.field_to_tag),
                    "field layout '" + layout.id + "' has non-finite tag pose");
        }
        if (layout.id == config.active_field_layout_id) {
            saw_active_field_layout = true;
        }
    }
    require(saw_active_field_layout,
            "active field layout references unknown layout: " + config.active_field_layout_id);

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

    // VIO companion workflow. The temporal-multiplex invariant is enforced
    // here: the ToF ranging window must start strictly after the IR LED
    // flash window has closed.
    require(config.vio.vio_slot_index >= 0 &&
                config.vio.vio_slot_index < static_cast<std::int32_t>(kMaxEnabledCameraTriggers),
            "VIO slot index must be in [0, " +
                std::to_string(kMaxEnabledCameraTriggers) + ")");
    require(config.vio.tof_divisor >= 1, "VIO ToF divisor must be >= 1");
    require(config.vio.tof_intermeasurement_period_ms >
                config.vio.tof_timing_budget_ms,
            "VIO ToF intermeasurement period must be greater than timing budget");
    require(config.vio.tof_offset_after_flash_us >= config.vio.ir_led_pulse_width_us,
            "VIO ToF offset after flash must be >= IR LED pulse width "
            "(temporal multiplex invariant: flash and ranging cannot overlap)");
    require(isFinite(config.vio.tof_mounting_offset_m),
            "VIO ToF mounting offset must be finite");
    require(isFinite(config.vio.tof_expected_min_m) &&
                isFinite(config.vio.tof_expected_max_m) &&
                config.vio.tof_expected_min_m < config.vio.tof_expected_max_m,
            "VIO ToF expected range must be finite and min < max");
    if (config.vio.enabled) {
        require(!config.vio.vio_camera_id.empty(),
                "VIO is enabled but vio_camera_id is empty");
        require(cameras_enabled.find(config.vio.vio_camera_id) != cameras_enabled.end(),
                "VIO references unknown camera: " + config.vio.vio_camera_id);
    }
}

}  // namespace posest::config
