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
        require(isFinite(calibration.reprojection_rms_px) &&
                    calibration.reprojection_rms_px >= 0.0,
                "calibration for camera '" + calibration.camera_id +
                    "' reprojection_rms_px must be finite and >= 0");
        require(calibration.observation_count >= 0,
                "calibration for camera '" + calibration.camera_id +
                    "' observation_count must be >= 0");
        require(isFinite(calibration.coverage_score) &&
                    calibration.coverage_score >= 0.0,
                "calibration for camera '" + calibration.camera_id +
                    "' coverage_score must be finite and >= 0");
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
        require(isFinite(calibration.reprojection_rms_px) &&
                    calibration.reprojection_rms_px >= 0.0,
                "camera-IMU calibration for camera '" + calibration.camera_id +
                    "' reprojection_rms_px must be finite and >= 0");
        require(isFinite(calibration.gyro_rms_radps) &&
                    calibration.gyro_rms_radps >= 0.0,
                "camera-IMU calibration for camera '" + calibration.camera_id +
                    "' gyro_rms_radps must be finite and >= 0");
        require(isFinite(calibration.accel_rms_mps2) &&
                    calibration.accel_rms_mps2 >= 0.0,
                "camera-IMU calibration for camera '" + calibration.camera_id +
                    "' accel_rms_mps2 must be finite and >= 0");
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
    require(config.calibration_tools.max_reprojection_rms_px > 0.0 &&
                isFinite(config.calibration_tools.max_reprojection_rms_px),
            "calibration_tools.max_reprojection_rms_px must be finite and > 0");
    require(config.calibration_tools.max_camera_imu_rms_px > 0.0 &&
                isFinite(config.calibration_tools.max_camera_imu_rms_px),
            "calibration_tools.max_camera_imu_rms_px must be finite and > 0");

    std::unordered_set<std::string> calibration_target_ids;
    static const std::unordered_set<std::string> kAllowedTargetTypes{
        "aprilgrid", "checkerboard", "circlegrid"};
    for (const auto& target : config.calibration_targets) {
        require(!target.id.empty(), "calibration target id is empty");
        require(calibration_target_ids.insert(target.id).second,
                "duplicate calibration target id: " + target.id);
        require(kAllowedTargetTypes.count(target.type) == 1,
                "calibration target '" + target.id +
                    "' has unsupported type: " + target.type);
        require(target.rows > 0 && target.cols > 0,
                "calibration target '" + target.id +
                    "' must have positive rows and cols");
        if (target.type == "aprilgrid") {
            require(target.tag_size_m > 0.0 && isFinite(target.tag_size_m),
                    "calibration target '" + target.id +
                        "' aprilgrid tag_size_m must be finite and > 0");
            require(target.tag_spacing_ratio > 0.0 &&
                        isFinite(target.tag_spacing_ratio),
                    "calibration target '" + target.id +
                        "' aprilgrid tag_spacing_ratio must be finite and > 0");
            require(!target.tag_family.empty(),
                    "calibration target '" + target.id +
                        "' aprilgrid tag_family is empty");
        } else {
            require(target.square_size_m > 0.0 && isFinite(target.square_size_m),
                    "calibration target '" + target.id +
                        "' square_size_m must be finite and > 0");
        }
    }

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

    std::unordered_set<std::string> cam_to_cam_keys;
    for (const auto& entry : config.camera_to_camera_extrinsics) {
        require(!entry.reference_camera_id.empty(),
                "camera-to-camera extrinsics has empty reference camera id");
        require(!entry.target_camera_id.empty(),
                "camera-to-camera extrinsics has empty target camera id");
        require(entry.reference_camera_id != entry.target_camera_id,
                "camera-to-camera extrinsics reference and target must differ: " +
                    entry.reference_camera_id);
        require(cameras_enabled.find(entry.reference_camera_id) != cameras_enabled.end(),
                "camera-to-camera extrinsics references unknown reference camera: " +
                    entry.reference_camera_id);
        require(cameras_enabled.find(entry.target_camera_id) != cameras_enabled.end(),
                "camera-to-camera extrinsics references unknown target camera: " +
                    entry.target_camera_id);
        require(!entry.version.empty(),
                "camera-to-camera extrinsics for '" + entry.reference_camera_id +
                    "->" + entry.target_camera_id + "' has empty version");
        const std::string key = entry.reference_camera_id + "\n" +
                                entry.target_camera_id + "\n" + entry.version;
        require(cam_to_cam_keys.insert(key).second,
                "duplicate camera-to-camera extrinsics for triple: " +
                    entry.reference_camera_id + "->" + entry.target_camera_id +
                    "/" + entry.version);
        require(isFinitePose(entry.target_in_reference),
                "camera-to-camera extrinsics for '" + entry.reference_camera_id +
                    "->" + entry.target_camera_id + "' has non-finite pose");
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

    // GTSAM fusion-graph tunables. Bounds are deliberately loose where the
    // graph itself will gracefully degrade and tight where a bad value would
    // crash GTSAM (non-positive sigmas) or silently produce garbage (e.g. an
    // inverted shock/free-fall ordering).
    for (std::size_t i = 0; i < config.fusion.chassis_sigmas.size(); ++i) {
        require(config.fusion.chassis_sigmas[i] > 0.0 &&
                    isFinite(config.fusion.chassis_sigmas[i]),
                "fusion chassis_sigmas[" + std::to_string(i) +
                    "] must be finite and > 0");
    }
    for (std::size_t i = 0; i < config.fusion.origin_prior_sigmas.size(); ++i) {
        require(config.fusion.origin_prior_sigmas[i] > 0.0 &&
                    isFinite(config.fusion.origin_prior_sigmas[i]),
                "fusion origin_prior_sigmas[" + std::to_string(i) +
                    "] must be finite and > 0");
    }
    require(config.fusion.shock_threshold_mps2 > 0.0 &&
                isFinite(config.fusion.shock_threshold_mps2),
            "fusion shock_threshold_mps2 must be finite and > 0");
    require(config.fusion.freefall_threshold_mps2 > 0.0 &&
                isFinite(config.fusion.freefall_threshold_mps2),
            "fusion freefall_threshold_mps2 must be finite and > 0");
    require(config.fusion.shock_threshold_mps2 > config.fusion.freefall_threshold_mps2,
            "fusion shock_threshold_mps2 must be greater than freefall_threshold_mps2");
    require(config.fusion.shock_inflation_factor >= 1.0 &&
                isFinite(config.fusion.shock_inflation_factor),
            "fusion shock_inflation_factor must be finite and >= 1");
    require(config.fusion.imu_window_seconds > 0.0 &&
                isFinite(config.fusion.imu_window_seconds),
            "fusion imu_window_seconds must be finite and > 0");
    require(config.fusion.max_chassis_dt_seconds > 0.0 &&
                isFinite(config.fusion.max_chassis_dt_seconds),
            "fusion max_chassis_dt_seconds must be finite and > 0");
    require(isFinite(config.fusion.gravity_local_mps2.x) &&
                isFinite(config.fusion.gravity_local_mps2.y) &&
                isFinite(config.fusion.gravity_local_mps2.z),
            "fusion gravity_local_mps2 must be finite");
    require(config.fusion.huber_k > 0.0 && isFinite(config.fusion.huber_k),
            "fusion huber_k must be finite and > 0");
    require(isFinitePose(config.fusion.imu_extrinsic_body_to_imu),
            "fusion imu_extrinsic_body_to_imu must be finite");
    require(config.fusion.accel_noise_sigma > 0.0 &&
                isFinite(config.fusion.accel_noise_sigma),
            "fusion accel_noise_sigma must be finite and > 0");
    require(config.fusion.gyro_noise_sigma > 0.0 &&
                isFinite(config.fusion.gyro_noise_sigma),
            "fusion gyro_noise_sigma must be finite and > 0");
    require(config.fusion.accel_bias_rw_sigma > 0.0 &&
                isFinite(config.fusion.accel_bias_rw_sigma),
            "fusion accel_bias_rw_sigma must be finite and > 0");
    require(config.fusion.gyro_bias_rw_sigma > 0.0 &&
                isFinite(config.fusion.gyro_bias_rw_sigma),
            "fusion gyro_bias_rw_sigma must be finite and > 0");
    require(config.fusion.integration_cov_sigma > 0.0 &&
                isFinite(config.fusion.integration_cov_sigma),
            "fusion integration_cov_sigma must be finite and > 0");
    for (std::size_t i = 0; i < config.fusion.persisted_bias.size(); ++i) {
        require(isFinite(config.fusion.persisted_bias[i]),
                "fusion persisted_bias[" + std::to_string(i) + "] must be finite");
    }
    require(config.fusion.bias_calibration_seconds >= 0.5 &&
                isFinite(config.fusion.bias_calibration_seconds),
            "fusion bias_calibration_seconds must be finite and >= 0.5");
    require(config.fusion.bias_calibration_chassis_threshold > 0.0 &&
                isFinite(config.fusion.bias_calibration_chassis_threshold),
            "fusion bias_calibration_chassis_threshold must be finite and > 0");
    require(config.fusion.max_keyframe_dt_seconds >= 0.005 &&
                config.fusion.max_keyframe_dt_seconds <= 0.1 &&
                isFinite(config.fusion.max_keyframe_dt_seconds),
            "fusion max_keyframe_dt_seconds must be finite and in [0.005, 0.1]");
    require(config.fusion.max_imu_gap_seconds > config.fusion.max_keyframe_dt_seconds &&
                isFinite(config.fusion.max_imu_gap_seconds),
            "fusion max_imu_gap_seconds must be finite and greater than max_keyframe_dt_seconds");
    require(config.fusion.marginalize_keyframe_window >= 1u,
            "fusion marginalize_keyframe_window must be >= 1");
    require(config.fusion.slip_disagreement_mps > 0.0 &&
                isFinite(config.fusion.slip_disagreement_mps),
            "fusion slip_disagreement_mps must be finite and > 0");
    // F-2: only validate floor sigmas when the constraint is enabled. With
    // the constraint off the column values are unused, so accept anything
    // finite; with it on the values build a Diagonal::Sigmas noise model and
    // must be strictly positive.
    for (std::size_t i = 0; i < config.fusion.floor_constraint_sigmas.size(); ++i) {
        require(isFinite(config.fusion.floor_constraint_sigmas[i]),
                "fusion floor_constraint_sigmas[" + std::to_string(i) +
                    "] must be finite");
        if (config.fusion.enable_floor_constraint) {
            require(config.fusion.floor_constraint_sigmas[i] > 0.0,
                    "fusion floor_constraint_sigmas[" + std::to_string(i) +
                        "] must be > 0 when enable_floor_constraint is true");
        }
    }
    // F-3: floor at 1 m/s prevents an accidental kill-switch (and the
    // platform cannot meaningfully operate below it).
    require(config.fusion.max_chassis_speed_mps >= 1.0 &&
                isFinite(config.fusion.max_chassis_speed_mps),
            "fusion max_chassis_speed_mps must be finite and >= 1.0");
}

}  // namespace posest::config
