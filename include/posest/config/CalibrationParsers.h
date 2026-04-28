#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>

#include "posest/MeasurementTypes.h"
#include "posest/runtime/RuntimeConfig.h"

namespace posest::config {

// W2 quality metrics extracted from Kalibr's results-cam-<bag>.txt.
// Keyed by Kalibr camera label ("cam0", "cam1", ...) — the caller maps
// those labels to GuessWork camera ids via the topic→id table built
// from CLI flags.
struct KalibrCameraQualityMetrics {
    double reprojection_rms_px{0.0};
    std::int32_t observation_count{0};
};

// Same shape, extracted from Kalibr's results-imucam-<bag>.txt. Gyro
// and accel errors are per-IMU and broadcast to every camera in the
// returned map.
struct KalibrCameraImuQualityMetrics {
    double reprojection_rms_px{0.0};
    double gyro_rms_radps{0.0};
    double accel_rms_mps2{0.0};
};

// W3 multi-camera bundle from Kalibr's camchain.yaml. `cameras` is in
// Kalibr iteration order (cam0, cam1, ...). `cam_to_cam` carries the
// adjacent-pair T_cn_cnm1 baselines (cam0→cam1, cam1→cam2, ...). The
// W2 quality metrics on each CameraCalibrationConfig are NOT filled by
// this parser; callers stitch them in from parseKalibrCameraResults.
struct KalibrCalibrationBundle {
    std::vector<runtime::CameraCalibrationConfig> cameras;
    std::vector<runtime::CameraToCameraExtrinsicsConfig> cam_to_cam;
};

runtime::CameraCalibrationConfig parseKalibrCameraCalibration(
    const std::filesystem::path& path,
    const std::string& camera_id,
    const std::string& version,
    bool active,
    const std::string& created_at,
    const std::optional<std::string>& topic = std::nullopt);

runtime::FieldLayoutConfig parseWpilibFieldLayout(
    const std::filesystem::path& path,
    std::string id,
    std::string name);

runtime::CameraImuCalibrationConfig parseKalibrCameraImuCalibration(
    const std::filesystem::path& path,
    const std::string& camera_id,
    const std::string& version,
    bool active,
    const std::string& created_at,
    const std::optional<std::string>& topic = std::nullopt);

Pose3d parsePoseCsv(const std::string& value);

// Parse a Kalibr target.yaml (aprilgrid / checkerboard / circlegrid) and
// return a CalibrationTargetConfig with the caller-supplied id.
runtime::CalibrationTargetConfig parseKalibrTargetYaml(
    const std::filesystem::path& path,
    std::string id);

// Parse Kalibr's results-cam-<bag>.txt (camera-only calibration).
// Returns an empty map if no camera sections were found; throws only
// when the file cannot be opened. The reprojection RMS is computed as
// hypot(std_x, std_y) from the per-camera "reprojection error: [a, b]
// +- [c, d]" line.
std::unordered_map<std::string, KalibrCameraQualityMetrics>
parseKalibrCameraResults(const std::filesystem::path& path);

// Parse Kalibr's results-imucam-<bag>.txt (camera-IMU calibration).
// The reprojection-error block is per-camera; the gyroscope/accelerometer
// error blocks are global to the run and broadcast to every camera in
// the result map. Missing sections leave their fields at 0.0.
std::unordered_map<std::string, KalibrCameraImuQualityMetrics>
parseKalibrCameraImuResults(const std::filesystem::path& path);

// Walk every cam<N> entry in Kalibr's camchain.yaml. The
// topic_to_camera_id map translates Kalibr's `rostopic` field into the
// GuessWork camera id used by the CalibrationConfig rows. Throws if a
// rostopic is not found in the map (CLI/Kalibr disagreement) or if the
// number of cam<N> entries does not match the size of the map (partial
// Kalibr result — fail-safe). active=true on every returned camera.
KalibrCalibrationBundle parseKalibrAllCameras(
    const std::filesystem::path& camchain_path,
    const std::unordered_map<std::string, std::string>& topic_to_camera_id,
    const std::string& version,
    const std::string& created_at);

}  // namespace posest::config
