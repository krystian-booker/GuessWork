#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Typed parse/serialize for Kalibr camchain YAML documents — both the
// intrinsics-only form (kalibr_calibrate_cameras output, what we store in
// cameras.calibration_json) and the camchain-imucam form
// (kalibr_calibrate_imu_camera output, per-camera blocks of which we store
// in cameras.imu_extrinsics_json).
//
// This is the boundary where calibration stops being opaque text: the
// AprilTag pipeline and OpenVINS config builder consume these structs, and
// the single-camera IMU-extrinsics flow serializes a camchain.yaml that
// Kalibr itself must be able to load — the serializers are load-bearing,
// not just for display.
//
// yaml-cpp is a private implementation detail (linked PRIVATE in
// gw_calibration); this header exposes only std types.

namespace gw::calib {

class CalibrationParseError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct CameraIntrinsics {
    std::string             camera_model;        // "pinhole"
    std::array<double, 4>   intrinsics{};        // fu fv pu pv
    std::string             distortion_model;    // "radtan" | "equidistant"
    std::array<double, 4>   distortion_coeffs{};
    std::array<uint32_t, 2> resolution{};        // width height
    std::string             rostopic;            // may be empty
};

// 4×4 row-major homogeneous transform.
using Mat4 = std::array<std::array<double, 4>, 4>;

struct CameraImuExtrinsics {
    Mat4                 T_cam_imu{};            // IMU frame → camera frame
    double               timeshift_cam_imu = 0;  // seconds; t_imu = t_cam + shift
    std::optional<Mat4>  T_cn_cnm1;              // this cam w.r.t. previous cam
};

struct CamchainEntry {
    CameraIntrinsics                   intrinsics;
    std::optional<CameraImuExtrinsics> imu;  // set when parsed from camchain-imucam
};

struct Camchain {
    // Ordered cam0..camN (sorted by the numeric suffix of the key). Non-cam
    // top-level keys (e.g. guesswork_meta) are ignored on parse.
    std::vector<std::pair<std::string, CamchainEntry>> cameras;
};

// Throws CalibrationParseError with a human-readable message on any shape
// violation (missing keys, wrong arity, non-numeric entries). Error strings
// surface in HTTP responses and job upload_error fields.
Camchain parse_camchain(const std::string& yaml_text);

// Kalibr-loadable round-trip serializers.
std::string serialize_camchain(const Camchain& chain);

// One-camera document keyed "cam0:" regardless of the entry's original
// position in the chain — the uniform shape stored per camera row.
std::string serialize_single_camera(const CamchainEntry& entry);

// The guesswork_meta block our Kalibr jobs append to stored calibration
// YAML (quality + provenance). All fields optional — older calibrations
// predate some of them.
struct GuessworkMeta {
    std::optional<std::string> session_id;
    std::optional<double>      reprojection_error_px;        // intrinsics RMS
    std::optional<double>      reprojection_error_mean_px;   // imucam per-cam
    std::optional<double>      reprojection_error_median_px;
    std::optional<double>      reprojection_error_std_px;
    std::optional<double>      timeshift_cam_imu_s;
    std::optional<int>         source_cam_index;
};

// nullopt when the document has no guesswork_meta block. Throws
// CalibrationParseError only on invalid YAML.
std::optional<GuessworkMeta> parse_guesswork_meta(const std::string& yaml_text);

// imu_config.t_imu_robot_json schema (defined here, Phase 3):
//   {"T_robot_imu": [[r00,r01,r02,t0], [...], [...], [0,0,0,1]]}
// 4×4 row-major homogeneous transform mapping IMU-frame points into the
// robot frame, meters. Parsed via yaml-cpp (JSON is a subset of YAML 1.2),
// so consumers of this header never see a JSON library.
//
// Validation: 4×4 numeric shape, bottom row ≈ (0,0,0,1), rotation block
// orthonormal within 1e-3 with det ≈ +1. Throws CalibrationParseError.
Mat4        parse_t_robot_imu(const std::string& json_text);
std::string serialize_t_robot_imu(const Mat4& T);

}  // namespace gw::calib
