#include "posest/vio/KimeraParamWriter.h"

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "posest/vio/KimeraStaticParams.h"

namespace posest::vio {

namespace {

using Matrix4 = std::array<double, 16>;

// 4×4 row-major. Order matches Kimera's `cv::FileStorage` `data` field
// for `T_BS`: r00 r01 r02 tx | r10 r11 r12 ty | r20 r21 r22 tz | 0 0 0 1.
constexpr std::size_t kRowStride = 4;

Matrix4 identity4() {
    Matrix4 m{};
    m[0] = 1.0;
    m[5] = 1.0;
    m[10] = 1.0;
    m[15] = 1.0;
    return m;
}

// RPY → 3×3 rotation matrix matching FusionService::toGtsamPose
// (FusionService.cpp:40): R = Rz(yaw) * Ry(pitch) * Rx(roll). Kept
// here in pure C++ rather than pulling in gtsam — the writer is a
// startup-time helper and does not need the full numeric library.
Matrix4 pose3dToMatrix(const Pose3d& pose) {
    const double r = pose.rotation_rpy_rad.x;  // roll
    const double p = pose.rotation_rpy_rad.y;  // pitch
    const double y = pose.rotation_rpy_rad.z;  // yaw
    const double cr = std::cos(r), sr = std::sin(r);
    const double cp = std::cos(p), sp = std::sin(p);
    const double cy = std::cos(y), sy = std::sin(y);

    Matrix4 m = identity4();
    m[0] = cy * cp;
    m[1] = cy * sp * sr - sy * cr;
    m[2] = cy * sp * cr + sy * sr;
    m[3] = pose.translation_m.x;

    m[4] = sy * cp;
    m[5] = sy * sp * sr + cy * cr;
    m[6] = sy * sp * cr - cy * sr;
    m[7] = pose.translation_m.y;

    m[8] = -sp;
    m[9] = cp * sr;
    m[10] = cp * cr;
    m[11] = pose.translation_m.z;
    return m;
}

double at(const Matrix4& m, std::size_t row, std::size_t col) {
    return m[row * kRowStride + col];
}

double& at(Matrix4& m, std::size_t row, std::size_t col) {
    return m[row * kRowStride + col];
}

// Closed-form inverse of a rigid-body homogeneous transform. Avoids a
// general matrix inverse since the rotation block is orthonormal:
// inverse([R | t]) = [R^T | -R^T * t].
Matrix4 invertRigid(const Matrix4& m) {
    Matrix4 out = identity4();
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            at(out, r, c) = at(m, c, r);
        }
    }
    for (std::size_t r = 0; r < 3; ++r) {
        double t = 0.0;
        for (std::size_t c = 0; c < 3; ++c) {
            t -= at(out, r, c) * at(m, c, 3);
        }
        at(out, r, 3) = t;
    }
    return out;
}

Matrix4 multiply(const Matrix4& a, const Matrix4& b) {
    Matrix4 out{};
    for (std::size_t r = 0; r < 4; ++r) {
        for (std::size_t c = 0; c < 4; ++c) {
            double sum = 0.0;
            for (std::size_t k = 0; k < 4; ++k) {
                sum += at(a, r, k) * at(b, k, c);
            }
            at(out, r, c) = sum;
        }
    }
    return out;
}

const runtime::CameraCalibrationConfig& findActiveCalibration(
    const runtime::RuntimeConfig& cfg, const std::string& camera_id) {
    for (const auto& c : cfg.calibrations) {
        if (c.camera_id == camera_id && c.active) {
            return c;
        }
    }
    throw std::runtime_error(
        "KimeraParamWriter: no active CameraCalibrationConfig for VIO "
        "camera '" + camera_id +
        "'. Run intrinsic calibration before enabling VIO.");
}

const runtime::CameraImuCalibrationConfig& findActiveCameraImu(
    const runtime::RuntimeConfig& cfg, const std::string& camera_id) {
    for (const auto& c : cfg.camera_imu_calibrations) {
        if (c.camera_id == camera_id && c.active) {
            return c;
        }
    }
    throw std::runtime_error(
        "KimeraParamWriter: no active CameraImuCalibrationConfig for "
        "VIO camera '" + camera_id +
        "'. Run camera-IMU calibration before enabling VIO.");
}

// Map Kalibr's distortion-model strings (CalibrationParsers.cpp:178
// reads `distortion_model` from the camchain YAML) onto Kimera's
// vocabulary. Kimera's parser strings are taken from the upstream
// VioParams loader; verify against the installed Kimera version on
// integration. "none" maps to "radial-tangential" with implicit
// zero coefficients (the writer does not synthesize them — Kalibr
// always emits at least an empty list).
std::string_view mapDistortionModel(const std::string& kalibr_name) {
    if (kalibr_name == "radtan" || kalibr_name == "radial-tangential" ||
        kalibr_name == "none") {
        return "radial-tangential";
    }
    if (kalibr_name == "equidistant" || kalibr_name == "equi") {
        return "equidistant";
    }
    throw std::runtime_error(
        "KimeraParamWriter: unsupported Kalibr distortion model: " +
        kalibr_name);
}

// fps for the configured VIO camera, looked up via the cameras vector.
// Kimera's parser tolerates 0 if the field is absent; emitting the real
// rate is preferable for diagnostics.
double cameraRateHz(
    const runtime::RuntimeConfig& cfg, const std::string& camera_id) {
    for (const auto& c : cfg.cameras) {
        if (c.id == camera_id) {
            return c.format.fps;
        }
    }
    return 0.0;
}

void writeMatrix4Block(
    std::ostream& out, std::string_view name, const Matrix4& m) {
    out << name << ":\n"
        << "  rows: 4\n"
        << "  cols: 4\n"
        << "  dt: d\n"
        << "  data: [";
    for (std::size_t i = 0; i < m.size(); ++i) {
        if (i > 0) {
            out << ", ";
        }
        out << std::setprecision(17) << m[i];
    }
    out << "]\n";
}

std::string buildLeftCameraParamsYaml(
    const runtime::RuntimeConfig& cfg,
    const runtime::CameraCalibrationConfig& cam,
    const runtime::CameraImuCalibrationConfig& cam_imu) {
    (void)cfg;
    // T_BS: pose of the camera sensor in the body frame.
    //
    // Kimera enforces that the IMU IS the body frame — its
    // ImuParams::parseYAML LOG(FATAL) aborts unless the IMU's T_BS is
    // identity (Kimera-VIO/src/imu-frontend/ImuFrontendParams.cpp:50).
    // So body == IMU here, and the camera's T_BS is just camera→IMU.
    //
    // CameraImuCalibrationConfig::camera_to_imu stores Kalibr's
    // T_cam_imu (parser at CalibrationParsers.cpp:453), which is the
    // IMU pose expressed in the camera frame — i.e., transforms IMU
    // → camera. Inverting yields camera → IMU == camera → body, which
    // is what Kimera's T_BS field expects.
    //
    // FusionConfig::imu_extrinsic_body_to_imu describes our own
    // (non-Kimera) body↔IMU offset and is intentionally ignored here:
    // mixing it in would push the camera transform outside Kimera's
    // assumed frame and silently bias every published VioMeasurement.
    const Matrix4 T_cam_imu_struct = pose3dToMatrix(cam_imu.camera_to_imu);
    const Matrix4 T_cam_body = invertRigid(T_cam_imu_struct);

    std::ostringstream out;
    out << "%YAML:1.0\n"
        << "# Generated by posest KimeraParamWriter at daemon start —\n"
        << "# do not hand-edit. Source: SQLite calibrations + camera_imu_\n"
        << "# calibrations rows for camera_id='" << cam.camera_id << "'.\n"
        << "camera_id: \"" << cam.camera_id << "\"\n"
        << "rate_hz: " << std::setprecision(17)
        << cameraRateHz(cfg, cam.camera_id) << "\n"
        << "resolution: [" << cam.image_width << ", " << cam.image_height << "]\n"
        << "camera_model: \"" << cam.camera_model << "\"\n"
        << "intrinsics: [" << std::setprecision(17) << cam.fx << ", " << cam.fy
        << ", " << cam.cx << ", " << cam.cy << "]\n"
        << "distortion_model: \"" << mapDistortionModel(cam.distortion_model) << "\"\n"
        << "distortion_coefficients: [";
    for (std::size_t i = 0; i < cam.distortion_coefficients.size(); ++i) {
        if (i > 0) {
            out << ", ";
        }
        out << std::setprecision(17) << cam.distortion_coefficients[i];
    }
    out << "]\n";
    writeMatrix4Block(out, "T_BS", T_cam_body);
    return out.str();
}

std::string buildImuParamsYaml(const runtime::RuntimeConfig& cfg) {
    std::ostringstream out;
    out << static_params::kImuParamsTemplateYaml;
    if (!static_params::kImuParamsTemplateYaml.empty() &&
        static_params::kImuParamsTemplateYaml.back() != '\n') {
        out << '\n';
    }
    // Bias random-walk coefficients. FusionService's preintegration
    // (FusionService.cpp IMU param block) consumes
    // accel_bias_rw_sigma / gyro_bias_rw_sigma directly; Kimera reads
    // the same physical quantity under different names. Sharing the
    // values keeps both preintegrators consistent on this BMI088 unit.
    out << "accelerometer_random_walk: " << std::setprecision(17)
        << cfg.fusion.accel_bias_rw_sigma << '\n';
    out << "gyroscope_random_walk: " << std::setprecision(17)
        << cfg.fusion.gyro_bias_rw_sigma << '\n';
    // FusionConfig::persisted_bias is intentionally NOT threaded through
    // this YAML. Kimera's ImuParams::parseYAML accepts no bias-mean key
    // (Kimera-VIO/src/imu-frontend/ImuFrontendParams.cpp:58-77 reads only
    // imu_bias_init_sigma, which is misleadingly named — it sets
    // PreintegrationParams::biasAccOmegaInt, not a prior on the bias
    // mean: see Kimera-VIO/src/imu-frontend/ImuFrontend.cpp:113). The
    // bias-mean prior lives on the backend (initialAccBiasSigma /
    // initialGyroBiasSigma in BackendParams.yaml), and the *mean* itself
    // is supplied at runtime by InitializationFromImu's static-window
    // estimate — there is no public Pipeline API to inject it from
    // outside. Bridging persisted_bias would require an upstream Kimera
    // change or a fork. FusionService's own ImuFactor chain still uses
    // persisted_bias correctly; only Kimera re-converges per boot.
    // T_BS: IMU sensor pose in body frame. Kimera assumes the IMU IS
    // the body — its ImuParams parser CHECK_FATAL aborts unless this
    // is identity (ImuFrontendParams.cpp:50). Anything off-identity
    // would not just be ignored, it would crash the daemon at start.
    // FusionConfig::imu_extrinsic_body_to_imu is preserved on our own
    // FusionService path and intentionally not threaded through here.
    writeMatrix4Block(out, "T_BS", identity4());
    return out.str();
}

void atomicWrite(const std::filesystem::path& dst, std::string_view content) {
    std::filesystem::path tmp = dst;
    tmp += ".tmp";
    {
        std::ofstream ofs(tmp, std::ios::binary | std::ios::trunc);
        if (!ofs) {
            throw std::runtime_error(
                "KimeraParamWriter: failed to open for writing: " + tmp.string());
        }
        ofs.write(content.data(), static_cast<std::streamsize>(content.size()));
        if (!ofs) {
            throw std::runtime_error(
                "KimeraParamWriter: failed to write: " + tmp.string());
        }
    }
    std::error_code ec;
    std::filesystem::rename(tmp, dst, ec);
    if (ec) {
        throw std::runtime_error(
            "KimeraParamWriter: failed to rename " + tmp.string() +
            " → " + dst.string() + ": " + ec.message());
    }
}

}  // namespace

void emitKimeraParamYamls(
    const runtime::RuntimeConfig& runtime_config,
    const std::filesystem::path& param_dir) {
    if (!runtime_config.vio.enabled) {
        return;
    }
    if (runtime_config.vio.vio_camera_id.empty()) {
        throw std::runtime_error(
            "KimeraParamWriter: vio.enabled is true but vio_camera_id "
            "is empty (validator should have caught this)");
    }

    const auto& cam = findActiveCalibration(
        runtime_config, runtime_config.vio.vio_camera_id);
    const auto& cam_imu = findActiveCameraImu(
        runtime_config, runtime_config.vio.vio_camera_id);

    std::error_code ec;
    std::filesystem::create_directories(param_dir, ec);
    if (ec) {
        throw std::runtime_error(
            "KimeraParamWriter: failed to create param_dir " +
            param_dir.string() + ": " + ec.message());
    }

    atomicWrite(param_dir / "PipelineParams.yaml",
                static_params::kPipelineParamsYaml);
    atomicWrite(param_dir / "FrontendParams.yaml",
                static_params::kFrontendParamsYaml);
    atomicWrite(param_dir / "BackendParams.yaml",
                static_params::kBackendParamsYaml);
    atomicWrite(param_dir / "LcdParams.yaml",
                static_params::kLcdParamsYaml);
    atomicWrite(param_dir / "DisplayParams.yaml",
                static_params::kDisplayParamsYaml);
    atomicWrite(param_dir / "ImuParams.yaml",
                buildImuParamsYaml(runtime_config));
    atomicWrite(param_dir / "LeftCameraParams.yaml",
                buildLeftCameraParamsYaml(runtime_config, cam, cam_imu));
}

}  // namespace posest::vio
