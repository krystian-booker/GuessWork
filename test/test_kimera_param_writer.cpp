#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "posest/runtime/RuntimeConfig.h"
#include "posest/vio/KimeraParamWriter.h"

namespace {

std::filesystem::path tempDir(const std::string& tag) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    auto p = std::filesystem::temp_directory_path() /
             ("posest_kimera_yaml_" + tag + "_" + std::to_string(stamp));
    std::filesystem::create_directories(p);
    return p;
}

std::string readWhole(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

posest::runtime::RuntimeConfig makeVioReadyConfig() {
    posest::runtime::RuntimeConfig config;

    posest::CameraConfig cam;
    cam.id = "downward";
    cam.type = "v4l2";
    cam.device = "/dev/video0";
    cam.enabled = true;
    cam.format.width = 640;
    cam.format.height = 480;
    cam.format.fps = 120.0;
    cam.format.pixel_format = "mjpeg";
    config.cameras.push_back(cam);

    posest::runtime::CameraCalibrationConfig calib;
    calib.camera_id = "downward";
    calib.version = "v1";
    calib.active = true;
    calib.source_file_path = "/tmp/calib.yaml";
    calib.created_at = "2026-04-27T00:00:00Z";
    calib.image_width = 640;
    calib.image_height = 480;
    calib.camera_model = "pinhole";
    calib.distortion_model = "radtan";
    calib.fx = 500.0;
    calib.fy = 501.0;
    calib.cx = 320.0;
    calib.cy = 240.0;
    calib.distortion_coefficients = {0.1, -0.05, 0.001, -0.001};
    config.calibrations.push_back(calib);

    posest::runtime::CameraImuCalibrationConfig cam_imu;
    cam_imu.camera_id = "downward";
    cam_imu.version = "imu-v1";
    cam_imu.active = true;
    cam_imu.source_file_path = "/tmp/imucam.yaml";
    cam_imu.created_at = "2026-04-27T00:01:00Z";
    cam_imu.camera_to_imu = {{0.01, 0.02, 0.03}, {0.0, 0.0, 0.0}};
    cam_imu.imu_to_camera = {{-0.01, -0.02, -0.03}, {0.0, 0.0, 0.0}};
    cam_imu.time_shift_s = 0.004;
    config.camera_imu_calibrations.push_back(cam_imu);

    config.vio.enabled = true;
    config.vio.vio_camera_id = "downward";

    // Non-default body↔IMU extrinsic so the test can prove it survives the
    // round-trip into IMUtoBodyT_BS rather than just observing identity.
    config.fusion.imu_extrinsic_body_to_imu.translation_m = {0.05, 0.0, 0.10};
    return config;
}

}  // namespace

TEST(KimeraParamWriter, EmitsAllFourFilesWhenVioEnabled) {
    const auto dir = tempDir("emits_all");
    const auto cfg = makeVioReadyConfig();

    posest::vio::emitKimeraParamYamls(cfg, dir);

    // Kimera's VioParams(folder) ctor unconditionally parses all of
    // these — leaving any of them out crashes start() with a YAML open
    // failure. See Pipeline-definitions.cpp:109-185.
    EXPECT_TRUE(std::filesystem::exists(dir / "PipelineParams.yaml"));
    EXPECT_TRUE(std::filesystem::exists(dir / "FrontendParams.yaml"));
    EXPECT_TRUE(std::filesystem::exists(dir / "BackendParams.yaml"));
    EXPECT_TRUE(std::filesystem::exists(dir / "LcdParams.yaml"));
    EXPECT_TRUE(std::filesystem::exists(dir / "DisplayParams.yaml"));
    EXPECT_TRUE(std::filesystem::exists(dir / "ImuParams.yaml"));
    EXPECT_TRUE(std::filesystem::exists(dir / "LeftCameraParams.yaml"));

    // PipelineParams must select MonoImu + plain VioBackend, otherwise
    // KimeraBackend's MonoImuPipeline + missing RightCameraParams.yaml
    // combination wouldn't load.
    const auto pipeline_yaml = readWhole(dir / "PipelineParams.yaml");
    EXPECT_NE(pipeline_yaml.find("frontend_type: 0"), std::string::npos);
    EXPECT_NE(pipeline_yaml.find("backend_type: 0"), std::string::npos);
    EXPECT_NE(pipeline_yaml.find("parallel_run: 1"), std::string::npos);

    const auto cam_yaml = readWhole(dir / "LeftCameraParams.yaml");
    EXPECT_NE(cam_yaml.find("%YAML:1.0"), std::string::npos);
    EXPECT_NE(cam_yaml.find("camera_id: \"downward\""), std::string::npos);
    EXPECT_NE(cam_yaml.find("rate_hz: 120"), std::string::npos);
    EXPECT_NE(cam_yaml.find("resolution: [640, 480]"), std::string::npos);
    EXPECT_NE(cam_yaml.find("camera_model: \"pinhole\""), std::string::npos);
    EXPECT_NE(cam_yaml.find("distortion_model: \"radial-tangential\""),
              std::string::npos);
    EXPECT_NE(cam_yaml.find("intrinsics: [500"), std::string::npos);
    EXPECT_NE(cam_yaml.find("T_BS:"), std::string::npos);

    const auto imu_yaml = readWhole(dir / "ImuParams.yaml");
    EXPECT_NE(imu_yaml.find("accelerometer_noise_density: 0.02"),
              std::string::npos);
    // Kimera's ImuParams parser CHECK_FATAL aborts unless this matrix
    // is identity (Kimera assumes IMU == body). The writer therefore
    // emits identity regardless of imu_extrinsic_body_to_imu — that
    // offset stays on FusionService's path. The test config above sets
    // a non-identity offset on purpose, which would surface as a
    // 0.05/0.10 substring if it leaked into the IMU YAML.
    EXPECT_NE(imu_yaml.find("T_BS:"), std::string::npos);
    EXPECT_EQ(imu_yaml.find("IMUtoBodyT_BS:"), std::string::npos);
    EXPECT_EQ(imu_yaml.find("0.050000000000000003"), std::string::npos);
    EXPECT_EQ(imu_yaml.find("0.10000000000000001"), std::string::npos);

    std::filesystem::remove_all(dir);
}

TEST(KimeraParamWriter, NoOpWhenVioDisabled) {
    const auto dir = tempDir("noop");
    auto cfg = makeVioReadyConfig();
    cfg.vio.enabled = false;

    posest::vio::emitKimeraParamYamls(cfg, dir);

    EXPECT_FALSE(std::filesystem::exists(dir / "FrontendParams.yaml"));
    EXPECT_FALSE(std::filesystem::exists(dir / "LeftCameraParams.yaml"));

    std::filesystem::remove_all(dir);
}

TEST(KimeraParamWriter, ThrowsOnMissingActiveCalibration) {
    const auto dir = tempDir("missing_calib");
    auto cfg = makeVioReadyConfig();
    cfg.calibrations[0].active = false;  // no active row left

    EXPECT_THROW(posest::vio::emitKimeraParamYamls(cfg, dir),
                 std::runtime_error);

    std::filesystem::remove_all(dir);
}

TEST(KimeraParamWriter, ThrowsOnMissingActiveCameraImuCalibration) {
    const auto dir = tempDir("missing_camimu");
    auto cfg = makeVioReadyConfig();
    cfg.camera_imu_calibrations[0].active = false;

    EXPECT_THROW(posest::vio::emitKimeraParamYamls(cfg, dir),
                 std::runtime_error);

    std::filesystem::remove_all(dir);
}

TEST(KimeraParamWriter, IdempotentOnRepeatCall) {
    const auto dir = tempDir("idempotent");
    const auto cfg = makeVioReadyConfig();

    posest::vio::emitKimeraParamYamls(cfg, dir);
    const auto first = readWhole(dir / "LeftCameraParams.yaml");
    posest::vio::emitKimeraParamYamls(cfg, dir);
    const auto second = readWhole(dir / "LeftCameraParams.yaml");

    EXPECT_EQ(first, second);
    // Atomic-rename leftover (.tmp) must not survive a successful write.
    EXPECT_FALSE(std::filesystem::exists(dir / "LeftCameraParams.yaml.tmp"));

    std::filesystem::remove_all(dir);
}

TEST(KimeraParamWriter, MapsKalibrEquidistantToKimeraEquidistant) {
    const auto dir = tempDir("equi");
    auto cfg = makeVioReadyConfig();
    cfg.calibrations[0].distortion_model = "equidistant";

    posest::vio::emitKimeraParamYamls(cfg, dir);
    const auto cam_yaml = readWhole(dir / "LeftCameraParams.yaml");
    EXPECT_NE(cam_yaml.find("distortion_model: \"equidistant\""),
              std::string::npos);

    std::filesystem::remove_all(dir);
}

TEST(KimeraParamWriter, RejectsUnknownDistortionModel) {
    const auto dir = tempDir("unknown_distortion");
    auto cfg = makeVioReadyConfig();
    cfg.calibrations[0].distortion_model = "definitely_not_a_real_model";

    EXPECT_THROW(posest::vio::emitKimeraParamYamls(cfg, dir),
                 std::runtime_error);

    std::filesystem::remove_all(dir);
}

TEST(KimeraParamWriter, ImuParamsCarriesRandomWalkFromFusionConfig) {
    const auto dir = tempDir("imu_random_walk");
    auto cfg = makeVioReadyConfig();
    // Distinct, non-default values so the assertion can't pass on
    // FusionConfig defaults.
    cfg.fusion.accel_bias_rw_sigma = 2.5e-4;
    cfg.fusion.gyro_bias_rw_sigma = 3.5e-5;

    posest::vio::emitKimeraParamYamls(cfg, dir);
    const auto imu_yaml = readWhole(dir / "ImuParams.yaml");

    // Build the expected substrings using the same setprecision(17)
    // formatting the writer applies, so the test is not coupled to
    // platform-specific double-to-string rounding decisions.
    const auto fmt = [](double v) {
        std::ostringstream s;
        s << std::setprecision(17) << v;
        return s.str();
    };
    EXPECT_NE(imu_yaml.find("accelerometer_random_walk: " + fmt(2.5e-4)),
              std::string::npos);
    EXPECT_NE(imu_yaml.find("gyroscope_random_walk: " + fmt(3.5e-5)),
              std::string::npos);
    // Both lines must precede the T_BS matrix block — Kimera reads by
    // key so order is irrelevant for parsing, but the writer's
    // contract is "scalars first, matrix block last".
    const auto rw_pos = imu_yaml.find("accelerometer_random_walk:");
    const auto extrinsic_pos = imu_yaml.rfind("T_BS:");
    ASSERT_NE(rw_pos, std::string::npos);
    ASSERT_NE(extrinsic_pos, std::string::npos);
    EXPECT_LT(rw_pos, extrinsic_pos);

    std::filesystem::remove_all(dir);
}

TEST(KimeraParamWriter, ImuParamsDoesNotLeakPersistedBiasMean) {
    // FusionConfig::persisted_bias drives FusionService's own ImuFactor
    // chain but cannot be threaded through Kimera's YAML — Kimera's
    // ImuParams parser accepts no bias-mean key, and the only runtime
    // pathway (VioBackend::initStateAndSetPriors) is fed by
    // InitializationFromImu, not by us. This test pins the current
    // behaviour: if a future commit starts emitting bias-mean keys, the
    // assertion will catch it (forcing a re-evaluation rather than a
    // silent no-op when Kimera ignores the unknown keys). See the
    // explanatory comment in KimeraParamWriter::buildImuParamsYaml.
    const auto dir = tempDir("no_bias_leak");
    auto cfg = makeVioReadyConfig();
    cfg.fusion.persisted_bias = {0.123, 0.456, 0.789, 0.012, 0.034, 0.056};

    posest::vio::emitKimeraParamYamls(cfg, dir);
    const auto imu_yaml = readWhole(dir / "ImuParams.yaml");

    EXPECT_EQ(imu_yaml.find("accelerometer_bias"), std::string::npos);
    EXPECT_EQ(imu_yaml.find("gyroscope_bias"), std::string::npos);
    EXPECT_EQ(imu_yaml.find("imu_bias_init_a"), std::string::npos);
    EXPECT_EQ(imu_yaml.find("imu_bias_init_g"), std::string::npos);
    // None of the persisted_bias values should appear verbatim either —
    // the values above were chosen to be distinctive and not equal to
    // any other field's default.
    EXPECT_EQ(imu_yaml.find("0.123"), std::string::npos);
    EXPECT_EQ(imu_yaml.find("0.456"), std::string::npos);
    EXPECT_EQ(imu_yaml.find("0.789"), std::string::npos);

    std::filesystem::remove_all(dir);
}

TEST(KimeraParamWriter, CreatesParamDirIfMissing) {
    const auto parent = tempDir("create_parent");
    const auto child = parent / "nested" / "kimera";
    EXPECT_FALSE(std::filesystem::exists(child));

    posest::vio::emitKimeraParamYamls(makeVioReadyConfig(), child);

    EXPECT_TRUE(std::filesystem::exists(child / "LeftCameraParams.yaml"));

    std::filesystem::remove_all(parent);
}
