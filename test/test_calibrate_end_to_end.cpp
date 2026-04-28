#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "posest/IFrameProducer.h"
#include "posest/MockProducer.h"
#include "posest/config/SqliteConfigStore.h"
#include "posest/runtime/Daemon.h"
#include "posest/runtime/Factories.h"

namespace {

std::filesystem::path tempDbPath(const std::string& name) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() /
           ("posest_e2e_" + name + "_" + std::to_string(stamp) + ".db");
}

std::filesystem::path tempOutputDir(const std::string& name) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() /
           ("posest_e2e_" + name + "_" + std::to_string(stamp));
}

std::filesystem::path fixturePath(const std::string& name) {
    return std::filesystem::path(POSEST_TEST_DATA_DIR) / "kalibr" / name;
}

void copyFixture(const std::string& fixture_name,
                 const std::filesystem::path& destination) {
    std::filesystem::create_directories(destination.parent_path());
    std::filesystem::copy_file(
        fixturePath(fixture_name), destination,
        std::filesystem::copy_options::overwrite_existing);
}

// Stub camera factory that hands back MockProducer instances. The
// orchestrator's recording step starts each producer for `duration_s`
// and the recorder collects whatever frames arrive. With
// --require-imu=no, min_trigger_match_fraction=0.0 so unmatched frames
// don't fail throwIfUnacceptable.
class FakeCameraFactory final : public posest::runtime::ICameraBackendFactory {
public:
    std::shared_ptr<posest::IFrameProducer> createCamera(
        const posest::CameraConfig& config) override {
        posest::mock::MockProducerConfig cfg;
        cfg.id = config.id;
        cfg.target_fps = 60.0;
        return std::make_shared<posest::mock::MockProducer>(cfg);
    }
};

posest::runtime::RuntimeConfig makeBaseConfig() {
    posest::runtime::RuntimeConfig config;
    for (int i = 0; i < 2; ++i) {
        posest::CameraConfig camera;
        camera.id = "cam" + std::to_string(i);
        camera.type = "v4l2";
        camera.device = "/dev/video" + std::to_string(i);
        camera.enabled = true;
        camera.format.width = 320;
        camera.format.height = 240;
        camera.format.fps = 60.0;
        camera.format.pixel_format = "mjpeg";
        config.cameras.push_back(camera);
    }
    posest::runtime::CalibrationTargetConfig target;
    target.id = "apgrid_e2e";
    target.type = "aprilgrid";
    target.rows = 6;
    target.cols = 6;
    target.tag_size_m = 0.088;
    target.tag_spacing_ratio = 0.3;
    target.tag_family = "tag36h11";
    config.calibration_targets.push_back(target);
    return config;
}

posest::runtime::DaemonOptions makeOrchestratorOptions(
    const std::filesystem::path& output_dir,
    posest::runtime::CalibrationMode mode,
    bool cleanup_dataset) {
    posest::runtime::DaemonOptions options;
    options.command =
        posest::runtime::DaemonCommand::CalibrateCameraEndToEnd;
    auto& cmd = options.calibrate_camera_end_to_end;
    cmd.camera_ids = {"cam0", "cam1"};
    cmd.topics = {"/posest/cam0/image_raw", "/posest/cam1/image_raw"};
    cmd.camera_to_robots = {
        {{0.10, 0.0, 0.20}, {0.0, 0.0, 0.0}},
        {{-0.10, 0.0, 0.20}, {0.0, 0.0, 0.0}},
    };
    cmd.target_id = "apgrid_e2e";
    cmd.output_dir = output_dir;
    cmd.version = "v1";
    cmd.duration_s = 0.05;  // ~3 frames at 60fps; enough for frames_seen >= 1
    cmd.require_imu = posest::runtime::ImuRequirement::No;
    cmd.mode = mode;
    cmd.imu_path = mode == posest::runtime::CalibrationMode::IntrinsicAndImu
                       ? std::filesystem::path("/tmp/imu.yaml")
                       : std::filesystem::path{};
    cmd.cleanup_dataset = cleanup_dataset;
    return options;
}

// RAII guard that swaps the system impl on construction and restores on
// destruction so a thrown test assertion doesn't leak a captured lambda
// into the next test.
class SystemImplGuard final {
public:
    explicit SystemImplGuard(posest::runtime::SystemImpl impl)
        : previous_(posest::runtime::setSystemImplForTesting(std::move(impl))) {}
    ~SystemImplGuard() {
        posest::runtime::setSystemImplForTesting(previous_);
    }
    SystemImplGuard(const SystemImplGuard&) = delete;
    SystemImplGuard& operator=(const SystemImplGuard&) = delete;

private:
    posest::runtime::SystemImpl previous_;
};

posest::runtime::SystemImpl makeStagingImpl(
    const std::filesystem::path& output_dir,
    const std::string& camchain_fixture,
    const std::string& cam_results_fixture,
    const std::string& imucam_results_fixture) {
    return [output_dir, camchain_fixture, cam_results_fixture,
            imucam_results_fixture](const char* cmd) -> int {
        const std::string s(cmd);
        if (s.find("make_rosbag.py") != std::string::npos) {
            std::filesystem::create_directories(output_dir);
            std::ofstream(output_dir / "kalibr.bag") << "stub";
        } else if (s.find("kalibr_calibrate_imu_camera") !=
                   std::string::npos) {
            copyFixture("camchain-imucam_two_cameras.yaml",
                        output_dir / "camchain-imucam.yaml");
            copyFixture(imucam_results_fixture,
                        output_dir / "results-imucam.txt");
        } else if (s.find("kalibr_calibrate_cameras") != std::string::npos) {
            copyFixture(camchain_fixture, output_dir / "camchain.yaml");
            copyFixture(cam_results_fixture,
                        output_dir / "results-cam.txt");
        }
        return 0;
    };
}

}  // namespace

TEST(CalibrateEndToEnd, IntrinsicSuccessPersistsCalibrationsAndBaseline) {
    const auto db_path = tempDbPath("intrinsic_success");
    const auto output_dir = tempOutputDir("intrinsic_success");
    std::filesystem::remove(db_path);
    {
        posest::config::SqliteConfigStore store(db_path);
        store.saveRuntimeConfig(makeBaseConfig());
    }

    SystemImplGuard guard(makeStagingImpl(
        output_dir, "camchain_two_cameras.yaml",
        "results-cam_two_cameras.txt", ""));
    posest::config::SqliteConfigStore store(db_path);
    FakeCameraFactory factory;

    const auto options = makeOrchestratorOptions(
        output_dir, posest::runtime::CalibrationMode::Intrinsic, false);
    posest::runtime::runConfigCommand(options, store, factory);

    const auto loaded = store.loadRuntimeConfig();
    ASSERT_EQ(loaded.calibrations.size(), 2u);
    EXPECT_GT(loaded.calibrations[0].reprojection_rms_px, 0.0);
    EXPECT_GT(loaded.calibrations[1].reprojection_rms_px, 0.0);
    EXPECT_EQ(loaded.camera_extrinsics.size(), 2u);
    ASSERT_EQ(loaded.camera_to_camera_extrinsics.size(), 1u);
    EXPECT_EQ(loaded.camera_to_camera_extrinsics[0].reference_camera_id, "cam0");
    EXPECT_EQ(loaded.camera_to_camera_extrinsics[0].target_camera_id, "cam1");
    ASSERT_EQ(loaded.kalibr_datasets.size(), 1u);

    std::filesystem::remove_all(output_dir);
    std::filesystem::remove(db_path);
}

TEST(CalibrateEndToEnd, FailedGateRollsBackNoCalibrations) {
    const auto db_path = tempDbPath("intrinsic_fail");
    const auto output_dir = tempOutputDir("intrinsic_fail");
    std::filesystem::remove(db_path);
    {
        posest::config::SqliteConfigStore store(db_path);
        store.saveRuntimeConfig(makeBaseConfig());
    }

    SystemImplGuard guard(makeStagingImpl(
        output_dir, "camchain_single_camera.yaml",
        "results-cam_fail.txt", ""));
    posest::config::SqliteConfigStore store(db_path);
    FakeCameraFactory factory;

    auto options = makeOrchestratorOptions(
        output_dir, posest::runtime::CalibrationMode::Intrinsic, false);
    // The fixtures we stage describe only cam0; trim the orchestrator's
    // request to match (camchain_single_camera.yaml's rostopic).
    options.calibrate_camera_end_to_end.camera_ids = {"cam0"};
    options.calibrate_camera_end_to_end.topics = {"/posest/cam0/image_raw"};
    options.calibrate_camera_end_to_end.camera_to_robots = {
        {{0.10, 0.0, 0.20}, {0.0, 0.0, 0.0}},
    };

    EXPECT_THROW(
        posest::runtime::runConfigCommand(options, store, factory),
        std::runtime_error);

    const auto loaded = store.loadRuntimeConfig();
    EXPECT_TRUE(loaded.calibrations.empty());
    EXPECT_TRUE(loaded.camera_extrinsics.empty());
    // The kalibr_datasets row IS expected — recording succeeded; the gate
    // failed afterwards. Operator can re-run after diagnosing.
    EXPECT_EQ(loaded.kalibr_datasets.size(), 1u);

    std::filesystem::remove_all(output_dir);
    std::filesystem::remove(db_path);
}

TEST(CalibrateEndToEnd, IntrinsicAndImuPersistsBoth) {
    const auto db_path = tempDbPath("intrinsic_imu_pass");
    const auto output_dir = tempOutputDir("intrinsic_imu_pass");
    std::filesystem::remove(db_path);
    {
        posest::config::SqliteConfigStore store(db_path);
        store.saveRuntimeConfig(makeBaseConfig());
    }

    SystemImplGuard guard(makeStagingImpl(
        output_dir, "camchain_two_cameras.yaml",
        "results-cam_two_cameras.txt",
        "results-imucam_two_cameras_pass.txt"));
    posest::config::SqliteConfigStore store(db_path);
    FakeCameraFactory factory;

    const auto options = makeOrchestratorOptions(
        output_dir,
        posest::runtime::CalibrationMode::IntrinsicAndImu,
        false);
    posest::runtime::runConfigCommand(options, store, factory);

    const auto loaded = store.loadRuntimeConfig();
    EXPECT_EQ(loaded.calibrations.size(), 2u);
    EXPECT_EQ(loaded.camera_imu_calibrations.size(), 2u);
    EXPECT_GT(loaded.camera_imu_calibrations[0].reprojection_rms_px, 0.0);
    EXPECT_GT(loaded.camera_imu_calibrations[0].gyro_rms_radps, 0.0);
    EXPECT_GT(loaded.camera_imu_calibrations[0].accel_rms_mps2, 0.0);

    std::filesystem::remove_all(output_dir);
    std::filesystem::remove(db_path);
}

TEST(CalibrateEndToEnd, IntrinsicSucceedsImuFailsKeepsIntrinsics) {
    const auto db_path = tempDbPath("imu_fail_keep_intrinsics");
    const auto output_dir = tempOutputDir("imu_fail_keep_intrinsics");
    std::filesystem::remove(db_path);
    {
        posest::config::SqliteConfigStore store(db_path);
        store.saveRuntimeConfig(makeBaseConfig());
    }

    SystemImplGuard guard(makeStagingImpl(
        output_dir, "camchain_two_cameras.yaml",
        "results-cam_two_cameras.txt",
        "results-imucam_two_cameras_fail.txt"));
    posest::config::SqliteConfigStore store(db_path);
    FakeCameraFactory factory;

    const auto options = makeOrchestratorOptions(
        output_dir,
        posest::runtime::CalibrationMode::IntrinsicAndImu,
        false);
    EXPECT_THROW(
        posest::runtime::runConfigCommand(options, store, factory),
        std::runtime_error);

    const auto loaded = store.loadRuntimeConfig();
    EXPECT_EQ(loaded.calibrations.size(), 2u);  // intrinsics survived
    EXPECT_TRUE(loaded.camera_imu_calibrations.empty());

    std::filesystem::remove_all(output_dir);
    std::filesystem::remove(db_path);
}

TEST(CalibrateEndToEnd, CleanupDatasetRemovesRowAndDirectory) {
    const auto db_path = tempDbPath("cleanup");
    const auto output_dir = tempOutputDir("cleanup");
    std::filesystem::remove(db_path);
    {
        posest::config::SqliteConfigStore store(db_path);
        store.saveRuntimeConfig(makeBaseConfig());
    }

    SystemImplGuard guard(makeStagingImpl(
        output_dir, "camchain_two_cameras.yaml",
        "results-cam_two_cameras.txt", ""));
    posest::config::SqliteConfigStore store(db_path);
    FakeCameraFactory factory;

    const auto options = makeOrchestratorOptions(
        output_dir, posest::runtime::CalibrationMode::Intrinsic, true);
    posest::runtime::runConfigCommand(options, store, factory);

    const auto loaded = store.loadRuntimeConfig();
    EXPECT_TRUE(loaded.kalibr_datasets.empty());
    EXPECT_FALSE(std::filesystem::exists(output_dir));
    // Calibrations + extrinsics survive the cleanup; only the dataset
    // recording artifacts go away.
    EXPECT_EQ(loaded.calibrations.size(), 2u);

    std::filesystem::remove(db_path);
}

TEST(CalibrateEndToEnd, KeepsDatasetByDefault) {
    const auto db_path = tempDbPath("keep_default");
    const auto output_dir = tempOutputDir("keep_default");
    std::filesystem::remove(db_path);
    {
        posest::config::SqliteConfigStore store(db_path);
        store.saveRuntimeConfig(makeBaseConfig());
    }

    SystemImplGuard guard(makeStagingImpl(
        output_dir, "camchain_two_cameras.yaml",
        "results-cam_two_cameras.txt", ""));
    posest::config::SqliteConfigStore store(db_path);
    FakeCameraFactory factory;

    const auto options = makeOrchestratorOptions(
        output_dir, posest::runtime::CalibrationMode::Intrinsic, false);
    posest::runtime::runConfigCommand(options, store, factory);

    const auto loaded = store.loadRuntimeConfig();
    EXPECT_EQ(loaded.kalibr_datasets.size(), 1u);
    EXPECT_TRUE(std::filesystem::is_directory(output_dir));

    std::filesystem::remove_all(output_dir);
    std::filesystem::remove(db_path);
}
