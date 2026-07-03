#include <gtest/gtest.h>

#include <filesystem>

#include "server/calibration_supervisor.hpp"
#include "server/camera_repository.hpp"
#include "server/camera_supervisor.hpp"
#include "server/database.hpp"
#include "server/imu_config_repository.hpp"
#include "server/teensy_manager.hpp"

// Validation-gauntlet tests. The CameraSupervisor is never start()ed (in
// GW_STUB_SPINNAKER builds it couldn't produce a camera anyway) and the
// TeensyManager is never start()ed — every camera is offline and the Teensy
// is disconnected, which is exactly the state the pre-flight gates exist to
// reject. The happy path (attach → record → Kalibr) is hardware/e2e turf.

namespace gw::server {

namespace {

class CalibrationSupervisorTest : public ::testing::Test {
protected:
    void SetUp() override {
        dir_ = std::filesystem::temp_directory_path() /
               ("gw_calib_sup_test_" + std::to_string(::getpid()));
        std::filesystem::create_directories(dir_);
        db_      = std::make_unique<Database>(dir_ / "test.db");
        cameras_ = std::make_unique<CameraRepository>(*db_);
        imu_cfg_ = std::make_unique<ImuConfigRepository>(*db_);
        sup_     = std::make_unique<CameraSupervisor>(
            *cameras_, StreamParams{1280, 720, 30, 4'000'000}, nullptr);
        calib_ = std::make_unique<CalibrationSupervisor>(
            *sup_, *cameras_, teensy_, *imu_cfg_, dir_ / "sessions");
    }
    void TearDown() override {
        calib_.reset();
        sup_.reset();
        cameras_.reset();
        imu_cfg_.reset();
        db_.reset();
        std::filesystem::remove_all(dir_);
    }

    int64_t add_camera(const std::string& name, const std::string& serial,
                       double focal_mm, bool hw_sync = false) {
        const auto c = cameras_->create(
            name, serial, focal_mm, std::nullopt, hw_sync,
            hw_sync ? std::optional<int64_t>(1) : std::nullopt);
        return c.id;
    }

    std::filesystem::path                  dir_;
    std::unique_ptr<Database>              db_;
    std::unique_ptr<CameraRepository>      cameras_;
    std::unique_ptr<ImuConfigRepository>   imu_cfg_;
    TeensyManager                          teensy_;  // never start()ed
    std::unique_ptr<CameraSupervisor>      sup_;     // never start()ed
    std::unique_ptr<CalibrationSupervisor> calib_;
};

void expect_calibration_error(const std::function<void()>& fn,
                              const std::string&           needle) {
    try {
        fn();
        FAIL() << "expected CalibrationError containing '" << needle << "'";
    } catch (const CalibrationError& e) {
        EXPECT_NE(std::string(e.what()).find(needle), std::string::npos)
            << "actual: " << e.what();
    }
}

}  // namespace

TEST_F(CalibrationSupervisorTest, StartRejectsUnknownCamera) {
    expect_calibration_error([&] { calib_->start(9999); }, "not found");
}

TEST_F(CalibrationSupervisorTest, StartRejectsZeroFocalLength) {
    // Regression for the Kalibr focal_px=0 / hfov=180° foot-gun: the guard
    // must fire BEFORE the offline check so the operator fixes the config
    // first, not after plugging in hardware.
    const auto id = add_camera("cam-nofocal", "SER-1", 0.0);
    expect_calibration_error([&] { calib_->start(id); }, "focal length");
}

TEST_F(CalibrationSupervisorTest, StartRejectsOfflineCamera) {
    const auto id = add_camera("cam-ok", "SER-2", 6.0);
    expect_calibration_error([&] { calib_->start(id); }, "offline");
}

TEST_F(CalibrationSupervisorTest, ExtrinsicsRejectsBadCameraCount) {
    expect_calibration_error([&] { calib_->start_extrinsics({}); },
                             "1 or 2 cameras");
    expect_calibration_error([&] { calib_->start_extrinsics({1, 2, 3}); },
                             "1 or 2 cameras");
}

TEST_F(CalibrationSupervisorTest, ExtrinsicsRejectsDuplicateIds) {
    expect_calibration_error([&] { calib_->start_extrinsics({7, 7}); },
                             "duplicate");
}

TEST_F(CalibrationSupervisorTest, ExtrinsicsRejectsWithoutTeensy) {
    // The Teensy gates run before any camera validation — extrinsics bags
    // are meaningless without the shared clock.
    const auto id = add_camera("cam-sync", "SER-3", 6.0, /*hw_sync=*/true);
    expect_calibration_error([&] { calib_->start_extrinsics({id}); },
                             "Teensy not connected");
}

TEST_F(CalibrationSupervisorTest, StatusEmptyWhenNoSessions) {
    EXPECT_FALSE(calib_->status(1).has_value());
    EXPECT_FALSE(calib_->extrinsics_status().has_value());
}

}  // namespace gw::server
