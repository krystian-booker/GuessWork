#include <gtest/gtest.h>

#include <filesystem>

#include "server/apriltag_supervisor.hpp"
#include "server/camera_repository.hpp"
#include "server/database.hpp"
#include "server/field_layout_repository.hpp"
#include "server/imu_config_repository.hpp"
#include "server/routes_apriltag.hpp"  // seed_default_field_layout

// Factory source-selection + status-composition tests. No frames flow here
// (consumers are created against no channel and immediately discarded) —
// what's under test is which calibration source the supervisor picks, why
// it declines, and how status() reports the shared gates.

namespace gw::server {

namespace {

// Same shapes as tests/test_calibration_store.cpp — a stored intrinsics-only
// camchain and a camchain-imucam block (single camera, re-keyed to cam0).
constexpr const char* kIntrinsicsOnly = R"(cam0:
  camera_model: pinhole
  distortion_coeffs: [-0.286, 0.073, 0.0002, -0.00003]
  distortion_model: radtan
  intrinsics: [1465.3, 1466.1, 1023.5, 767.2]
  resolution: [2048, 1536]
  rostopic: /cam0/image_raw
guesswork_meta:
  reprojection_error_px: 0.214301
)";

constexpr const char* kImucam = R"(cam0:
  T_cam_imu:
  - [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975]
  - [0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768]
  - [-0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949]
  - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
  distortion_model: radtan
  intrinsics: [458.654, 457.296, 367.215, 248.375]
  resolution: [752, 480]
  rostopic: /cam0/image_raw
  timeshift_cam_imu: 5.6375620963e-05
guesswork_meta:
  reprojection_error_px: 0.31
)";

constexpr const char* kTRobotImu =
    R"({"T_robot_imu": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]})";

class ApriltagSupervisorTest : public ::testing::Test {
protected:
    void SetUp() override {
        dir_ = std::filesystem::temp_directory_path() /
               ("gw_apriltag_sup_test_" + std::to_string(::getpid()));
        std::filesystem::create_directories(dir_);
        db_      = std::make_unique<Database>(dir_ / "test.db");
        cameras_ = std::make_unique<CameraRepository>(*db_);
        layouts_ = std::make_unique<FieldLayoutRepository>(*db_);
        imu_cfg_ = std::make_unique<ImuConfigRepository>(*db_);
        seed_default_field_layout(*layouts_);
        sup_ = std::make_unique<ApriltagSupervisor>(*cameras_, *layouts_,
                                                    *imu_cfg_);
    }
    void TearDown() override {
        sup_.reset();
        cameras_.reset();
        layouts_.reset();
        imu_cfg_.reset();
        db_.reset();
        std::filesystem::remove_all(dir_);
    }

    Camera add_apriltag_camera(const std::string& name,
                               const std::string& serial) {
        const auto c = cameras_->create(name, serial, 6.0, std::nullopt,
                                        false, std::nullopt);
        CameraUpdate upd;
        upd.role = std::optional<std::string>("apriltag");
        return *cameras_->update(c.id, upd);
    }

    std::filesystem::path                 dir_;
    std::unique_ptr<Database>             db_;
    std::unique_ptr<CameraRepository>     cameras_;
    std::unique_ptr<FieldLayoutRepository> layouts_;
    std::unique_ptr<ImuConfigRepository>  imu_cfg_;
    std::unique_ptr<ApriltagSupervisor>   sup_;
};

}  // namespace

TEST_F(ApriltagSupervisorTest, DeclinesWrongRoleAndUncalibrated) {
    // Non-apriltag role: not this factory's camera.
    const auto plain = cameras_->create("plain", "S-0", 6.0, std::nullopt,
                                        false, std::nullopt);
    EXPECT_EQ(sup_->make_consumer(plain), nullptr);

    // Right role, no stored calibration: declined with a recorded reason.
    const auto cam = add_apriltag_camera("uncal", "S-1");
    EXPECT_EQ(sup_->make_consumer(cam), nullptr);

    const auto st = sup_->status();
    ASSERT_EQ(st.cameras.size(), 1u);  // only role='apriltag' rows listed
    EXPECT_EQ(st.cameras[0].camera_id, cam.id);
    EXPECT_FALSE(st.cameras[0].running);
    EXPECT_FALSE(st.cameras[0].reason.empty());
}

TEST_F(ApriltagSupervisorTest, IntrinsicsOnlyBuildsDetectionConsumer) {
    auto cam = add_apriltag_camera("intr", "S-2");
    cam      = *cameras_->set_calibration(cam.id, kIntrinsicsOnly);

    const auto consumer = sup_->make_consumer(cam);
    EXPECT_NE(consumer, nullptr);  // detection runs; publish gated downstream
}

TEST_F(ApriltagSupervisorTest, ImuExtrinsicsPreferred) {
    auto cam = add_apriltag_camera("full", "S-3");
    cam      = *cameras_->set_calibration(cam.id, kIntrinsicsOnly);
    cam      = *cameras_->set_imu_extrinsics(cam.id, kImucam);

    const auto consumer = sup_->make_consumer(cam);
    EXPECT_NE(consumer, nullptr);
}

TEST_F(ApriltagSupervisorTest, UnparseableCalibrationDeclinesSafely) {
    auto cam = add_apriltag_camera("garbled", "S-4");
    cam      = *cameras_->set_calibration(cam.id, "not: [valid: camchain");
    // A bad calibration row must not throw out of the factory (the slot
    // start path treats exceptions as fatal for the camera).
    std::shared_ptr<gw::IConsumer> consumer;
    EXPECT_NO_THROW(consumer = sup_->make_consumer(cam));
    EXPECT_EQ(consumer, nullptr);
}

TEST_F(ApriltagSupervisorTest, StatusReportsSharedGates) {
    auto st = sup_->status();
    EXPECT_TRUE(st.active_layout_id.has_value());  // seeded season layout
    EXPECT_FALSE(st.t_robot_imu_set);

    ImuConfigUpdate upd;
    upd.t_imu_robot_json = std::optional<std::string>(kTRobotImu);
    imu_cfg_->update(upd);
    sup_->reload_shared();

    st = sup_->status();
    EXPECT_TRUE(st.t_robot_imu_set);
}

}  // namespace gw::server
