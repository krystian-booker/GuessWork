#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>

#include "server/camera_repository.hpp"
#include "server/database.hpp"

namespace gw::server {

namespace {

class CameraCalibrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        path_ = std::filesystem::temp_directory_path() /
                ("gw_calib_test_" + std::to_string(::getpid()) + "_" +
                 ::testing::UnitTest::GetInstance()->current_test_info()->name() + ".db");
        cleanup();
        db_   = std::make_unique<Database>(path_);
        repo_ = std::make_unique<CameraRepository>(*db_);
    }
    void TearDown() override {
        repo_.reset();
        db_.reset();
        cleanup();
    }
    void cleanup() {
        std::error_code ec;
        std::filesystem::remove(path_, ec);
        std::filesystem::remove(path_.string() + "-wal", ec);
        std::filesystem::remove(path_.string() + "-shm", ec);
    }

    std::filesystem::path             path_;
    std::unique_ptr<Database>         db_;
    std::unique_ptr<CameraRepository> repo_;
};

constexpr const char* kStubCalibrationJson =
    R"({"value0":{"intrinsics":[{"camera_type":"ds","intrinsics":{"fx":500.0,"fy":500.0,"cx":320.0,"cy":240.0,"xi":-0.2,"alpha":0.6}}],"resolution":[[640,480]]}})";

}  // namespace

TEST_F(CameraCalibrationTest, FreshRowHasNoCalibration) {
    const auto c = repo_->create("front", "SN001", 6.0);
    EXPECT_FALSE(c.calibration_json.has_value());
    EXPECT_FALSE(c.calibrated_at.has_value());
}

TEST_F(CameraCalibrationTest, SetCalibrationStoresJsonAndTimestamp) {
    const auto c = repo_->create("front", "SN001", 6.0);
    const auto updated = repo_->set_calibration(c.id, kStubCalibrationJson);
    ASSERT_TRUE(updated.has_value());
    ASSERT_TRUE(updated->calibration_json.has_value());
    EXPECT_EQ(*updated->calibration_json, kStubCalibrationJson);
    ASSERT_TRUE(updated->calibrated_at.has_value());
    EXPECT_GT(*updated->calibrated_at, 0);
}

TEST_F(CameraCalibrationTest, SetCalibrationPersistsAcrossGet) {
    const auto c = repo_->create("front", "SN001", 6.0);
    repo_->set_calibration(c.id, kStubCalibrationJson);
    const auto fetched = repo_->get(c.id);
    ASSERT_TRUE(fetched.has_value());
    ASSERT_TRUE(fetched->calibration_json.has_value());
    EXPECT_EQ(*fetched->calibration_json, kStubCalibrationJson);
}

TEST_F(CameraCalibrationTest, SetCalibrationOnMissingIdReturnsNullopt) {
    EXPECT_FALSE(repo_->set_calibration(9999, kStubCalibrationJson).has_value());
}

TEST_F(CameraCalibrationTest, SetCalibrationOverwrites) {
    const auto c = repo_->create("front", "SN001", 6.0);
    repo_->set_calibration(c.id, R"({"value0":{"intrinsics":[{"camera_type":6.0}]}})");
    const auto updated = repo_->set_calibration(c.id, kStubCalibrationJson);
    ASSERT_TRUE(updated.has_value());
    ASSERT_TRUE(updated->calibration_json.has_value());
    EXPECT_EQ(*updated->calibration_json, kStubCalibrationJson);
}

TEST_F(CameraCalibrationTest, ClearCalibrationResetsBothColumns) {
    const auto c = repo_->create("front", "SN001", 6.0);
    repo_->set_calibration(c.id, kStubCalibrationJson);

    EXPECT_TRUE(repo_->clear_calibration(c.id));

    const auto fetched = repo_->get(c.id);
    ASSERT_TRUE(fetched.has_value());
    EXPECT_FALSE(fetched->calibration_json.has_value());
    EXPECT_FALSE(fetched->calibrated_at.has_value());
}

TEST_F(CameraCalibrationTest, ClearCalibrationOnMissingIdReturnsFalse) {
    EXPECT_FALSE(repo_->clear_calibration(9999));
}

TEST_F(CameraCalibrationTest, ListAllReturnsCalibrationFields) {
    const auto a = repo_->create("a", "SN001", 6.0);
    repo_->create("b", "SN002", 6.0);
    repo_->set_calibration(a.id, kStubCalibrationJson);

    const auto rows = repo_->list_all();
    ASSERT_EQ(rows.size(), 2u);
    EXPECT_TRUE(rows[0].calibration_json.has_value());
    EXPECT_FALSE(rows[1].calibration_json.has_value());
}

// --- IMU extrinsics (camera-IMU calibration result) ---

TEST_F(CameraCalibrationTest, SetImuExtrinsicsStoresYamlAndTimestamp) {
    const auto c = repo_->create("front", "SN001", 6.0);
    EXPECT_FALSE(c.imu_extrinsics_json.has_value());
    EXPECT_FALSE(c.extrinsics_calibrated_at.has_value());

    const auto updated = repo_->set_imu_extrinsics(c.id, "cam0:\n  T_cam_imu: []\n");
    ASSERT_TRUE(updated.has_value());
    ASSERT_TRUE(updated->imu_extrinsics_json.has_value());
    EXPECT_NE(updated->imu_extrinsics_json->find("T_cam_imu"), std::string::npos);
    ASSERT_TRUE(updated->extrinsics_calibrated_at.has_value());
    EXPECT_GT(*updated->extrinsics_calibrated_at, 0);
    // Intrinsics columns are independent — untouched.
    EXPECT_FALSE(updated->calibration_json.has_value());
    EXPECT_FALSE(updated->calibrated_at.has_value());
}

TEST_F(CameraCalibrationTest, ClearImuExtrinsicsResetsBothColumns) {
    const auto c = repo_->create("front", "SN001", 6.0);
    repo_->set_calibration(c.id, kStubCalibrationJson);
    repo_->set_imu_extrinsics(c.id, "cam0: {}\n");

    EXPECT_TRUE(repo_->clear_imu_extrinsics(c.id));

    const auto fetched = repo_->get(c.id);
    ASSERT_TRUE(fetched.has_value());
    EXPECT_FALSE(fetched->imu_extrinsics_json.has_value());
    EXPECT_FALSE(fetched->extrinsics_calibrated_at.has_value());
    // Intrinsics survive an extrinsics clear.
    EXPECT_TRUE(fetched->calibration_json.has_value());
}

TEST_F(CameraCalibrationTest, SetImuExtrinsicsOnMissingIdReturnsNullopt) {
    EXPECT_FALSE(repo_->set_imu_extrinsics(9999, "cam0: {}\n").has_value());
    EXPECT_FALSE(repo_->clear_imu_extrinsics(9999));
}

// --- role column ---

TEST_F(CameraCalibrationTest, RoleRoundTripsThroughUpdate) {
    const auto c = repo_->create("front", "SN001", 6.0);
    EXPECT_FALSE(c.role.has_value());

    CameraUpdate set_role;
    set_role.role = std::optional<std::string>("vio_left");
    const auto updated = repo_->update(c.id, set_role);
    ASSERT_TRUE(updated.has_value());
    ASSERT_TRUE(updated->role.has_value());
    EXPECT_EQ(*updated->role, "vio_left");

    CameraUpdate clear_role;
    clear_role.role = std::optional<std::string>(std::nullopt);
    const auto cleared = repo_->update(c.id, clear_role);
    ASSERT_TRUE(cleared.has_value());
    EXPECT_FALSE(cleared->role.has_value());
}

TEST_F(CameraCalibrationTest, BogusRoleRejectedByCheckConstraint) {
    const auto c = repo_->create("front", "SN001", 6.0);
    CameraUpdate bad;
    bad.role = std::optional<std::string>("steering_wheel");
    EXPECT_THROW(repo_->update(c.id, bad), std::exception);
}

}  // namespace gw::server
