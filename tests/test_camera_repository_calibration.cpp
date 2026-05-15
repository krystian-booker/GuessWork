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
    const auto c = repo_->create("front", "SN001", "pinhole");
    EXPECT_FALSE(c.calibration_json.has_value());
    EXPECT_FALSE(c.calibrated_at.has_value());
}

TEST_F(CameraCalibrationTest, SetCalibrationStoresJsonAndTimestamp) {
    const auto c = repo_->create("front", "SN001", "pinhole");
    const auto updated = repo_->set_calibration(c.id, kStubCalibrationJson);
    ASSERT_TRUE(updated.has_value());
    ASSERT_TRUE(updated->calibration_json.has_value());
    EXPECT_EQ(*updated->calibration_json, kStubCalibrationJson);
    ASSERT_TRUE(updated->calibrated_at.has_value());
    EXPECT_GT(*updated->calibrated_at, 0);
}

TEST_F(CameraCalibrationTest, SetCalibrationPersistsAcrossGet) {
    const auto c = repo_->create("front", "SN001", "pinhole");
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
    const auto c = repo_->create("front", "SN001", "pinhole");
    repo_->set_calibration(c.id, R"({"value0":{"intrinsics":[{"camera_type":"pinhole"}]}})");
    const auto updated = repo_->set_calibration(c.id, kStubCalibrationJson);
    ASSERT_TRUE(updated.has_value());
    ASSERT_TRUE(updated->calibration_json.has_value());
    EXPECT_EQ(*updated->calibration_json, kStubCalibrationJson);
}

TEST_F(CameraCalibrationTest, ClearCalibrationResetsBothColumns) {
    const auto c = repo_->create("front", "SN001", "pinhole");
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
    const auto a = repo_->create("a", "SN001", "pinhole");
    repo_->create("b", "SN002", "pinhole");
    repo_->set_calibration(a.id, kStubCalibrationJson);

    const auto rows = repo_->list_all();
    ASSERT_EQ(rows.size(), 2u);
    EXPECT_TRUE(rows[0].calibration_json.has_value());
    EXPECT_FALSE(rows[1].calibration_json.has_value());
}

}  // namespace gw::server
