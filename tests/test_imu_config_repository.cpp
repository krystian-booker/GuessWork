#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <string>

#include "server/database.hpp"
#include "server/imu_config_repository.hpp"

namespace gw::server {

namespace {

class ImuConfigRepositoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        dir_ = std::filesystem::temp_directory_path() /
               ("gw_imu_cfg_test_" + std::to_string(::getpid()));
        std::filesystem::create_directories(dir_);
        path_ = dir_ / (std::string(::testing::UnitTest::GetInstance()
                                        ->current_test_info()
                                        ->name()) +
                        ".db");
        Database::remove_files(path_);
        db_   = std::make_unique<Database>(path_);
        repo_ = std::make_unique<ImuConfigRepository>(*db_);
    }
    void TearDown() override {
        repo_.reset();
        db_.reset();
        Database::remove_files(path_);
    }

    std::filesystem::path                dir_;
    std::filesystem::path                path_;
    std::unique_ptr<Database>            db_;
    std::unique_ptr<ImuConfigRepository> repo_;
};

}  // namespace

TEST_F(ImuConfigRepositoryTest, SeedRowExistsWithBmi088Defaults) {
    const auto c = repo_->get();
    EXPECT_DOUBLE_EQ(c.rate_hz, 400.0);
    EXPECT_GT(c.accel_noise_density, 0.0);
    EXPECT_GT(c.accel_random_walk, 0.0);
    EXPECT_GT(c.gyro_noise_density, 0.0);
    EXPECT_GT(c.gyro_random_walk, 0.0);
    EXPECT_FALSE(c.t_imu_robot_json.has_value());
}

TEST_F(ImuConfigRepositoryTest, PartialUpdatePreservesOtherFields) {
    const auto before = repo_->get();

    ImuConfigUpdate patch;
    patch.gyro_noise_density = 1.5e-4;
    const auto after = repo_->update(patch);

    EXPECT_DOUBLE_EQ(after.gyro_noise_density, 1.5e-4);
    EXPECT_DOUBLE_EQ(after.rate_hz, before.rate_hz);
    EXPECT_DOUBLE_EQ(after.accel_noise_density, before.accel_noise_density);
    EXPECT_DOUBLE_EQ(after.accel_random_walk, before.accel_random_walk);
    EXPECT_DOUBLE_EQ(after.gyro_random_walk, before.gyro_random_walk);
}

TEST_F(ImuConfigRepositoryTest, SetAndClearImuRobotTransform) {
    ImuConfigUpdate set;
    set.t_imu_robot_json =
        std::optional<std::string>{R"({"translation":[0.1,0.0,0.2]})"};
    auto c = repo_->update(set);
    ASSERT_TRUE(c.t_imu_robot_json.has_value());
    EXPECT_NE(c.t_imu_robot_json->find("translation"), std::string::npos);

    ImuConfigUpdate clear;
    clear.t_imu_robot_json = std::optional<std::string>{};
    c = repo_->update(clear);
    EXPECT_FALSE(c.t_imu_robot_json.has_value());
}

TEST_F(ImuConfigRepositoryTest, EmptyPatchIsNoOp) {
    const auto before = repo_->get();
    const auto after  = repo_->update(ImuConfigUpdate{});
    EXPECT_DOUBLE_EQ(after.rate_hz, before.rate_hz);
    EXPECT_EQ(after.updated_at, before.updated_at);
}

TEST_F(ImuConfigRepositoryTest, UpdateSurvivesReopen) {
    ImuConfigUpdate patch;
    patch.rate_hz = 200.0;
    repo_->update(patch);

    repo_.reset();
    db_.reset();
    db_   = std::make_unique<Database>(path_);
    repo_ = std::make_unique<ImuConfigRepository>(*db_);

    EXPECT_DOUBLE_EQ(repo_->get().rate_hz, 200.0);
}

}  // namespace gw::server
