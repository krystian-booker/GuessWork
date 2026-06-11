#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>

#include "server/database.hpp"
#include "server/fusion_config_repository.hpp"

namespace gw::server {

namespace {

class FusionConfigRepositoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        path_ = std::filesystem::temp_directory_path() /
                ("gw_fusion_cfg_test_" + std::to_string(::getpid()) + "_" +
                 ::testing::UnitTest::GetInstance()->current_test_info()->name() +
                 ".db");
        Database::remove_files(path_);
        db_   = std::make_unique<Database>(path_);
        repo_ = std::make_unique<FusionConfigRepository>(*db_);
    }
    void TearDown() override {
        repo_.reset();
        db_.reset();
        Database::remove_files(path_);
    }

    std::filesystem::path                   path_;
    std::unique_ptr<Database>               db_;
    std::unique_ptr<FusionConfigRepository> repo_;
};

}  // namespace

TEST_F(FusionConfigRepositoryTest, SeedRowHasDefaults) {
    const auto c = repo_->get();
    EXPECT_TRUE(c.enabled);
    EXPECT_DOUBLE_EQ(c.lag_s, 2.0);
    EXPECT_EQ(c.min_state_dt_ms, 25);
    EXPECT_EQ(c.output_hz, 100);
    EXPECT_EQ(c.max_extrapolation_ms, 150);
    EXPECT_DOUBLE_EQ(c.tag_gate_chi2, 22.46);
    EXPECT_DOUBLE_EQ(c.tag_huber_k, 1.345);
    EXPECT_DOUBLE_EQ(c.vio_huber_k, 1.345);
    EXPECT_DOUBLE_EQ(c.odom_cauchy_k, 0.5);
    EXPECT_DOUBLE_EQ(c.odom_sigma_vx, 0.05);
    EXPECT_DOUBLE_EQ(c.odom_sigma_vy, 0.05);
    EXPECT_DOUBLE_EQ(c.odom_sigma_omega, 0.05);
    EXPECT_DOUBLE_EQ(c.vio_sigma_rot, 0.01);
    EXPECT_DOUBLE_EQ(c.vio_sigma_trans, 0.01);
    EXPECT_DOUBLE_EQ(c.collision_inflation, 10.0);
    EXPECT_EQ(c.collision_window, 20);
    EXPECT_DOUBLE_EQ(c.reinit_pos_std_m, 1.0);
}

TEST_F(FusionConfigRepositoryTest, PartialUpdatePreservesOthers) {
    FusionConfigUpdate patch;
    patch.lag_s     = 3.0;
    patch.output_hz = 50;
    const auto c = repo_->update(patch);
    EXPECT_DOUBLE_EQ(c.lag_s, 3.0);
    EXPECT_EQ(c.output_hz, 50);
    EXPECT_EQ(c.min_state_dt_ms, 25);  // untouched
    EXPECT_DOUBLE_EQ(c.tag_gate_chi2, 22.46);
}

TEST_F(FusionConfigRepositoryTest, CheckConstraintRejected) {
    FusionConfigUpdate patch;
    patch.lag_s = 99.0;  // above CHECK 0.5..10
    EXPECT_THROW(repo_->update(patch), std::exception);
    EXPECT_DOUBLE_EQ(repo_->get().lag_s, 2.0);  // row unchanged
}

TEST_F(FusionConfigRepositoryTest, EmptyPatchIsNoOp) {
    const auto before = repo_->get();
    const auto after  = repo_->update(FusionConfigUpdate{});
    EXPECT_EQ(after.updated_at, before.updated_at);
}

}  // namespace gw::server
