#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>

#include "server/database.hpp"
#include "server/vio_config_repository.hpp"

namespace gw::server {

namespace {

class VioConfigRepositoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        path_ = std::filesystem::temp_directory_path() /
                ("gw_vio_cfg_test_" + std::to_string(::getpid()) + "_" +
                 ::testing::UnitTest::GetInstance()->current_test_info()->name() +
                 ".db");
        Database::remove_files(path_);
        db_   = std::make_unique<Database>(path_);
        repo_ = std::make_unique<VioConfigRepository>(*db_);
    }
    void TearDown() override {
        repo_.reset();
        db_.reset();
        Database::remove_files(path_);
    }

    std::filesystem::path                path_;
    std::unique_ptr<Database>            db_;
    std::unique_ptr<VioConfigRepository> repo_;
};

}  // namespace

TEST_F(VioConfigRepositoryTest, SeedRowHasDefaults) {
    const auto c = repo_->get();
    EXPECT_TRUE(c.enabled);
    EXPECT_EQ(c.num_pts, 150);
    EXPECT_EQ(c.fast_threshold, 20);
    EXPECT_TRUE(c.downsample);
    EXPECT_DOUBLE_EQ(c.max_reproj_std_px, 1.0);
    EXPECT_TRUE(c.auto_reinit);
    EXPECT_EQ(c.reinit_min_features, 15);
    EXPECT_EQ(c.reinit_window_frames, 15);
    EXPECT_DOUBLE_EQ(c.reinit_max_pos_std_m, 2.0);
}

TEST_F(VioConfigRepositoryTest, PartialUpdatePreservesOthers) {
    VioConfigUpdate patch;
    patch.num_pts = 200;
    patch.enabled = false;
    const auto c = repo_->update(patch);
    EXPECT_EQ(c.num_pts, 200);
    EXPECT_FALSE(c.enabled);
    EXPECT_EQ(c.fast_threshold, 20);  // untouched
    EXPECT_DOUBLE_EQ(c.max_reproj_std_px, 1.0);
}

TEST_F(VioConfigRepositoryTest, CheckConstraintRejected) {
    VioConfigUpdate patch;
    patch.num_pts = 10;  // below CHECK 50..400
    EXPECT_THROW(repo_->update(patch), std::exception);
    // Row unchanged after the failed update.
    EXPECT_EQ(repo_->get().num_pts, 150);
}

TEST_F(VioConfigRepositoryTest, EmptyPatchIsNoOp) {
    const auto before = repo_->get();
    const auto after  = repo_->update(VioConfigUpdate{});
    EXPECT_EQ(after.updated_at, before.updated_at);
}

}  // namespace gw::server
