#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>

#include "server/can_config_repository.hpp"
#include "server/database.hpp"

namespace gw::server {

namespace {

class CanConfigRepositoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        path_ = std::filesystem::temp_directory_path() /
                ("gw_can_cfg_test_" + std::to_string(::getpid()) + "_" +
                 ::testing::UnitTest::GetInstance()->current_test_info()->name() +
                 ".db");
        Database::remove_files(path_);
        db_   = std::make_unique<Database>(path_);
        repo_ = std::make_unique<CanConfigRepository>(*db_);
    }
    void TearDown() override {
        repo_.reset();
        db_.reset();
        Database::remove_files(path_);
    }

    std::filesystem::path                path_;
    std::unique_ptr<Database>            db_;
    std::unique_ptr<CanConfigRepository> repo_;
};

}  // namespace

TEST_F(CanConfigRepositoryTest, SeedRowDefaultsToOff) {
    const auto c = repo_->get();
    EXPECT_EQ(c.mode, "off");
    EXPECT_GT(c.updated_at, 0);
}

TEST_F(CanConfigRepositoryTest, UpdatesMode) {
    CanConfigUpdate patch;
    patch.mode = "roborio";
    EXPECT_EQ(repo_->update(patch).mode, "roborio");
    EXPECT_EQ(repo_->get().mode, "roborio");

    patch.mode = "systemcore";
    EXPECT_EQ(repo_->update(patch).mode, "systemcore");

    patch.mode = "off";
    EXPECT_EQ(repo_->update(patch).mode, "off");
}

TEST_F(CanConfigRepositoryTest, CheckConstraintRejectsUnknownMode) {
    CanConfigUpdate patch;
    patch.mode = "canopen";
    EXPECT_THROW(repo_->update(patch), std::exception);
    EXPECT_EQ(repo_->get().mode, "off");  // row unchanged
}

TEST_F(CanConfigRepositoryTest, EmptyPatchIsNoOp) {
    const auto before = repo_->get();
    const auto after  = repo_->update(CanConfigUpdate{});
    EXPECT_EQ(after.mode, before.mode);
    EXPECT_EQ(after.updated_at, before.updated_at);
}

}  // namespace gw::server
