#include <gtest/gtest.h>

#include <filesystem>

#include "server/database.hpp"
#include "server/net_config_repository.hpp"

namespace gw::server {

namespace {

class NetConfigRepositoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        dir_ = std::filesystem::temp_directory_path() /
               ("gw_net_cfg_test_" + std::to_string(::getpid()));
        std::filesystem::create_directories(dir_);
        db_ = std::make_unique<Database>(dir_ / "test.db");
    }
    void TearDown() override {
        db_.reset();
        std::filesystem::remove_all(dir_);
    }

    std::filesystem::path     dir_;
    std::unique_ptr<Database> db_;
};

}  // namespace

TEST_F(NetConfigRepositoryTest, SeedDefaults) {
    NetConfigRepository repo(*db_);
    const auto c = repo.get();
    EXPECT_TRUE(c.enabled);
    EXPECT_EQ(c.bind_port, 5809);
    EXPECT_EQ(c.robot_port, 5810);
    EXPECT_EQ(c.robot_ip, "");
    EXPECT_GT(c.updated_at, 0);
}

TEST_F(NetConfigRepositoryTest, PartialUpdatePreservesOtherFields) {
    NetConfigRepository repo(*db_);
    NetConfigUpdate patch;
    patch.robot_ip = "10.28.52.2";
    const auto c = repo.update(patch);
    EXPECT_EQ(c.robot_ip, "10.28.52.2");
    EXPECT_EQ(c.bind_port, 5809);
    EXPECT_TRUE(c.enabled);

    NetConfigUpdate patch2;
    patch2.enabled   = false;
    patch2.bind_port = 5801;
    const auto c2 = repo.update(patch2);
    EXPECT_FALSE(c2.enabled);
    EXPECT_EQ(c2.bind_port, 5801);
    EXPECT_EQ(c2.robot_ip, "10.28.52.2");  // survived
}

TEST_F(NetConfigRepositoryTest, CheckConstraintRejectsBadPort) {
    NetConfigRepository repo(*db_);
    NetConfigUpdate patch;
    patch.bind_port = 80;  // below 1024
    EXPECT_THROW(repo.update(patch), std::runtime_error);
    EXPECT_EQ(repo.get().bind_port, 5809);  // unchanged
}

TEST_F(NetConfigRepositoryTest, EmptyPatchIsNoOp) {
    NetConfigRepository repo(*db_);
    const auto before = repo.get();
    const auto after  = repo.update(NetConfigUpdate{});
    EXPECT_EQ(after.bind_port, before.bind_port);
    EXPECT_EQ(after.robot_ip, before.robot_ip);
}

TEST_F(NetConfigRepositoryTest, ClearingRobotIpRestoresAutoLearn) {
    NetConfigRepository repo(*db_);
    NetConfigUpdate set;
    set.robot_ip = "10.28.52.2";
    repo.update(set);
    NetConfigUpdate clear;
    clear.robot_ip = "";
    EXPECT_EQ(repo.update(clear).robot_ip, "");
}

}  // namespace gw::server
