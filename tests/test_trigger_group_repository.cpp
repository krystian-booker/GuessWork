#include <gtest/gtest.h>

#include <filesystem>

#include "server/database.hpp"
#include "server/trigger_group_repository.hpp"

namespace gw::server {

namespace {

class TriggerGroupRepositoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        dir_ = std::filesystem::temp_directory_path() /
               ("gw_trigger_repo_test_" + std::to_string(::getpid()));
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

TEST_F(TriggerGroupRepositoryTest, CreateListRoundTrip) {
    TriggerGroupRepository repo(*db_);
    const auto g = repo.create("apriltag", 30.0, {1, 2, 3});
    EXPECT_GT(g.id, 0);
    const auto all = repo.list_all();
    ASSERT_EQ(all.size(), 1u);
    EXPECT_EQ(all[0].name, "apriltag");
    EXPECT_EQ(all[0].fps, 30.0);
    EXPECT_EQ(all[0].output_pins, (std::vector<uint8_t>{1, 2, 3}));
}

TEST_F(TriggerGroupRepositoryTest, RejectsCrossGroupPinConflict) {
    TriggerGroupRepository repo(*db_);
    repo.create("a", 30.0, {1, 2});
    EXPECT_THROW(repo.create("b", 60.0, {2, 3}),
                 TriggerOutputPinConflictError);
}

TEST_F(TriggerGroupRepositoryTest, ArmedIntentDefaultsOffAndPersists) {
    {
        TriggerGroupRepository repo(*db_);
        EXPECT_FALSE(repo.armed());  // seed default: disarmed
        repo.set_armed(true);
        EXPECT_TRUE(repo.armed());
    }
    // The whole point of the flag: it survives a process restart (robot
    // power-cycle) so boot-time auto-arm can converge with no operator.
    db_.reset();
    db_ = std::make_unique<Database>(dir_ / "test.db");
    TriggerGroupRepository repo(*db_);
    EXPECT_TRUE(repo.armed());
    repo.set_armed(false);
    EXPECT_FALSE(repo.armed());
}

}  // namespace gw::server
