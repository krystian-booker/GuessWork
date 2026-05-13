#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>

#include "server/camera_repository.hpp"
#include "server/database.hpp"

namespace gw::server {

namespace {

class CameraRepositoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        path_ = std::filesystem::temp_directory_path() /
                ("gw_repo_test_" + std::to_string(::getpid()) + "_" +
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

}  // namespace

TEST_F(CameraRepositoryTest, ListAllEmptyByDefault) {
    EXPECT_TRUE(repo_->list_all().empty());
}

TEST_F(CameraRepositoryTest, CreateReturnsRowWithIdAndSerial) {
    const auto c = repo_->create("front", "SN001");
    EXPECT_GT(c.id, 0);
    EXPECT_EQ(c.name, "front");
    EXPECT_EQ(c.serial, "SN001");
    EXPECT_GT(c.created_at, 0);
}

TEST_F(CameraRepositoryTest, CreateThenListReturnsRow) {
    repo_->create("front", "SN001");
    repo_->create("rear",  "SN002");
    const auto rows = repo_->list_all();
    ASSERT_EQ(rows.size(), 2u);
    EXPECT_EQ(rows[0].name,   "front");
    EXPECT_EQ(rows[0].serial, "SN001");
    EXPECT_EQ(rows[1].name,   "rear");
    EXPECT_EQ(rows[1].serial, "SN002");
}

TEST_F(CameraRepositoryTest, CreateDuplicateNameThrows) {
    repo_->create("front", "SN001");
    EXPECT_THROW(repo_->create("front", "SN002"), DuplicateNameError);
}

TEST_F(CameraRepositoryTest, CreateDuplicateSerialThrows) {
    repo_->create("front", "SN001");
    EXPECT_THROW(repo_->create("rear", "SN001"), DuplicateSerialError);
}

TEST_F(CameraRepositoryTest, GetReturnsRowById) {
    const auto created = repo_->create("front", "SN001");
    const auto fetched = repo_->get(created.id);
    ASSERT_TRUE(fetched.has_value());
    EXPECT_EQ(fetched->name,   "front");
    EXPECT_EQ(fetched->serial, "SN001");
    EXPECT_EQ(fetched->id,     created.id);
}

TEST_F(CameraRepositoryTest, GetMissingIdReturnsNullopt) {
    EXPECT_FALSE(repo_->get(9999).has_value());
}

TEST_F(CameraRepositoryTest, FindBySerialReturnsRow) {
    repo_->create("front", "SN001");
    const auto found = repo_->find_by_serial("SN001");
    ASSERT_TRUE(found.has_value());
    EXPECT_EQ(found->name, "front");
}

TEST_F(CameraRepositoryTest, FindBySerialMissingReturnsNullopt) {
    EXPECT_FALSE(repo_->find_by_serial("nope").has_value());
}

TEST_F(CameraRepositoryTest, UpdateChangesName) {
    const auto c       = repo_->create("front", "SN001");
    const auto updated = repo_->update(c.id, "front-left");
    ASSERT_TRUE(updated.has_value());
    EXPECT_EQ(updated->name,   "front-left");
    EXPECT_EQ(updated->serial, "SN001");
    EXPECT_EQ(updated->id,     c.id);

    const auto refetched = repo_->get(c.id);
    ASSERT_TRUE(refetched.has_value());
    EXPECT_EQ(refetched->name, "front-left");
}

TEST_F(CameraRepositoryTest, UpdateMissingIdReturnsNullopt) {
    EXPECT_FALSE(repo_->update(9999, "ghost").has_value());
}

TEST_F(CameraRepositoryTest, UpdateToExistingNameThrows) {
    repo_->create("front", "SN001");
    const auto rear = repo_->create("rear", "SN002");
    EXPECT_THROW(repo_->update(rear.id, "front"), DuplicateNameError);
}

TEST_F(CameraRepositoryTest, RemoveReturnsTrueWhenRowDeleted) {
    const auto c = repo_->create("front", "SN001");
    EXPECT_TRUE(repo_->remove(c.id));
    EXPECT_FALSE(repo_->get(c.id).has_value());
}

TEST_F(CameraRepositoryTest, RemoveReturnsFalseWhenMissing) {
    EXPECT_FALSE(repo_->remove(9999));
}

}  // namespace gw::server
