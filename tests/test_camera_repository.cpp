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
    EXPECT_FALSE(c.mode.has_value());
    EXPECT_GT(c.created_at, 0);
}

TEST_F(CameraRepositoryTest, CreateWithModeRoundTrips) {
    const auto c = repo_->create("front", "SN001", std::string_view("Mode1"));
    ASSERT_TRUE(c.mode.has_value());
    EXPECT_EQ(*c.mode, "Mode1");

    const auto fetched = repo_->get(c.id);
    ASSERT_TRUE(fetched.has_value());
    ASSERT_TRUE(fetched->mode.has_value());
    EXPECT_EQ(*fetched->mode, "Mode1");
}

TEST_F(CameraRepositoryTest, CreateWithoutModeStoresNull) {
    const auto c = repo_->create("front", "SN001");
    const auto fetched = repo_->get(c.id);
    ASSERT_TRUE(fetched.has_value());
    EXPECT_FALSE(fetched->mode.has_value());
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
    const auto updated = repo_->update(c.id, std::string_view("front-left"), std::nullopt);
    ASSERT_TRUE(updated.has_value());
    EXPECT_EQ(updated->name,   "front-left");
    EXPECT_EQ(updated->serial, "SN001");
    EXPECT_EQ(updated->id,     c.id);

    const auto refetched = repo_->get(c.id);
    ASSERT_TRUE(refetched.has_value());
    EXPECT_EQ(refetched->name, "front-left");
}

TEST_F(CameraRepositoryTest, UpdateMissingIdReturnsNullopt) {
    EXPECT_FALSE(
        repo_->update(9999, std::string_view("ghost"), std::nullopt).has_value());
}

TEST_F(CameraRepositoryTest, UpdateToExistingNameThrows) {
    repo_->create("front", "SN001");
    const auto rear = repo_->create("rear", "SN002");
    EXPECT_THROW(repo_->update(rear.id, std::string_view("front"), std::nullopt),
                 DuplicateNameError);
}

TEST_F(CameraRepositoryTest, UpdateModeOnly) {
    const auto c = repo_->create("front", "SN001");
    const auto updated = repo_->update(c.id, std::nullopt, std::string_view("Mode1"));
    ASSERT_TRUE(updated.has_value());
    EXPECT_EQ(updated->name, "front");        // unchanged
    ASSERT_TRUE(updated->mode.has_value());
    EXPECT_EQ(*updated->mode, "Mode1");
}

TEST_F(CameraRepositoryTest, UpdateNameAndMode) {
    const auto c = repo_->create("front", "SN001");
    const auto updated = repo_->update(
        c.id, std::string_view("front-left"), std::string_view("Mode5"));
    ASSERT_TRUE(updated.has_value());
    EXPECT_EQ(updated->name, "front-left");
    ASSERT_TRUE(updated->mode.has_value());
    EXPECT_EQ(*updated->mode, "Mode5");
}

TEST_F(CameraRepositoryTest, UpdateNeitherIsNoOpReturningCurrentRow) {
    const auto c = repo_->create("front", "SN001", std::string_view("Mode0"));
    const auto unchanged = repo_->update(c.id, std::nullopt, std::nullopt);
    ASSERT_TRUE(unchanged.has_value());
    EXPECT_EQ(unchanged->id, c.id);
    EXPECT_EQ(unchanged->name, "front");
    ASSERT_TRUE(unchanged->mode.has_value());
    EXPECT_EQ(*unchanged->mode, "Mode0");
}

TEST_F(CameraRepositoryTest, ListAllReturnsMode) {
    repo_->create("front", "SN001", std::string_view("Mode1"));
    repo_->create("rear",  "SN002");
    const auto rows = repo_->list_all();
    ASSERT_EQ(rows.size(), 2u);
    ASSERT_TRUE(rows[0].mode.has_value());
    EXPECT_EQ(*rows[0].mode, "Mode1");
    EXPECT_FALSE(rows[1].mode.has_value());
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
