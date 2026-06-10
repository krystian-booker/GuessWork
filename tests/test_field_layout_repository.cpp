#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>

#include "server/database.hpp"
#include "server/field_layout_repository.hpp"

namespace gw::server {

namespace {

constexpr const char* kStubLayoutJson =
    R"({"field": {"length": 16.541, "width": 8.069}, "tags": []})";

class FieldLayoutRepositoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        path_ = std::filesystem::temp_directory_path() /
                ("gw_layout_test_" + std::to_string(::getpid()) + "_" +
                 ::testing::UnitTest::GetInstance()->current_test_info()->name() +
                 ".db");
        Database::remove_files(path_);
        db_   = std::make_unique<Database>(path_);
        repo_ = std::make_unique<FieldLayoutRepository>(*db_);
    }
    void TearDown() override {
        repo_.reset();
        db_.reset();
        Database::remove_files(path_);
    }

    std::filesystem::path                  path_;
    std::unique_ptr<Database>              db_;
    std::unique_ptr<FieldLayoutRepository> repo_;
};

}  // namespace

TEST_F(FieldLayoutRepositoryTest, FirstRowAutoActivates) {
    const auto a = repo_->create("2026-rebuilt", kStubLayoutJson);
    EXPECT_TRUE(a.active);
    const auto b = repo_->create("bench", kStubLayoutJson);
    EXPECT_FALSE(b.active);

    const auto active = repo_->get_active();
    ASSERT_TRUE(active.has_value());
    EXPECT_EQ(active->id, a.id);
}

TEST_F(FieldLayoutRepositoryTest, ActivateSwapsExactlyOneActive) {
    const auto a = repo_->create("a", kStubLayoutJson);
    const auto b = repo_->create("b", kStubLayoutJson);

    EXPECT_TRUE(repo_->activate(b.id));

    int active_count = 0;
    for (const auto& r : repo_->list_all()) {
        if (r.active) ++active_count;
    }
    EXPECT_EQ(active_count, 1);
    EXPECT_EQ(repo_->get_active()->id, b.id);

    // Unknown id → false, active row unchanged.
    EXPECT_FALSE(repo_->activate(9999));
    EXPECT_EQ(repo_->get_active()->id, b.id);
    (void)a;
}

TEST_F(FieldLayoutRepositoryTest, DeleteActiveRejected) {
    const auto a = repo_->create("a", kStubLayoutJson);
    const auto b = repo_->create("b", kStubLayoutJson);

    EXPECT_THROW(repo_->remove(a.id), ActiveLayoutDeleteError);
    EXPECT_TRUE(repo_->remove(b.id));
    EXPECT_FALSE(repo_->remove(9999));
}

TEST_F(FieldLayoutRepositoryTest, DuplicateNameRejected) {
    repo_->create("a", kStubLayoutJson);
    EXPECT_THROW(repo_->create("a", kStubLayoutJson), DuplicateLayoutNameError);
}

TEST_F(FieldLayoutRepositoryTest, SeedIsIdempotent) {
    EXPECT_TRUE(repo_->seed_if_empty("default", kStubLayoutJson));
    EXPECT_FALSE(repo_->seed_if_empty("default", kStubLayoutJson));
    EXPECT_EQ(repo_->list_all().size(), 1u);
    EXPECT_TRUE(repo_->get_active().has_value());

    // Seed never overrides operator data: a user-activated row survives.
    const auto user = repo_->create("user", kStubLayoutJson);
    repo_->activate(user.id);
    EXPECT_FALSE(repo_->seed_if_empty("default", kStubLayoutJson));
    EXPECT_EQ(repo_->get_active()->id, user.id);
}

TEST_F(FieldLayoutRepositoryTest, GetReturnsVerbatimJson) {
    const auto a = repo_->create("a", kStubLayoutJson);
    const auto fetched = repo_->get(a.id);
    ASSERT_TRUE(fetched.has_value());
    EXPECT_EQ(fetched->json, kStubLayoutJson);
    EXPECT_FALSE(repo_->get(9999).has_value());
}

}  // namespace gw::server
