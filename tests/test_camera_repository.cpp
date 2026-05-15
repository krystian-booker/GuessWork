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
    const auto c = repo_->create("front", "SN001", "pinhole");
    EXPECT_GT(c.id, 0);
    EXPECT_EQ(c.name, "front");
    EXPECT_EQ(c.serial, "SN001");
    EXPECT_EQ(c.lens_type, "pinhole");
    EXPECT_FALSE(c.mode.has_value());
    EXPECT_GT(c.created_at, 0);
}

TEST_F(CameraRepositoryTest, CreateRoundTripsFisheyeLensType) {
    const auto c = repo_->create("front", "SN001", "fisheye");
    EXPECT_EQ(c.lens_type, "fisheye");
    const auto fetched = repo_->get(c.id);
    ASSERT_TRUE(fetched.has_value());
    EXPECT_EQ(fetched->lens_type, "fisheye");
}

TEST_F(CameraRepositoryTest, UpdateLensType) {
    const auto c = repo_->create("front", "SN001", "pinhole");
    CameraUpdate u; u.lens_type = "fisheye";
    const auto upd = repo_->update(c.id, u);
    ASSERT_TRUE(upd.has_value());
    EXPECT_EQ(upd->lens_type, "fisheye");
}

TEST_F(CameraRepositoryTest, CreateWithModeRoundTrips) {
    const auto c = repo_->create("front", "SN001", "pinhole", std::string_view("Mode1"));
    ASSERT_TRUE(c.mode.has_value());
    EXPECT_EQ(*c.mode, "Mode1");

    const auto fetched = repo_->get(c.id);
    ASSERT_TRUE(fetched.has_value());
    ASSERT_TRUE(fetched->mode.has_value());
    EXPECT_EQ(*fetched->mode, "Mode1");
}

TEST_F(CameraRepositoryTest, CreateWithoutModeStoresNull) {
    const auto c = repo_->create("front", "SN001", "pinhole");
    const auto fetched = repo_->get(c.id);
    ASSERT_TRUE(fetched.has_value());
    EXPECT_FALSE(fetched->mode.has_value());
}

TEST_F(CameraRepositoryTest, CreateThenListReturnsRow) {
    repo_->create("front", "SN001", "pinhole");
    repo_->create("rear",  "SN002", "pinhole");
    const auto rows = repo_->list_all();
    ASSERT_EQ(rows.size(), 2u);
    EXPECT_EQ(rows[0].name,   "front");
    EXPECT_EQ(rows[0].serial, "SN001");
    EXPECT_EQ(rows[1].name,   "rear");
    EXPECT_EQ(rows[1].serial, "SN002");
}

TEST_F(CameraRepositoryTest, CreateDuplicateNameThrows) {
    repo_->create("front", "SN001", "pinhole");
    EXPECT_THROW(repo_->create("front", "SN002", "pinhole"), DuplicateNameError);
}

TEST_F(CameraRepositoryTest, CreateDuplicateSerialThrows) {
    repo_->create("front", "SN001", "pinhole");
    EXPECT_THROW(repo_->create("rear", "SN001", "pinhole"), DuplicateSerialError);
}

TEST_F(CameraRepositoryTest, GetReturnsRowById) {
    const auto created = repo_->create("front", "SN001", "pinhole");
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
    repo_->create("front", "SN001", "pinhole");
    const auto found = repo_->find_by_serial("SN001");
    ASSERT_TRUE(found.has_value());
    EXPECT_EQ(found->name, "front");
}

TEST_F(CameraRepositoryTest, FindBySerialMissingReturnsNullopt) {
    EXPECT_FALSE(repo_->find_by_serial("nope").has_value());
}

TEST_F(CameraRepositoryTest, UpdateChangesName) {
    const auto c       = repo_->create("front", "SN001", "pinhole");
    CameraUpdate u; u.name = "front-left";
    const auto updated = repo_->update(c.id, u);
    ASSERT_TRUE(updated.has_value());
    EXPECT_EQ(updated->name,   "front-left");
    EXPECT_EQ(updated->serial, "SN001");
    EXPECT_EQ(updated->id,     c.id);

    const auto refetched = repo_->get(c.id);
    ASSERT_TRUE(refetched.has_value());
    EXPECT_EQ(refetched->name, "front-left");
}

TEST_F(CameraRepositoryTest, UpdateMissingIdReturnsNullopt) {
    CameraUpdate u; u.name = "ghost";
    EXPECT_FALSE(repo_->update(9999, u).has_value());
}

TEST_F(CameraRepositoryTest, UpdateToExistingNameThrows) {
    repo_->create("front", "SN001", "pinhole");
    const auto rear = repo_->create("rear", "SN002", "pinhole");
    CameraUpdate u; u.name = "front";
    EXPECT_THROW(repo_->update(rear.id, u), DuplicateNameError);
}

TEST_F(CameraRepositoryTest, UpdateModeOnly) {
    const auto c = repo_->create("front", "SN001", "pinhole");
    CameraUpdate u; u.mode = "Mode1";
    const auto updated = repo_->update(c.id, u);
    ASSERT_TRUE(updated.has_value());
    EXPECT_EQ(updated->name, "front");        // unchanged
    ASSERT_TRUE(updated->mode.has_value());
    EXPECT_EQ(*updated->mode, "Mode1");
}

TEST_F(CameraRepositoryTest, UpdateNameAndMode) {
    const auto c = repo_->create("front", "SN001", "pinhole");
    CameraUpdate u; u.name = "front-left"; u.mode = "Mode5";
    const auto updated = repo_->update(c.id, u);
    ASSERT_TRUE(updated.has_value());
    EXPECT_EQ(updated->name, "front-left");
    ASSERT_TRUE(updated->mode.has_value());
    EXPECT_EQ(*updated->mode, "Mode5");
}

TEST_F(CameraRepositoryTest, UpdateNeitherIsNoOpReturningCurrentRow) {
    const auto c = repo_->create("front", "SN001", "pinhole", std::string_view("Mode0"));
    const auto unchanged = repo_->update(c.id, CameraUpdate{});
    ASSERT_TRUE(unchanged.has_value());
    EXPECT_EQ(unchanged->id, c.id);
    EXPECT_EQ(unchanged->name, "front");
    ASSERT_TRUE(unchanged->mode.has_value());
    EXPECT_EQ(*unchanged->mode, "Mode0");
}

TEST_F(CameraRepositoryTest, CreateLeavesSettingsNull) {
    const auto c = repo_->create("front", "SN001", "pinhole");
    EXPECT_FALSE(c.gain_auto.has_value());
    EXPECT_FALSE(c.gain.has_value());
    EXPECT_FALSE(c.exposure_auto.has_value());
    EXPECT_FALSE(c.exposure.has_value());
}

TEST_F(CameraRepositoryTest, UpdateGainAutoOnlyRoundTrips) {
    const auto c = repo_->create("front", "SN001", "pinhole");
    CameraUpdate u; u.gain_auto = true;
    const auto upd = repo_->update(c.id, u);
    ASSERT_TRUE(upd.has_value());
    ASSERT_TRUE(upd->gain_auto.has_value());
    EXPECT_TRUE(*upd->gain_auto);
    EXPECT_FALSE(upd->gain.has_value());
}

TEST_F(CameraRepositoryTest, UpdateGainAutoAndValueRoundTrips) {
    const auto c = repo_->create("front", "SN001", "pinhole");
    CameraUpdate u; u.gain_auto = false; u.gain = 12.5;
    const auto upd = repo_->update(c.id, u);
    ASSERT_TRUE(upd.has_value());
    ASSERT_TRUE(upd->gain_auto.has_value());
    EXPECT_FALSE(*upd->gain_auto);
    ASSERT_TRUE(upd->gain.has_value());
    EXPECT_DOUBLE_EQ(*upd->gain, 12.5);
}

TEST_F(CameraRepositoryTest, UpdateExposureRoundTrips) {
    const auto c = repo_->create("front", "SN001", "pinhole");
    CameraUpdate u; u.exposure_auto = false; u.exposure = 8333.0;
    const auto upd = repo_->update(c.id, u);
    ASSERT_TRUE(upd.has_value());
    ASSERT_TRUE(upd->exposure_auto.has_value());
    EXPECT_FALSE(*upd->exposure_auto);
    ASSERT_TRUE(upd->exposure.has_value());
    EXPECT_DOUBLE_EQ(*upd->exposure, 8333.0);
}

TEST_F(CameraRepositoryTest, UpdateAllSettingsAtOnce) {
    const auto c = repo_->create("front", "SN001", "pinhole");
    CameraUpdate u;
    u.gain_auto     = false;
    u.gain          = 6.0;
    u.exposure_auto = false;
    u.exposure      = 16666.0;
    const auto upd = repo_->update(c.id, u);
    ASSERT_TRUE(upd.has_value());
    EXPECT_EQ(*upd->gain_auto,     false);
    EXPECT_DOUBLE_EQ(*upd->gain,          6.0);
    EXPECT_EQ(*upd->exposure_auto, false);
    EXPECT_DOUBLE_EQ(*upd->exposure,      16666.0);

    const auto refetched = repo_->get(c.id);
    ASSERT_TRUE(refetched.has_value());
    EXPECT_DOUBLE_EQ(*refetched->gain,     6.0);
    EXPECT_DOUBLE_EQ(*refetched->exposure, 16666.0);
}

TEST_F(CameraRepositoryTest, UpdatePartialSettingsLeavesOthersUntouched) {
    const auto c = repo_->create("front", "SN001", "pinhole");
    {
        CameraUpdate u; u.gain = 10.0; u.exposure = 5000.0;
        repo_->update(c.id, u);
    }
    {
        CameraUpdate u; u.gain_auto = true;
        const auto upd = repo_->update(c.id, u);
        ASSERT_TRUE(upd.has_value());
        ASSERT_TRUE(upd->gain_auto.has_value());
        EXPECT_TRUE(*upd->gain_auto);
        // gain + exposure from the previous update must survive.
        ASSERT_TRUE(upd->gain.has_value());
        EXPECT_DOUBLE_EQ(*upd->gain, 10.0);
        ASSERT_TRUE(upd->exposure.has_value());
        EXPECT_DOUBLE_EQ(*upd->exposure, 5000.0);
    }
}

TEST_F(CameraRepositoryTest, ListAllReturnsMode) {
    repo_->create("front", "SN001", "pinhole", std::string_view("Mode1"));
    repo_->create("rear",  "SN002", "pinhole");
    const auto rows = repo_->list_all();
    ASSERT_EQ(rows.size(), 2u);
    ASSERT_TRUE(rows[0].mode.has_value());
    EXPECT_EQ(*rows[0].mode, "Mode1");
    EXPECT_FALSE(rows[1].mode.has_value());
}

TEST_F(CameraRepositoryTest, RemoveReturnsTrueWhenRowDeleted) {
    const auto c = repo_->create("front", "SN001", "pinhole");
    EXPECT_TRUE(repo_->remove(c.id));
    EXPECT_FALSE(repo_->get(c.id).has_value());
}

TEST_F(CameraRepositoryTest, RemoveReturnsFalseWhenMissing) {
    EXPECT_FALSE(repo_->remove(9999));
}

}  // namespace gw::server
