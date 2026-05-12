#include <gtest/gtest.h>
#include <sqlite3.h>

#include <cstdlib>
#include <filesystem>
#include <string>

#include "server/database.hpp"

namespace gw::server {

namespace {

std::filesystem::path make_temp_db_path(const char* slot) {
    const auto dir =
        std::filesystem::temp_directory_path() / ("gw_db_test_" + std::to_string(::getpid()));
    std::filesystem::create_directories(dir);
    return dir / (std::string(slot) + ".db");
}

void cleanup(const std::filesystem::path& p) {
    std::error_code ec;
    std::filesystem::remove(p, ec);
    std::filesystem::remove(p.string() + "-wal", ec);
    std::filesystem::remove(p.string() + "-shm", ec);
}

}  // namespace

TEST(DatabaseTest, CreatesParentDirectory) {
    const auto root = std::filesystem::temp_directory_path() /
                      ("gw_db_test_" + std::to_string(::getpid())) / "nested" / "deep";
    std::filesystem::remove_all(root.parent_path());
    const auto path = root / "db.sqlite";

    { Database db(path); }

    EXPECT_TRUE(std::filesystem::exists(path));
    std::filesystem::remove_all(root.parent_path().parent_path());
}

TEST(DatabaseTest, OpensAndCreatesCamerasTable) {
    const auto path = make_temp_db_path("schema");
    cleanup(path);

    Database db(path);
    db.with_handle([](sqlite3* h) {
        sqlite3_stmt* stmt = nullptr;
        ASSERT_EQ(SQLITE_OK,
                  sqlite3_prepare_v2(
                      h,
                      "SELECT name FROM sqlite_master WHERE type='table' AND name='cameras';",
                      -1, &stmt, nullptr));
        EXPECT_EQ(SQLITE_ROW, sqlite3_step(stmt));
        sqlite3_finalize(stmt);
    });

    cleanup(path);
}

TEST(DatabaseTest, ReopenIsIdempotent) {
    const auto path = make_temp_db_path("reopen");
    cleanup(path);

    { Database first(path); }
    EXPECT_NO_THROW({ Database second(path); });

    cleanup(path);
}

TEST(DatabaseTest, DefaultPathHonoursHome) {
    const char* old_home = std::getenv("HOME");
    setenv("HOME", "/tmp/gw_home_for_test", 1);
    const auto p = Database::default_path();
    EXPECT_EQ(p, std::filesystem::path("/tmp/gw_home_for_test") / ".guesswork" / "guesswork.db");
    if (old_home) {
        setenv("HOME", old_home, 1);
    } else {
        unsetenv("HOME");
    }
}

}  // namespace gw::server
