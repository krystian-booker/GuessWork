#include "server/database.hpp"

#include <sqlite3.h>

#include <cstdlib>
#include <stdexcept>
#include <string>

namespace gw::server {

namespace {

void exec_or_throw(sqlite3* db, const char* sql) {
    char* err = nullptr;
    if (sqlite3_exec(db, sql, nullptr, nullptr, &err) != SQLITE_OK) {
        const std::string msg = err ? err : "unknown sqlite error";
        sqlite3_free(err);
        throw std::runtime_error(std::string("sqlite3_exec failed: ") + msg + " (sql: " + sql + ")");
    }
}

// Schema is the source of truth. While the project is in early development we
// don't write migrations - bump the schema here and run `guesswork --reset-db`
// (or call Database::remove_files) to start over.
constexpr const char* kSchemaCameras =
    "CREATE TABLE IF NOT EXISTS cameras ("
    "  id               INTEGER PRIMARY KEY AUTOINCREMENT,"
    "  name             TEXT    NOT NULL UNIQUE,"
    "  serial           TEXT    NOT NULL UNIQUE,"
    "  focal_length_mm  REAL    NOT NULL,"   // lens focal length, drives Kalibr focal hint + model
    "  mode             TEXT,"
    "  gain_auto        INTEGER,"
    "  gain             REAL,"
    "  exposure_auto    INTEGER,"
    "  exposure         REAL,"
    "  calibration_json TEXT,"      // Kalibr camchain YAML, NULL = uncalibrated
    "  calibrated_at    INTEGER,"   // unix seconds when calibration was uploaded
    "  created_at       INTEGER NOT NULL DEFAULT (strftime('%s', 'now'))"
    ");";

}  // namespace

Database::Database(const std::filesystem::path& db_path) {
    if (db_path.has_parent_path()) {
        std::filesystem::create_directories(db_path.parent_path());
    }

    if (sqlite3_open(db_path.string().c_str(), &db_) != SQLITE_OK) {
        const std::string msg = db_ ? sqlite3_errmsg(db_) : "sqlite3_open failed";
        sqlite3_close(db_);
        db_ = nullptr;
        throw std::runtime_error("Database: open failed: " + msg);
    }

    exec_or_throw(db_, "PRAGMA journal_mode = WAL;");
    exec_or_throw(db_, "PRAGMA foreign_keys = ON;");
    exec_or_throw(db_, "PRAGMA busy_timeout = 2000;");
    exec_or_throw(db_, kSchemaCameras);
}

void Database::remove_files(const std::filesystem::path& db_path) {
    std::error_code ec;
    std::filesystem::remove(db_path, ec);
    std::filesystem::remove(db_path.string() + "-wal", ec);
    std::filesystem::remove(db_path.string() + "-shm", ec);
}

Database::~Database() {
    if (db_) {
        sqlite3_close(db_);
    }
}

std::filesystem::path Database::default_path() {
    return data_dir() / "guesswork.db";
}

std::filesystem::path Database::data_dir() {
    const char* home = std::getenv("HOME");
    if (!home || !*home) {
        throw std::runtime_error("Database::data_dir: $HOME is not set");
    }
    return std::filesystem::path(home) / ".guesswork";
}

}  // namespace gw::server
