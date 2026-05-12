#pragma once

#include <filesystem>
#include <mutex>
#include <utility>

struct sqlite3;

namespace gw::server {

// Thread-safe SQLite handle. Opens (and creates if missing) the database file
// on construction, applies PRAGMAs, and runs schema migrations.
//
// Concurrent access from multiple HTTP worker threads is serialised through
// with_handle(): callers receive the raw sqlite3* under the lock. SQLite's own
// busy_timeout PRAGMA covers the (rare) case where a second OS process opens
// the same file.
class Database {
public:
    explicit Database(const std::filesystem::path& db_path);
    ~Database();

    Database(const Database&)            = delete;
    Database& operator=(const Database&) = delete;

    template <typename F>
    auto with_handle(F&& fn) -> decltype(std::forward<F>(fn)(std::declval<sqlite3*>())) {
        std::lock_guard<std::mutex> lk(mu_);
        return std::forward<F>(fn)(db_);
    }

    // Resolves $HOME/.guesswork/guesswork.db. Throws if $HOME is unset.
    static std::filesystem::path default_path();

private:
    sqlite3*   db_ = nullptr;
    std::mutex mu_;
};

}  // namespace gw::server
