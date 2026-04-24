#pragma once

#include <filesystem>
#include <mutex>

#include "posest/config/IConfigStore.h"

struct sqlite3;

namespace posest::config {

class SqliteConfigStore final : public IConfigStore {
public:
    explicit SqliteConfigStore(std::filesystem::path db_path);
    ~SqliteConfigStore() override;

    SqliteConfigStore(const SqliteConfigStore&) = delete;
    SqliteConfigStore& operator=(const SqliteConfigStore&) = delete;

    runtime::RuntimeConfig loadRuntimeConfig() const override;
    void saveRuntimeConfig(const runtime::RuntimeConfig& config) override;

    const std::filesystem::path& path() const { return db_path_; }

private:
    sqlite3* db_{nullptr};
    std::filesystem::path db_path_;
    mutable std::mutex mu_;
};

}  // namespace posest::config
