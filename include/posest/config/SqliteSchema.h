#pragma once

struct sqlite3;

namespace posest::config {

int currentSchemaVersion();
void applyMigrations(sqlite3* db);

}  // namespace posest::config
