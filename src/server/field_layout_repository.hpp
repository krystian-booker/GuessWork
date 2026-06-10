#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace gw::server {

class Database;

struct FieldLayoutRow {
    int64_t     id = 0;
    std::string name;
    std::string json;        // verbatim WPILib AprilTagFieldLayout document
    bool        active = false;
    int64_t     created_at = 0;  // unix seconds
};

// Thrown when deleting the active layout. Maps to HTTP 409 — activate a
// different layout first.
class ActiveLayoutDeleteError : public std::runtime_error {
public:
    ActiveLayoutDeleteError()
        : std::runtime_error("cannot delete the active field layout") {}
};

class DuplicateLayoutNameError : public std::runtime_error {
public:
    explicit DuplicateLayoutNameError(const std::string& name)
        : std::runtime_error("field layout name already exists: " + name) {}
};

// Storage for WPILib field layouts. Exactly one row is active at a time
// (enforced by activate()'s transaction); the first row ever created is
// auto-activated so the pipeline never boots layout-less after seeding.
class FieldLayoutRepository {
public:
    explicit FieldLayoutRepository(Database& db) : db_(db) {}

    std::vector<FieldLayoutRow>   list_all();
    std::optional<FieldLayoutRow> get(int64_t id);
    std::optional<FieldLayoutRow> get_active();

    // JSON validity is the route layer's concern (parse_field_layout_json
    // before insert). Throws DuplicateLayoutNameError on UNIQUE conflict.
    FieldLayoutRow create(std::string_view name, std::string_view json);

    // Returns false if id doesn't exist.
    bool activate(int64_t id);

    // Throws ActiveLayoutDeleteError when the row is active. Returns false
    // if id doesn't exist.
    bool remove(int64_t id);

    // Inserts + activates the bundled default layout when the table is
    // empty. Idempotent; returns true when a row was inserted.
    bool seed_if_empty(std::string_view name, std::string_view json);

private:
    Database& db_;
};

}  // namespace gw::server
