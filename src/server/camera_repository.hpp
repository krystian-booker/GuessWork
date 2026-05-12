#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace gw::server {

class Database;

struct Camera {
    int64_t     id         = 0;
    std::string name;
    int64_t     created_at = 0;  // unix seconds
};

// Thrown when a write violates the UNIQUE(name) constraint. Route layer maps
// this to HTTP 409 Conflict.
class DuplicateNameError : public std::runtime_error {
public:
    explicit DuplicateNameError(const std::string& name)
        : std::runtime_error("camera name already exists: " + name) {}
};

class CameraRepository {
public:
    explicit CameraRepository(Database& db) : db_(db) {}

    std::vector<Camera>   list_all();
    std::optional<Camera> get(int64_t id);

    // Throws DuplicateNameError if name is already taken.
    Camera                create(std::string_view name);

    // Returns the updated row, or std::nullopt if id does not exist.
    // Throws DuplicateNameError on UNIQUE conflict.
    std::optional<Camera> update(int64_t id, std::string_view name);

    // Returns true if a row was deleted.
    bool                  remove(int64_t id);

private:
    Database& db_;
};

}  // namespace gw::server
