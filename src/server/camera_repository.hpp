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
    int64_t                    id         = 0;
    std::string                name;
    std::string                serial;
    std::optional<std::string> mode;       // GenICam VideoMode symbolic, or unset
    int64_t                    created_at = 0;  // unix seconds
};

// Thrown when a write violates the UNIQUE(name) constraint. Route layer maps
// this to HTTP 409 Conflict.
class DuplicateNameError : public std::runtime_error {
public:
    explicit DuplicateNameError(const std::string& name)
        : std::runtime_error("camera name already exists: " + name) {}
};

// Thrown when an insert violates the UNIQUE index on `serial`. The route layer
// maps this to HTTP 409 Conflict.
class DuplicateSerialError : public std::runtime_error {
public:
    explicit DuplicateSerialError(const std::string& serial)
        : std::runtime_error("camera serial already mapped: " + serial) {}
};

class CameraRepository {
public:
    explicit CameraRepository(Database& db) : db_(db) {}

    std::vector<Camera>   list_all();
    std::optional<Camera> get(int64_t id);
    std::optional<Camera> find_by_serial(std::string_view serial);

    // Throws DuplicateNameError or DuplicateSerialError on UNIQUE conflict.
    Camera                create(std::string_view                name,
                                 std::string_view                serial,
                                 std::optional<std::string_view> mode = std::nullopt);

    // Partial update. Either field may be nullopt to leave it unchanged; an
    // entirely-nullopt call returns the row as-is. Serial cannot be changed.
    // Returns the updated row, or std::nullopt if id does not exist.
    // Throws DuplicateNameError on UNIQUE conflict.
    std::optional<Camera> update(int64_t                         id,
                                 std::optional<std::string_view> name,
                                 std::optional<std::string_view> mode);

    // Returns true if a row was deleted.
    bool                  remove(int64_t id);

private:
    Database& db_;
};

}  // namespace gw::server
