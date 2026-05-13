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
    // Live-tunable settings. Each unset (nullopt) means "use camera default" —
    // we don't touch the corresponding GenICam node at start.
    std::optional<bool>        gain_auto;
    std::optional<double>      gain;
    std::optional<bool>        exposure_auto;
    std::optional<double>      exposure;
    int64_t                    created_at = 0;  // unix seconds
};

// Partial update payload. All fields are optional; an entirely-empty struct is
// a no-op. The repository preserves any field left at nullopt.
struct CameraUpdate {
    std::optional<std::string> name;
    std::optional<std::string> mode;
    std::optional<bool>        gain_auto;
    std::optional<double>      gain;
    std::optional<bool>        exposure_auto;
    std::optional<double>      exposure;

    bool empty() const {
        return !name && !mode && !gain_auto && !gain && !exposure_auto && !exposure;
    }
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

    // Partial update. Any field of CameraUpdate may be nullopt to leave it
    // unchanged; an entirely-empty patch returns the row as-is. Serial cannot
    // be changed. Returns the updated row, or std::nullopt if id does not
    // exist. Throws DuplicateNameError on UNIQUE conflict.
    std::optional<Camera> update(int64_t id, const CameraUpdate& patch);

    // Returns true if a row was deleted.
    bool                  remove(int64_t id);

private:
    Database& db_;
};

}  // namespace gw::server
