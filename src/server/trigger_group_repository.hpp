#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace gw::server {

class Database;

// Persistent record for one sync controller trigger group. `output_pins` is the
// expanded representation of the DB-side bitmask: each entry is a 1..6
// physical output pin on the sync controller and entries are sorted ascending.
struct TriggerGroup {
    int64_t              id          = 0;
    std::string          name;
    double               fps         = 0.0;
    std::vector<uint8_t> output_pins;       // sorted, 1..6 each, no duplicates
    int64_t              created_at  = 0;   // unix seconds
};

// Partial update payload. nullopt fields stay unchanged. An entirely-empty
// patch is a no-op (returns the row as-is).
struct TriggerGroupUpdate {
    std::optional<std::string>          name;
    std::optional<double>               fps;
    std::optional<std::vector<uint8_t>> output_pins;

    bool empty() const { return !name && !fps && !output_pins; }
};

class DuplicateTriggerGroupNameError : public std::runtime_error {
public:
    explicit DuplicateTriggerGroupNameError(const std::string& name)
        : std::runtime_error("trigger group name already exists: " + name) {}
};

// Thrown when a write would claim a sync controller output that another group already
// owns. Maps to HTTP 409 Conflict.
class TriggerOutputPinConflictError : public std::runtime_error {
public:
    explicit TriggerOutputPinConflictError(int pin)
        : std::runtime_error("trigger output pin already in another group: " +
                             std::to_string(pin)) {}
};

// Thrown when output_pins is empty or contains values outside 1..6 (or
// duplicates within a single group). Maps to HTTP 400 Bad Request.
class InvalidTriggerOutputsError : public std::runtime_error {
public:
    explicit InvalidTriggerOutputsError(const std::string& detail)
        : std::runtime_error("invalid output_pins: " + detail) {}
};

class TriggerGroupRepository {
public:
    explicit TriggerGroupRepository(Database& db) : db_(db) {}

    std::vector<TriggerGroup>   list_all();
    std::optional<TriggerGroup> get(int64_t id);

    // Validates output_pins (non-empty, all in 1..6, no duplicates) and
    // checks that no other group already claims any of those pins. Throws
    // DuplicateTriggerGroupNameError, TriggerOutputPinConflictError, or
    // InvalidTriggerOutputsError on failure.
    TriggerGroup create(std::string_view              name,
                        double                        fps,
                        const std::vector<uint8_t>&   output_pins);

    // Partial update. Same validation rules as create(); when output_pins is
    // present its values are checked against every other group. Returns the
    // updated row, or std::nullopt if id does not exist.
    std::optional<TriggerGroup> update(int64_t id, const TriggerGroupUpdate& patch);

    bool                        remove(int64_t id);

    // Desired armed state (single-row sync_config table). This is operator
    // intent, not device state: the arm/stop routes set it BEFORE pushing so
    // a failed push (or a robot power-cycle) still converges — main.cpp
    // seeds SyncControllerManager's desired config from it at boot and the
    // reconnect/retry resync delivers it.
    bool armed();
    void set_armed(bool armed);

private:
    Database& db_;
};

// Helpers shared with the route layer / SyncControllerManager.
// Returns a sorted, deduplicated, range-checked copy of `pins`. Throws
// InvalidTriggerOutputsError on any failure.
std::vector<uint8_t> normalize_output_pins(const std::vector<uint8_t>& pins);
uint8_t              pins_to_bitmask(const std::vector<uint8_t>& pins);
std::vector<uint8_t> bitmask_to_pins(uint8_t mask);

}  // namespace gw::server
