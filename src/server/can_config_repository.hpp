#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace gw::server {

class Database;

// The single can_config row. mode: 'off' | 'roborio' (classic CAN 2.0 @
// 1 Mbps) | 'systemcore' (CAN FD 1M/4M). The route layer pushes the matching
// CAN_MODE command to the Teensy on update; TeensyManager re-pushes it on
// every reconnect.
struct CanConfig {
    std::string mode       = "off";
    int64_t     updated_at = 0;  // unix seconds
};

// Partial update. nullopt fields stay unchanged.
struct CanConfigUpdate {
    std::optional<std::string> mode;

    bool empty() const { return !mode; }
};

// Accessor for the single-row can_config table (seeded at schema creation).
// Column CHECK violations surface as std::runtime_error from update().
class CanConfigRepository {
public:
    explicit CanConfigRepository(Database& db) : db_(db) {}

    CanConfig get();
    CanConfig update(const CanConfigUpdate& patch);

private:
    Database& db_;
};

}  // namespace gw::server
