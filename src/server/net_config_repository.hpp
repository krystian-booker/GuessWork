#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace gw::server {

class Database;

// The single net_config row — UDP robot-link settings (src/net/robot_link.hpp,
// docs/ethernet-protocol.md). robot_ip empty = learn the controller address
// from inbound chassis-speeds packets (the normal case; a static IP is a
// bench override).
struct NetConfig {
    bool        enabled    = true;
    int64_t     bind_port  = 5809;
    int64_t     robot_port = 5810;
    std::string robot_ip;  // "" = auto-learn
    int64_t     updated_at = 0;  // unix seconds
};

// Partial update. nullopt fields stay unchanged.
struct NetConfigUpdate {
    std::optional<bool>        enabled;
    std::optional<int64_t>     bind_port;
    std::optional<int64_t>     robot_port;
    std::optional<std::string> robot_ip;  // "" clears the static override

    bool empty() const {
        return !enabled && !bind_port && !robot_port && !robot_ip;
    }
};

// Accessor for the single-row net_config table (seeded at schema creation).
// Column CHECK violations surface as std::runtime_error from update().
class NetConfigRepository {
public:
    explicit NetConfigRepository(Database& db) : db_(db) {}

    NetConfig get();
    NetConfig update(const NetConfigUpdate& patch);

private:
    Database& db_;
};

}  // namespace gw::server
