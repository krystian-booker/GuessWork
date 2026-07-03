#include "server/trigger_group_repository.hpp"

#include <sqlite3.h>

#include <algorithm>
#include <set>
#include <stdexcept>
#include <string>

#include "server/database.hpp"

namespace gw::server {

namespace {

struct StmtGuard {
    sqlite3_stmt* stmt = nullptr;
    ~StmtGuard() { if (stmt) sqlite3_finalize(stmt); }
};

[[noreturn]] void throw_sqlite(sqlite3* h, const char* what) {
    throw std::runtime_error(std::string(what) + ": " + sqlite3_errmsg(h));
}

constexpr const char* kSelectColumns =
    "id, name, fps, outputs_bitmask, created_at";

TriggerGroup read_row(sqlite3_stmt* stmt) {
    TriggerGroup g;
    g.id   = sqlite3_column_int64(stmt, 0);
    const auto* name_text = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
    g.name = name_text ? name_text : "";
    g.fps  = sqlite3_column_double(stmt, 2);
    g.output_pins = bitmask_to_pins(
        static_cast<uint8_t>(sqlite3_column_int(stmt, 3) & 0x3F));
    g.created_at  = sqlite3_column_int64(stmt, 4);
    return g;
}

// Walks all rows except `exclude_id` and aborts if any of their bitmasks
// overlap `mask`. Pass exclude_id = 0 for create().
void check_no_pin_overlap(sqlite3* h, uint8_t mask, int64_t exclude_id) {
    StmtGuard g;
    if (sqlite3_prepare_v2(h,
            "SELECT id, outputs_bitmask FROM trigger_groups WHERE id != ?;",
            -1, &g.stmt, nullptr) != SQLITE_OK) {
        throw_sqlite(h, "check_no_pin_overlap: prepare");
    }
    sqlite3_bind_int64(g.stmt, 1, exclude_id);
    for (;;) {
        const int rc = sqlite3_step(g.stmt);
        if (rc == SQLITE_DONE) break;
        if (rc != SQLITE_ROW) throw_sqlite(h, "check_no_pin_overlap: step");
        const uint8_t other = static_cast<uint8_t>(sqlite3_column_int(g.stmt, 1) & 0x3F);
        const uint8_t conflict = mask & other;
        if (conflict) {
            for (int pin = 1; pin <= 6; ++pin) {
                if (conflict & (1u << (pin - 1))) {
                    throw TriggerOutputPinConflictError(pin);
                }
            }
        }
    }
}

bool is_unique_violation(int rc) {
    return rc == SQLITE_CONSTRAINT_UNIQUE || rc == SQLITE_CONSTRAINT;
}

}  // namespace

std::vector<uint8_t> normalize_output_pins(const std::vector<uint8_t>& pins) {
    if (pins.empty()) {
        throw InvalidTriggerOutputsError("must include at least one pin");
    }
    std::set<int> uniq;
    for (auto p : pins) {
        if (p < 1 || p > 6) {
            throw InvalidTriggerOutputsError(
                "pin " + std::to_string(p) + " out of range (1..6)");
        }
        if (!uniq.insert(p).second) {
            throw InvalidTriggerOutputsError(
                "duplicate pin " + std::to_string(p));
        }
    }
    std::vector<uint8_t> sorted(uniq.begin(), uniq.end());
    std::sort(sorted.begin(), sorted.end());
    return sorted;
}

uint8_t pins_to_bitmask(const std::vector<uint8_t>& pins) {
    uint8_t mask = 0;
    for (auto p : pins) mask = static_cast<uint8_t>(mask | (1u << (p - 1)));
    return mask;
}

std::vector<uint8_t> bitmask_to_pins(uint8_t mask) {
    std::vector<uint8_t> out;
    for (int i = 0; i < 6; ++i) {
        if (mask & (1u << i)) out.push_back(static_cast<uint8_t>(i + 1));
    }
    return out;
}

std::vector<TriggerGroup> TriggerGroupRepository::list_all() {
    return db_.with_handle([](sqlite3* h) {
        StmtGuard g;
        const std::string sql =
            std::string("SELECT ") + kSelectColumns +
            " FROM trigger_groups ORDER BY id;";
        if (sqlite3_prepare_v2(h, sql.c_str(), -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "list_all: prepare");
        }
        std::vector<TriggerGroup> out;
        for (;;) {
            const int rc = sqlite3_step(g.stmt);
            if (rc == SQLITE_DONE) break;
            if (rc != SQLITE_ROW) throw_sqlite(h, "list_all: step");
            out.push_back(read_row(g.stmt));
        }
        return out;
    });
}

std::optional<TriggerGroup> TriggerGroupRepository::get(int64_t id) {
    return db_.with_handle([id](sqlite3* h) -> std::optional<TriggerGroup> {
        StmtGuard g;
        const std::string sql =
            std::string("SELECT ") + kSelectColumns +
            " FROM trigger_groups WHERE id = ?;";
        if (sqlite3_prepare_v2(h, sql.c_str(), -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "get: prepare");
        }
        sqlite3_bind_int64(g.stmt, 1, id);
        const int rc = sqlite3_step(g.stmt);
        if (rc == SQLITE_ROW) return read_row(g.stmt);
        if (rc == SQLITE_DONE) return std::nullopt;
        throw_sqlite(h, "get: step");
    });
}

TriggerGroup TriggerGroupRepository::create(std::string_view            name,
                                            double                      fps,
                                            const std::vector<uint8_t>& pins_in) {
    if (!(fps > 0.0)) throw InvalidTriggerOutputsError("fps must be > 0");
    const std::string         name_str(name);
    const auto                pins = normalize_output_pins(pins_in);
    const uint8_t             mask = pins_to_bitmask(pins);

    return db_.with_handle([&](sqlite3* h) {
        check_no_pin_overlap(h, mask, /*exclude_id=*/0);

        StmtGuard g;
        const std::string sql =
            std::string("INSERT INTO trigger_groups (name, fps, outputs_bitmask) "
                        "VALUES (?, ?, ?) RETURNING ") + kSelectColumns + ";";
        if (sqlite3_prepare_v2(h, sql.c_str(), -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "create: prepare");
        }
        sqlite3_bind_text  (g.stmt, 1, name_str.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_double(g.stmt, 2, fps);
        sqlite3_bind_int   (g.stmt, 3, mask);

        const int rc = sqlite3_step(g.stmt);
        if (rc == SQLITE_ROW) return read_row(g.stmt);
        if (is_unique_violation(rc)) throw DuplicateTriggerGroupNameError(name_str);
        throw_sqlite(h, "create: step");
    });
}

std::optional<TriggerGroup> TriggerGroupRepository::update(int64_t                   id,
                                                           const TriggerGroupUpdate& patch) {
    if (patch.empty()) return get(id);
    if (patch.fps && !(*patch.fps > 0.0)) {
        throw InvalidTriggerOutputsError("fps must be > 0");
    }
    std::optional<std::vector<uint8_t>> normalized;
    std::optional<uint8_t>              mask;
    if (patch.output_pins) {
        normalized = normalize_output_pins(*patch.output_pins);
        mask       = pins_to_bitmask(*normalized);
    }

    return db_.with_handle([&](sqlite3* h) -> std::optional<TriggerGroup> {
        if (mask) check_no_pin_overlap(h, *mask, id);

        StmtGuard g;
        std::string sql   = "UPDATE trigger_groups SET ";
        bool        first = true;
        auto add_col = [&](const char* expr) {
            if (!first) sql += ", ";
            sql += expr;
            first = false;
        };
        if (patch.name)        add_col("name = ?");
        if (patch.fps)         add_col("fps = ?");
        if (patch.output_pins) add_col("outputs_bitmask = ?");
        sql += " WHERE id = ? RETURNING ";
        sql += kSelectColumns;
        sql += ";";

        if (sqlite3_prepare_v2(h, sql.c_str(), -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "update: prepare");
        }
        int idx = 1;
        if (patch.name) sqlite3_bind_text(g.stmt, idx++, patch.name->c_str(),
                                          -1, SQLITE_TRANSIENT);
        if (patch.fps)  sqlite3_bind_double(g.stmt, idx++, *patch.fps);
        if (mask)       sqlite3_bind_int   (g.stmt, idx++, *mask);
        sqlite3_bind_int64(g.stmt, idx, id);

        const int rc = sqlite3_step(g.stmt);
        if (rc == SQLITE_ROW)  return read_row(g.stmt);
        if (rc == SQLITE_DONE) return std::nullopt;
        if (is_unique_violation(rc) && patch.name) {
            throw DuplicateTriggerGroupNameError(*patch.name);
        }
        throw_sqlite(h, "update: step");
    });
}

bool TriggerGroupRepository::remove(int64_t id) {
    return db_.with_handle([id](sqlite3* h) {
        StmtGuard g;
        if (sqlite3_prepare_v2(h,
                "DELETE FROM trigger_groups WHERE id = ?;",
                -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "remove: prepare");
        }
        sqlite3_bind_int64(g.stmt, 1, id);
        const int rc = sqlite3_step(g.stmt);
        if (rc != SQLITE_DONE) throw_sqlite(h, "remove: step");
        return sqlite3_changes(h) > 0;
    });
}

bool TriggerGroupRepository::armed() {
    return db_.with_handle([](sqlite3* h) {
        StmtGuard g;
        if (sqlite3_prepare_v2(h, "SELECT armed FROM sync_config WHERE id = 1;",
                               -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "sync_config armed: prepare");
        }
        if (sqlite3_step(g.stmt) != SQLITE_ROW) {
            throw std::runtime_error("sync_config row missing (schema seed failed?)");
        }
        return sqlite3_column_int64(g.stmt, 0) != 0;
    });
}

void TriggerGroupRepository::set_armed(bool armed) {
    db_.with_handle([armed](sqlite3* h) {
        StmtGuard g;
        if (sqlite3_prepare_v2(h, "UPDATE sync_config SET armed = ?1 WHERE id = 1;",
                               -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "sync_config set_armed: prepare");
        }
        sqlite3_bind_int64(g.stmt, 1, armed ? 1 : 0);
        if (sqlite3_step(g.stmt) != SQLITE_DONE) {
            throw_sqlite(h, "sync_config set_armed: step");
        }
        return 0;
    });
}

}  // namespace gw::server
