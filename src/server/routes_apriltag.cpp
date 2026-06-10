#include "server/routes_apriltag.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <unordered_map>

#include "apriltag/field_layout.hpp"
#include "server/apriltag_supervisor.hpp"
#include "server/camera_supervisor.hpp"
#include "server/field_layout_repository.hpp"
#include "server/route_helpers.hpp"

#ifndef GW_FIELD_LAYOUT_DEFAULT
#define GW_FIELD_LAYOUT_DEFAULT ""
#endif

namespace gw::server {

namespace {

// Summary row: parse lazily for tag_count / field dims; tolerate stored
// rows that no longer parse (surface nulls instead of failing the list).
crow::json::wvalue layout_summary_to_json(const FieldLayoutRow& r) {
    crow::json::wvalue j;
    j["id"]         = r.id;
    j["name"]       = r.name;
    j["active"]     = r.active;
    j["created_at"] = r.created_at;
    try {
        const auto layout = gw::apriltag::parse_field_layout_json(r.json);
        j["tag_count"]      = static_cast<int>(layout.tags.size());
        j["field_length_m"] = layout.length_m;
        j["field_width_m"]  = layout.width_m;
    } catch (...) {
        j["tag_count"]      = nullptr;
        j["field_length_m"] = nullptr;
        j["field_width_m"]  = nullptr;
    }
    return j;
}

crow::json::wvalue mat4_to_json(const gw::apriltag::Mat4& T) {
    crow::json::wvalue::list rows;
    for (int r = 0; r < 4; ++r) {
        crow::json::wvalue::list row;
        for (int c = 0; c < 4; ++c) row.emplace_back(T[r][c]);
        rows.emplace_back(std::move(row));
    }
    return crow::json::wvalue(std::move(rows));
}

crow::json::wvalue camera_entry_to_json(const ApriltagStatus::CameraEntry& e,
                                        bool online) {
    crow::json::wvalue j;
    j["camera_id"] = e.camera_id;
    j["name"]      = e.name;
    j["running"]   = e.running;
    j["reason"]    = (!online && !e.running) ? "offline" : e.reason;
    if (!e.running) return j;

    const auto& s = e.stats;
    j["det_per_s"]             = s.det_per_s;
    j["frames_seen"]           = s.frames_seen;
    j["detections_total"]      = s.detections_total;
    j["published"]             = s.published;
    j["skipped_no_tags"]       = s.skipped_no_tags;
    j["skipped_ambiguous"]     = s.skipped_ambiguous;
    j["skipped_high_reproj"]   = s.skipped_high_reproj;
    j["skipped_no_extrinsics"] = s.skipped_no_extrinsics;
    j["skipped_solve_failed"]  = s.skipped_solve_failed;
    j["last_latency_ms"]       = s.last_latency_ms;
    j["latency_ewma_ms"]       = s.latency_ewma_ms;
    j["mean_reproj_err_px"]    = s.mean_reproj_err_px;

    crow::json::wvalue::list tags;
    for (const auto& t : s.last_tags) {
        crow::json::wvalue tj;
        tj["id"]              = t.id;
        tj["decision_margin"] = t.decision_margin;
        if (t.range_m) tj["range_m"] = *t.range_m;
        else           tj["range_m"] = nullptr;
        tags.emplace_back(std::move(tj));
    }
    j["last_tags"] = std::move(tags);

    if (s.last_pose) {
        j["last_pose"]      = mat4_to_json(*s.last_pose);
        j["last_pose_t_ns"] = s.last_pose_t_ns;
    } else {
        j["last_pose"]      = nullptr;
        j["last_pose_t_ns"] = nullptr;
    }
    return j;
}

}  // namespace

void seed_default_field_layout(FieldLayoutRepository& field_layouts) {
    const std::filesystem::path path = GW_FIELD_LAYOUT_DEFAULT;
    if (path.empty()) return;
    try {
        std::ifstream in(path, std::ios::binary);
        if (!in.is_open()) {
            std::cerr << "field layout seed: cannot open " << path << "\n";
            return;
        }
        const std::string json((std::istreambuf_iterator<char>(in)),
                                std::istreambuf_iterator<char>());
        gw::apriltag::parse_field_layout_json(json);  // validate before insert
        if (field_layouts.seed_if_empty(path.stem().string(), json)) {
            std::cerr << "field layout seed: activated '" << path.stem().string()
                      << "'\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "field layout seed failed: " << e.what() << "\n";
    }
}

void register_apriltag_routes(crow::SimpleApp&       app,
                              FieldLayoutRepository& field_layouts,
                              ApriltagSupervisor&    apriltag,
                              CameraSupervisor&      supervisor) {
    CROW_ROUTE(app, "/api/field-layouts").methods("GET"_method)
    ([&field_layouts] {
        try {
            crow::json::wvalue::list items;
            for (const auto& r : field_layouts.list_all()) {
                items.emplace_back(layout_summary_to_json(r));
            }
            return json_response(200, crow::json::wvalue(std::move(items)));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/field-layouts/<int>").methods("GET"_method)
    ([&field_layouts](int64_t id) {
        try {
            const auto r = field_layouts.get(id);
            if (!r) return error_response(404, "field layout not found");
            auto j     = layout_summary_to_json(*r);
            j["json"]  = r->json;
            return json_response(200, std::move(j));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/field-layouts").methods("POST"_method)
    ([&field_layouts](const crow::request& req) {
        const auto body = crow::json::load(req.body);
        if (!body) return error_response(400, "invalid JSON body");
        if (!body.has("name") || body["name"].t() != crow::json::type::String) {
            return error_response(400, "missing string field: name");
        }
        if (!body.has("layout") || body["layout"].t() != crow::json::type::Object) {
            return error_response(400, "missing object field: layout");
        }
        const std::string name(body["name"].s());
        if (name.empty()) return error_response(400, "name must be non-empty");
        const std::string layout_json = crow::json::wvalue(body["layout"]).dump();

        try {
            gw::apriltag::parse_field_layout_json(layout_json);
        } catch (const std::exception& e) {
            return error_response(400, std::string("invalid field layout: ") + e.what());
        }
        try {
            const auto r = field_layouts.create(name, layout_json);
            return json_response(201, layout_summary_to_json(r));
        } catch (const DuplicateLayoutNameError& e) {
            return error_response(409, e.what());
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/field-layouts/<int>/activate").methods("POST"_method)
    ([&field_layouts, &apriltag](int64_t id) {
        try {
            if (!field_layouts.activate(id)) {
                return error_response(404, "field layout not found");
            }
            apriltag.reload_shared();
            const auto r = field_layouts.get(id);
            return json_response(200, layout_summary_to_json(*r));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/field-layouts/<int>").methods("DELETE"_method)
    ([&field_layouts](int64_t id) {
        try {
            if (!field_layouts.remove(id)) {
                return error_response(404, "field layout not found");
            }
            return with_no_store(crow::response(204));
        } catch (const ActiveLayoutDeleteError& e) {
            return error_response(409, e.what());
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/apriltag/status").methods("GET"_method)
    ([&apriltag, &supervisor] {
        try {
            const auto st = apriltag.status();

            // Compose per-camera online state separately — ApriltagSupervisor
            // never touches CameraSupervisor (lock-ordering contract).
            std::unordered_map<int64_t, bool> online;
            for (const auto& c : supervisor.snapshot_all()) online[c.id] = c.online;

            crow::json::wvalue j;
            put_opt(j, "active_layout_id",   st.active_layout_id);
            put_opt(j, "active_layout_name", st.active_layout_name);
            j["t_robot_imu_set"] = st.t_robot_imu_set;
            crow::json::wvalue::list cams;
            for (const auto& e : st.cameras) {
                const auto it = online.find(e.camera_id);
                cams.emplace_back(camera_entry_to_json(
                    e, it != online.end() && it->second));
            }
            j["cameras"] = std::move(cams);
            return json_response(200, std::move(j));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });
}

}  // namespace gw::server
