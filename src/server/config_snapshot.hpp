#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <crow.h>

namespace gw::server {

class CameraRepository;
class CanConfigRepository;
class FieldLayoutRepository;
class FusionConfigRepository;
class ImuConfigRepository;
class TriggerGroupRepository;
class VioConfigRepository;

// Full-robot configuration snapshot (Crow JSON types, no HTTP — the route
// layer in routes_config.cpp is a thin wrapper, so import/export logic is
// unit-testable against temp databases).
//
// Snapshot v1 carries: cameras (every row field except id/created_at —
// calibration + camera-IMU extrinsics blobs as VERBATIM strings, no
// re-serialization drift), trigger groups, field layouts (+ which is
// active), and the four single-row tunable configs.
//
// Import is a NON-DESTRUCTIVE MERGE: cameras match by serial, groups and
// layouts by name; local rows absent from the snapshot are left alone; blob
// fields are only set when present (never cleared by absence). Every per-row
// failure is recorded in the report and skipped — only a version mismatch /
// non-object root throws. The caller is responsible for post-import
// propagation (CameraSupervisor notifications, supervisor reloads, CAN mode
// push) — see routes_config.cpp.

inline constexpr int kSnapshotVersion = 1;

struct SectionReport {
    int                      created = 0;
    int                      updated = 0;
    std::vector<std::string> errors;
};

struct ImportReport {
    SectionReport cameras, trigger_groups, field_layouts;
    SectionReport imu_config, vio_config, can_config, fusion_config;

    // For CameraSupervisor::on_camera_added / on_camera_updated.
    std::vector<int64_t> camera_ids_created;
    std::vector<int64_t> camera_ids_updated;

    bool ok() const {
        for (const SectionReport* s :
             {&cameras, &trigger_groups, &field_layouts, &imu_config,
              &vio_config, &can_config, &fusion_config}) {
            if (!s->errors.empty()) return false;
        }
        return true;
    }
    crow::json::wvalue to_json() const;
};

crow::json::wvalue export_snapshot(CameraRepository&       cameras,
                                   TriggerGroupRepository& trigger_groups,
                                   FieldLayoutRepository&  field_layouts,
                                   ImuConfigRepository&    imu_config,
                                   VioConfigRepository&    vio_config,
                                   CanConfigRepository&    can_config,
                                   FusionConfigRepository& fusion_config);

// Throws std::runtime_error on snapshot_version mismatch or non-object root.
ImportReport import_snapshot(const crow::json::rvalue& snap,
                             CameraRepository&         cameras,
                             TriggerGroupRepository&   trigger_groups,
                             FieldLayoutRepository&    field_layouts,
                             ImuConfigRepository&      imu_config,
                             VioConfigRepository&      vio_config,
                             CanConfigRepository&      can_config,
                             FusionConfigRepository&   fusion_config);

}  // namespace gw::server
