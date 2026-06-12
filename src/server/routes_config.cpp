#include "server/routes_config.hpp"

#include <chrono>
#include <exception>
#include <string>

#include "server/apriltag_supervisor.hpp"
#include "server/camera_supervisor.hpp"
#include "server/can_config_repository.hpp"
#include "server/config_snapshot.hpp"
#include "server/fusion_supervisor.hpp"
#include "server/route_helpers.hpp"
#include "server/teensy_manager.hpp"
#include "server/vio_supervisor.hpp"

namespace gw::server {

void register_config_routes(crow::SimpleApp&        app,
                            CameraRepository&       cameras,
                            TriggerGroupRepository& trigger_groups,
                            FieldLayoutRepository&  field_layouts,
                            ImuConfigRepository&    imu_config,
                            VioConfigRepository&    vio_config,
                            CanConfigRepository&    can_config,
                            FusionConfigRepository& fusion_config,
                            CameraSupervisor&       supervisor,
                            ApriltagSupervisor&     apriltag,
                            VioSupervisor&          vio,
                            FusionSupervisor&       fusion,
                            TeensyManager&          teensy) {
    CROW_ROUTE(app, "/api/config/export").methods("GET"_method)
    ([&cameras, &trigger_groups, &field_layouts, &imu_config, &vio_config,
      &can_config, &fusion_config] {
        try {
            auto snap = export_snapshot(cameras, trigger_groups, field_layouts,
                                        imu_config, vio_config, can_config,
                                        fusion_config);
            auto res  = json_response(200, std::move(snap));
            const auto ts =
                std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count();
            res.add_header("Content-Disposition",
                           "attachment; filename=\"guesswork-config-" +
                               std::to_string(ts) + ".json\"");
            return res;
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/config/import").methods("POST"_method)
    ([&cameras, &trigger_groups, &field_layouts, &imu_config, &vio_config,
      &can_config, &fusion_config, &supervisor, &apriltag, &vio, &fusion,
      &teensy](const crow::request& req) {
        const auto body = crow::json::load(req.body);
        if (!body) return error_response(400, "invalid JSON body");

        ImportReport report;
        try {
            report = import_snapshot(body, cameras, trigger_groups,
                                     field_layouts, imu_config, vio_config,
                                     can_config, fusion_config);
        } catch (const std::exception& e) {
            return error_response(400, e.what());
        }

        // Propagate — repository writes alone don't take effect.
        for (const int64_t id : report.camera_ids_created) {
            supervisor.on_camera_added(id);
        }
        for (const int64_t id : report.camera_ids_updated) {
            supervisor.on_camera_updated(id);
        }
        apriltag.reload_shared();  // layouts and/or T_robot_imu may have changed
        vio.reload();
        fusion.reload();
        try {
            const std::string mode = can_config.get().mode;
            CanMode m = CanMode::Off;
            if (mode == "roborio")    m = CanMode::Classic;
            if (mode == "systemcore") m = CanMode::Fd;
            std::string ignored;
            teensy.set_can_mode(m, ignored);  // best-effort; resync re-pushes
        } catch (...) {}

        crow::json::wvalue j;
        j["ok"]     = report.ok();
        j["report"] = report.to_json();
        j["note"] =
            "trigger groups stored; re-arm via /api/hardware-sync to push "
            "them to the Teensy";
        return json_response(200, std::move(j));
    });
}

}  // namespace gw::server
