#pragma once

#include <crow.h>

namespace gw::server {

class ApriltagSupervisor;
class CameraRepository;
class CameraSupervisor;
class CanConfigRepository;
class FieldLayoutRepository;
class FusionConfigRepository;
class FusionSupervisor;
class ImuConfigRepository;
class TeensyManager;
class TriggerGroupRepository;
class VioConfigRepository;
class VioSupervisor;

// /api/config/export  GET   — full-robot configuration snapshot (cameras
//                             incl. calibration blobs, trigger groups, field
//                             layouts, all tunables) as a downloadable JSON.
// /api/config/import  POST  — non-destructive merge of a snapshot, then
//                             propagation: camera supervisor notifications,
//                             apriltag/vio/fusion reloads, CAN-mode push.
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
                            TeensyManager&          teensy);

}  // namespace gw::server
