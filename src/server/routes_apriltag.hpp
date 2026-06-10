#pragma once

#include <crow.h>

namespace gw::server {

class ApriltagSupervisor;
class CameraSupervisor;
class FieldLayoutRepository;

// /api/field-layouts        GET, POST
// /api/field-layouts/<id>   GET, DELETE, POST <id>/activate
// /api/apriltag/status      GET
void register_apriltag_routes(crow::SimpleApp&       app,
                              FieldLayoutRepository& field_layouts,
                              ApriltagSupervisor&    apriltag,
                              CameraSupervisor&      supervisor);

// Seeds the bundled season layout (GW_FIELD_LAYOUT_DEFAULT) into an empty
// field_layouts table so the pipeline boots headless with a usable layout.
// Validates the file before inserting; logs and continues on failure.
void seed_default_field_layout(FieldLayoutRepository& field_layouts);

}  // namespace gw::server
