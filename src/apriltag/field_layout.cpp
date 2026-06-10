#include "apriltag/field_layout.hpp"

#include <yaml-cpp/yaml.h>

namespace gw::apriltag {

namespace {

double get_num(const YAML::Node& n, const char* where, const char* key) {
    const auto child = n[key];
    if (!child) {
        throw FieldLayoutParseError(std::string("field layout: ") + where +
                                    " missing key " + key);
    }
    try {
        return child.as<double>();
    } catch (const YAML::Exception&) {
        throw FieldLayoutParseError(std::string("field layout: ") + where + "." +
                                    key + " is not a number");
    }
}

}  // namespace

FieldLayout parse_field_layout_json(const std::string& text) {
    YAML::Node root;
    try {
        root = YAML::Load(text);
    } catch (const YAML::Exception& e) {
        throw FieldLayoutParseError(std::string("field layout: invalid JSON: ") +
                                    e.what());
    }
    if (!root.IsMap()) {
        throw FieldLayoutParseError("field layout: document is not an object");
    }

    FieldLayout out;

    const auto field = root["field"];
    if (!field || !field.IsMap()) {
        throw FieldLayoutParseError("field layout: missing object key: field");
    }
    out.length_m = get_num(field, "field", "length");
    out.width_m  = get_num(field, "field", "width");
    if (out.length_m <= 0 || out.width_m <= 0) {
        throw FieldLayoutParseError("field layout: field dimensions must be positive");
    }

    const auto tags = root["tags"];
    if (!tags || !tags.IsSequence() || tags.size() == 0) {
        throw FieldLayoutParseError("field layout: tags must be a non-empty array");
    }
    out.tags.reserve(tags.size());
    for (const auto& t : tags) {
        FieldLayoutTag tag;
        const auto id_node = t["ID"];
        if (!id_node) throw FieldLayoutParseError("field layout: tag missing ID");
        try {
            tag.id = id_node.as<int>();
        } catch (const YAML::Exception&) {
            throw FieldLayoutParseError("field layout: tag ID is not an integer");
        }

        const auto pose = t["pose"];
        if (!pose || !pose.IsMap()) {
            throw FieldLayoutParseError("field layout: tag " + std::to_string(tag.id) +
                                        " missing pose");
        }
        const auto tr = pose["translation"];
        if (!tr) {
            throw FieldLayoutParseError("field layout: tag " + std::to_string(tag.id) +
                                        " missing translation");
        }
        const auto rot = pose["rotation"];
        const auto q   = rot ? rot["quaternion"] : YAML::Node();
        if (!q) {
            throw FieldLayoutParseError("field layout: tag " + std::to_string(tag.id) +
                                        " missing rotation.quaternion");
        }

        const std::string where = "tag " + std::to_string(tag.id);
        const Mat3 R = quat_wxyz_to_mat3(
            get_num(q, where.c_str(), "W"), get_num(q, where.c_str(), "X"),
            get_num(q, where.c_str(), "Y"), get_num(q, where.c_str(), "Z"));
        tag.T_field_tag = mat4_from_rt(
            R, {get_num(tr, where.c_str(), "x"), get_num(tr, where.c_str(), "y"),
                get_num(tr, where.c_str(), "z")});
        out.tags.push_back(tag);
    }
    return out;
}

TagCorners tag_corners_field(const Mat4& T_field_tag, double tag_size_m) {
    const double h = tag_size_m / 2.0;
    // WPILib tag frame: +X out of the face, +Y viewer's right, +Z up.
    // Order matches det->p[0..3] = BL, BR, TR, TL.
    const std::array<std::array<double, 3>, 4> local = {{
        {0.0, -h, -h},  // bottom-left
        {0.0, +h, -h},  // bottom-right
        {0.0, +h, +h},  // top-right
        {0.0, -h, +h},  // top-left
    }};
    TagCorners out{};
    for (int i = 0; i < 4; ++i) {
        for (int r = 0; r < 3; ++r) {
            out[i][r] = T_field_tag[r][0] * local[i][0] +
                        T_field_tag[r][1] * local[i][1] +
                        T_field_tag[r][2] * local[i][2] + T_field_tag[r][3];
        }
    }
    return out;
}

PreparedLayout prepare_layout(const FieldLayout& layout, double tag_size_m) {
    PreparedLayout out;
    out.tag_size_m = tag_size_m;
    out.by_id.reserve(layout.tags.size());
    out.corners.reserve(layout.tags.size());
    for (const auto& t : layout.tags) {
        out.by_id[t.id]   = t;
        out.corners[t.id] = tag_corners_field(t.T_field_tag, tag_size_m);
    }
    return out;
}

}  // namespace gw::apriltag
