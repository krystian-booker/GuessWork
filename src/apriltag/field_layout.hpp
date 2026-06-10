#pragma once

#include <array>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "apriltag/pose_math.hpp"

// WPILib AprilTagFieldLayout handling.
//
// JSON shape (verbatim from allwpilib):
//   {"tags":[{"ID": n, "pose": {"translation": {"x","y","z"},
//             "rotation": {"quaternion": {"W","X","Y","Z"}}}}, …],
//    "field": {"length": m, "width": m}}
//
// Frames (normative — see also tag_pose_estimator.cpp's convention block):
//   field: FRC field frame — NWU, origin at the blue-alliance right corner,
//          +X toward red, +Z up, meters.
//   tag:   origin at the tag center; zero rotation = tag facing toward red,
//          so the tag's outward face normal is tag-frame +X and tag +Z is up
//          (+Y completes right-handed: a viewer facing the tag sees tag +Y
//          pointing to their right... derivation: with zero rotation tag
//          axes coincide with field axes).
//
// Parsed with yaml-cpp (JSON ⊂ YAML 1.2) so gw_apriltag stays Crow-free.

namespace gw::apriltag {

class FieldLayoutParseError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct FieldLayoutTag {
    int  id = 0;
    Mat4 T_field_tag = mat4_identity();
};

struct FieldLayout {
    double                      length_m = 0.0;
    double                      width_m  = 0.0;
    std::vector<FieldLayoutTag> tags;
};

FieldLayout parse_field_layout_json(const std::string& text);

// FRC 36h11 black-square edge: 6.5 in.
constexpr double kFrcTagSizeM = 0.1651;

// Field-frame positions of a tag's four corners, ordered to match the
// apriltag detector's det->p[0..3] = BL, BR, TR, TL (image sense, viewer
// facing the tag). In the WPILib tag frame the corners are
//   BL (0, −s/2, −s/2)   BR (0, +s/2, −s/2)
//   TR (0, +s/2, +s/2)   TL (0, −s/2, +s/2)
using TagCorners = std::array<std::array<double, 3>, 4>;
TagCorners tag_corners_field(const Mat4& T_field_tag, double tag_size_m);

// Layout indexed for the hot path.
struct PreparedLayout {
    double                                  tag_size_m = kFrcTagSizeM;
    std::unordered_map<int, FieldLayoutTag> by_id;
    std::unordered_map<int, TagCorners>     corners;  // field frame
};
PreparedLayout prepare_layout(const FieldLayout& layout,
                              double             tag_size_m = kFrcTagSizeM);

}  // namespace gw::apriltag
