#include <gtest/gtest.h>

#include <fstream>
#include <iterator>
#include <string>

#include "apriltag/field_layout.hpp"

namespace gw::apriltag {

#ifdef GW_FIELD_LAYOUT_DEFAULT
namespace {

std::string read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    EXPECT_TRUE(in.is_open()) << "missing fixture: " << path;
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

}  // namespace

TEST(FieldLayoutTest, ParsesShippedSeasonLayout) {
    const auto layout = parse_field_layout_json(read_file(GW_FIELD_LAYOUT_DEFAULT));
    EXPECT_EQ(layout.tags.size(), 32u);  // 2026 REBUILT
    EXPECT_NEAR(layout.length_m, 16.541, 1e-6);
    EXPECT_NEAR(layout.width_m, 8.069, 1e-6);

    // Spot-check tag 1 against the raw JSON numbers.
    const auto* tag1 = [&]() -> const FieldLayoutTag* {
        for (const auto& t : layout.tags)
            if (t.id == 1) return &t;
        return nullptr;
    }();
    ASSERT_NE(tag1, nullptr);
    EXPECT_NEAR(tag1->T_field_tag[0][3], 11.8779798, 1e-6);
    EXPECT_NEAR(tag1->T_field_tag[1][3], 7.4247756, 1e-6);
    EXPECT_NEAR(tag1->T_field_tag[2][3], 0.889, 1e-6);

    // Every rotation block must be orthonormal (quaternion parse sanity).
    for (const auto& t : layout.tags) {
        const auto& T = t.T_field_tag;
        for (int i = 0; i < 3; ++i) {
            double n = 0;
            for (int r = 0; r < 3; ++r) n += T[r][i] * T[r][i];
            EXPECT_NEAR(n, 1.0, 1e-9) << "tag " << t.id;
        }
    }

    const auto prepared = prepare_layout(layout);
    EXPECT_EQ(prepared.by_id.size(), 32u);
    EXPECT_EQ(prepared.corners.size(), 32u);
    EXPECT_DOUBLE_EQ(prepared.tag_size_m, kFrcTagSizeM);
}
#endif

TEST(FieldLayoutTest, IdentityPoseCornerTable) {
    // A tag at the field origin with zero rotation: corners must land at the
    // normative table — BL (0,−h,−h), BR (0,+h,−h), TR (0,+h,+h), TL (0,−h,+h).
    const double s = 0.2;
    const double h = s / 2;
    const auto   c = tag_corners_field(mat4_identity(), s);
    const double expect[4][3] = {
        {0, -h, -h}, {0, +h, -h}, {0, +h, +h}, {0, -h, +h}};
    for (int i = 0; i < 4; ++i)
        for (int k = 0; k < 3; ++k) EXPECT_NEAR(c[i][k], expect[i][k], 1e-12)
            << "corner " << i << " axis " << k;
}

TEST(FieldLayoutTest, TranslatedRotatedCorners) {
    // Tag rotated 180° about Z (facing −X, i.e. toward blue) at (4, 2, 1):
    // the outward normal flips to −X and the corner Y offsets mirror.
    const Mat3 R = quat_wxyz_to_mat3(0, 0, 0, 1);
    const Mat4 T = mat4_from_rt(R, {4, 2, 1});
    const auto c = tag_corners_field(T, 0.2);
    // BL: local (0,−0.1,−0.1) → field (4, 2.1, 0.9)
    EXPECT_NEAR(c[0][0], 4.0, 1e-12);
    EXPECT_NEAR(c[0][1], 2.1, 1e-12);
    EXPECT_NEAR(c[0][2], 0.9, 1e-12);
    // TR: local (0,+0.1,+0.1) → field (4, 1.9, 1.1)
    EXPECT_NEAR(c[2][1], 1.9, 1e-12);
    EXPECT_NEAR(c[2][2], 1.1, 1e-12);
}

TEST(FieldLayoutTest, MalformedInputsThrow) {
    EXPECT_THROW(parse_field_layout_json("not json ["), FieldLayoutParseError);
    EXPECT_THROW(parse_field_layout_json("[]"), FieldLayoutParseError);
    EXPECT_THROW(parse_field_layout_json(R"({"tags": []})"), FieldLayoutParseError);
    EXPECT_THROW(parse_field_layout_json(R"({"field": {"length": 1, "width": 1}, "tags": []})"),
                 FieldLayoutParseError);
    // Tag missing pose.
    EXPECT_THROW(parse_field_layout_json(
                     R"({"field": {"length": 1, "width": 1}, "tags": [{"ID": 1}]})"),
                 FieldLayoutParseError);
    // Negative field dims.
    EXPECT_THROW(parse_field_layout_json(
                     R"({"field": {"length": -1, "width": 1}, "tags": [{"ID": 1}]})"),
                 FieldLayoutParseError);
}

}  // namespace gw::apriltag
