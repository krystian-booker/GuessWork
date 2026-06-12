#include <gtest/gtest.h>

#include <optional>

#include "server/fusion_mode.hpp"

namespace gw::server {

namespace {
constexpr std::optional<int64_t> kFresh = 100;   // < 500 ms
constexpr std::optional<int64_t> kStale = 2000;  // ≥ 500 ms
constexpr std::optional<int64_t> kNever = std::nullopt;
}  // namespace

TEST(FusionModeTest, MatrixRows) {
    // initialized, tag, vio, odom, vio_enabled, collision → mode
    EXPECT_STREQ(derive_fusion_mode(false, kFresh, kFresh, kFresh, true, false),
                 "uninitialized");
    EXPECT_STREQ(derive_fusion_mode(true, kFresh, kFresh, kFresh, true, true),
                 "collision");
    EXPECT_STREQ(derive_fusion_mode(true, kFresh, kFresh, kFresh, true, false),
                 "nominal");
    EXPECT_STREQ(derive_fusion_mode(true, kFresh, kStale, kFresh, true, false),
                 "no_vio");
    EXPECT_STREQ(derive_fusion_mode(true, kFresh, kFresh, kStale, true, false),
                 "no_odom");
    EXPECT_STREQ(derive_fusion_mode(true, kFresh, kStale, kStale, true, false),
                 "tags_only");
    EXPECT_STREQ(derive_fusion_mode(true, kStale, kFresh, kFresh, true, false),
                 "dead_reckoning");
}

TEST(FusionModeTest, VioDisabledCountsAsNoVio) {
    // VIO fresh on the bus but T_robot_imu unset → ingestion disabled.
    EXPECT_STREQ(derive_fusion_mode(true, kFresh, kFresh, kFresh, false, false),
                 "no_vio");
    EXPECT_STREQ(derive_fusion_mode(true, kFresh, kFresh, kStale, false, false),
                 "tags_only");
}

TEST(FusionModeTest, NeverSeenIsStale) {
    EXPECT_STREQ(derive_fusion_mode(true, kNever, kFresh, kFresh, true, false),
                 "dead_reckoning");
    EXPECT_STREQ(derive_fusion_mode(true, kFresh, kNever, kNever, true, false),
                 "tags_only");
}

TEST(FusionModeTest, FreshnessBoundary) {
    EXPECT_STREQ(derive_fusion_mode(true, std::optional<int64_t>{499},
                                    kFresh, kFresh, true, false),
                 "nominal");
    EXPECT_STREQ(derive_fusion_mode(true, std::optional<int64_t>{500},
                                    kFresh, kFresh, true, false),
                 "dead_reckoning");
}

TEST(FusionModeTest, UninitializedBeatsCollision) {
    EXPECT_STREQ(derive_fusion_mode(false, kFresh, kFresh, kFresh, true, true),
                 "uninitialized");
}

}  // namespace gw::server
