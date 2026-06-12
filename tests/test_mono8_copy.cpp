#include "core/mono8_copy.hpp"

#include <gtest/gtest.h>

#include <array>
#include <vector>

namespace {

TEST(Mono8CopyTest, StraightCopyTightStride) {
    const std::array<uint8_t, 6> src = {1, 2, 3, 4, 5, 6};  // 3x2
    std::array<uint8_t, 6> dst{};
    gw::copy_mono8(dst.data(), src.data(), 3, 3, 2, /*rotate_180=*/false);
    EXPECT_EQ(dst, src);
}

TEST(Mono8CopyTest, StraightCopyPaddedStride) {
    // 3x2 image with stride 5 (2 pad bytes per row, value 99).
    const std::array<uint8_t, 10> src = {1, 2, 3, 99, 99, 4, 5, 6, 99, 99};
    std::array<uint8_t, 6> dst{};
    gw::copy_mono8(dst.data(), src.data(), 5, 3, 2, /*rotate_180=*/false);
    const std::array<uint8_t, 6> want = {1, 2, 3, 4, 5, 6};
    EXPECT_EQ(dst, want);
}

TEST(Mono8CopyTest, Rotate180TightStride) {
    // 3x2:  1 2 3      rotated 180°:  6 5 4
    //       4 5 6                     3 2 1
    const std::array<uint8_t, 6> src = {1, 2, 3, 4, 5, 6};
    std::array<uint8_t, 6> dst{};
    gw::copy_mono8(dst.data(), src.data(), 3, 3, 2, /*rotate_180=*/true);
    const std::array<uint8_t, 6> want = {6, 5, 4, 3, 2, 1};
    EXPECT_EQ(dst, want);
}

TEST(Mono8CopyTest, Rotate180PaddedStride) {
    const std::array<uint8_t, 10> src = {1, 2, 3, 99, 99, 4, 5, 6, 99, 99};
    std::array<uint8_t, 6> dst{};
    gw::copy_mono8(dst.data(), src.data(), 5, 3, 2, /*rotate_180=*/true);
    const std::array<uint8_t, 6> want = {6, 5, 4, 3, 2, 1};
    EXPECT_EQ(dst, want);
}

TEST(Mono8CopyTest, Rotate180TwiceIsIdentity) {
    // Larger asymmetric image: double rotation must restore the original.
    constexpr uint32_t kW = 17, kH = 9;
    std::vector<uint8_t> src(kW * kH);
    for (size_t i = 0; i < src.size(); ++i) src[i] = static_cast<uint8_t>(i * 7 + 3);
    std::vector<uint8_t> once(kW * kH), twice(kW * kH);
    gw::copy_mono8(once.data(), src.data(), kW, kW, kH, true);
    gw::copy_mono8(twice.data(), once.data(), kW, kW, kH, true);
    EXPECT_EQ(twice, src);
}

}  // namespace
