#include <gtest/gtest.h>

#include <CoreVideo/CoreVideo.h>

#include <apriltag.h>
#include <tag36h11.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "apriltag/apriltag_consumer.hpp"
#include "apriltag/field_layout.hpp"
#include "core/frame.hpp"
#include "core/frame_channel.hpp"
#include "core/frame_pool.hpp"

// End-to-end consumer test: render a REAL tag36h11 bitmap into a Mono8
// CVPixelBuffer, publish it through a real FrameChannel, and assert a
// measurement appears on the bus. This validates the IOSurface stride
// wrapping, the detector wiring, and the corner-order conventions with the
// actual AprilRobotics decoder in the loop — no hardware needed.

namespace gw::apriltag {

namespace {

constexpr int      kImgW       = 640;
constexpr int      kImgH       = 480;
constexpr int      kTagId      = 7;
constexpr int      kPxPerCell  = 12;
constexpr double   kFx         = 600.0;
// tag36h11 bitmap is 10×10 cells including the 1-cell white border; the
// detected black square spans the inner 8 cells.
constexpr int      kBlackPx    = 8 * kPxPerCell;  // 96 px
const double       kExpectedRangeM = kFrcTagSizeM * kFx / kBlackPx;

constexpr FrameFormat kMono640x480{
    .width        = kImgW,
    .height       = kImgH,
    .pixel_format = kCVPixelFormatType_OneComponent8,
};

// Renders tag kTagId centered at the principal point.
void draw_tag_frame(Frame* f) {
    apriltag_family_t* fam = tag36h11_create();
    image_u8_t*        tag = apriltag_to_image(fam, kTagId);

    CVPixelBufferRef pb = f->pixel_buffer();
    CVPixelBufferLockBaseAddress(pb, 0);
    auto*        base   = static_cast<uint8_t*>(CVPixelBufferGetBaseAddress(pb));
    const size_t stride = CVPixelBufferGetBytesPerRow(pb);

    // White background.
    for (int y = 0; y < kImgH; ++y) std::fill_n(base + y * stride, kImgW, 255);

    // Nearest-neighbour upscale of the tag bitmap, centered.
    const int patch = tag->width * kPxPerCell;
    const int x0    = kImgW / 2 - patch / 2;
    const int y0    = kImgH / 2 - patch / 2;
    for (int y = 0; y < patch; ++y) {
        for (int x = 0; x < patch; ++x) {
            base[(y0 + y) * stride + (x0 + x)] =
                tag->buf[(y / kPxPerCell) * tag->stride + (x / kPxPerCell)];
        }
    }

    CVPixelBufferUnlockBaseAddress(pb, 0);
    image_u8_destroy(tag);
    tag36h11_destroy(fam);
}

PinholeCamera test_camera() {
    PinholeCamera cam;
    cam.fxfycxcy = {kFx, kFx, kImgW / 2.0, kImgH / 2.0};
    cam.model    = PinholeCamera::Dist::kRadTan;
    cam.d        = {0, 0, 0, 0};
    cam.width    = kImgW;
    cam.height   = kImgH;
    return cam;
}

// One tag at field (2, 0, 0) facing toward blue (−X, toward the camera).
std::shared_ptr<const SharedTagConfig> test_shared(double ambiguity_ratio) {
    FieldLayout layout;
    layout.length_m = 16.0;
    layout.width_m  = 8.0;
    layout.tags.push_back(
        {kTagId, mat4_from_rt(quat_wxyz_to_mat3(0, 0, 0, 1), {2.0, 0.0, 0.0})});

    auto shared = std::make_shared<SharedTagConfig>();
    shared->layout      = prepare_layout(layout);
    shared->T_robot_imu = mat4_identity();
    // The rendered tag is fronto-parallel — disable the ambiguity gate
    // (ratio < 1 can never trigger since solutions are sorted ascending).
    shared->est.ambiguity_min_ratio = ambiguity_ratio;
    return shared;
}

template <typename Pred>
bool wait_until(Pred pred, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return pred();
}

}  // namespace

TEST(AprilTagConsumerTest, RenderedTagProducesMeasurementOnBus) {
    FramePool    pool(kMono640x480, 4);
    FrameChannel ch;
    auto         bus = std::make_shared<TagPoseBus>();
    auto         sub = bus->subscribe();

    AprilTagConsumer consumer(/*camera_id=*/42, "bench", test_camera(),
                              /*T_cam_imu=*/mat4_identity(), bus,
                              test_shared(/*ambiguity_ratio=*/1.0));
    consumer.attach(ch);

    Frame* f = pool.acquire();
    ASSERT_NE(f, nullptr);
    f->set_sequence(1);
    f->set_host_capture_ns(1'000'000);
    f->set_camera_ts_ns(5'000'000'000ull);  // sync controller-domain stamp
    draw_tag_frame(f);
    ch.publish(f);

    TagPoseMeasurement m;
    const bool got = wait_until([&] { return bus->try_pop(sub, m); },
                                std::chrono::milliseconds(2000));
    const auto dbg = consumer.snapshot();
    ASSERT_TRUE(got) << "frames=" << dbg.frames_seen
                     << " dets=" << dbg.detections_total
                     << " no_tags=" << dbg.skipped_no_tags
                     << " ambiguous=" << dbg.skipped_ambiguous
                     << " high_reproj=" << dbg.skipped_high_reproj
                     << " no_extr=" << dbg.skipped_no_extrinsics
                     << " solve_failed=" << dbg.skipped_solve_failed
                     << " reason=" << dbg.reason;
    consumer.detach();

    EXPECT_EQ(m.camera_id, 42);
    EXPECT_EQ(m.t_ns, 5'000'000'000ll);
    EXPECT_EQ(m.clock_source, TagPoseMeasurement::Clock::kSyncController);
    EXPECT_EQ(m.n_tags, 1u);
    ASSERT_EQ(m.tag_ids.size(), 1u);
    EXPECT_EQ(m.tag_ids[0], kTagId);

    // With identity extrinsics the robot frame IS the camera frame; the
    // camera sits directly in front of the tag center, so its field position
    // is (2 − range, ≈0, ≈0).
    EXPECT_NEAR(m.T_field_robot[0][3], 2.0 - kExpectedRangeM, 0.03);
    EXPECT_NEAR(m.T_field_robot[1][3], 0.0, 0.05);
    EXPECT_NEAR(m.T_field_robot[2][3], 0.0, 0.05);

    const auto snap = consumer.snapshot();
    EXPECT_EQ(snap.published, 1u);
    EXPECT_EQ(snap.reason, "ok");
    ASSERT_EQ(snap.last_tags.size(), 1u);
    ASSERT_TRUE(snap.last_tags[0].range_m.has_value());
    EXPECT_NEAR(*snap.last_tags[0].range_m, kExpectedRangeM,
                kExpectedRangeM * 0.02);
    EXPECT_GT(snap.last_latency_ms, 0.0);
}

TEST(AprilTagConsumerTest, NoExtrinsicsChainDetectsButDoesNotPublish) {
    FramePool    pool(kMono640x480, 4);
    FrameChannel ch;
    auto         bus = std::make_shared<TagPoseBus>();
    auto         sub = bus->subscribe();

    // No T_robot_imu in the shared config → chain incomplete.
    auto shared = std::make_shared<SharedTagConfig>(*test_shared(1.0));
    shared->T_robot_imu.reset();

    AprilTagConsumer consumer(7, "bench", test_camera(), mat4_identity(), bus,
                              shared);
    consumer.attach(ch);

    Frame* f = pool.acquire();
    ASSERT_NE(f, nullptr);
    f->set_sequence(1);
    f->set_host_capture_ns(1'000'000);
    f->set_camera_ts_ns(1'000'000'000ull);
    draw_tag_frame(f);
    ch.publish(f);

    ASSERT_TRUE(wait_until(
        [&] { return consumer.snapshot().skipped_no_extrinsics >= 1; },
        std::chrono::milliseconds(2000)));
    consumer.detach();

    const auto snap = consumer.snapshot();
    EXPECT_EQ(snap.published, 0u);
    EXPECT_EQ(snap.reason, "no_extrinsics_chain");
    // Ranges still computed for bench validation.
    ASSERT_EQ(snap.last_tags.size(), 1u);
    EXPECT_TRUE(snap.last_tags[0].range_m.has_value());
    TagPoseMeasurement m;
    EXPECT_FALSE(bus->try_pop(sub, m));
}

TEST(AprilTagConsumerTest, ResolutionMismatchReported) {
    FramePool    pool(kMono640x480, 4);
    FrameChannel ch;
    auto         bus = std::make_shared<TagPoseBus>();

    auto cam  = test_camera();
    cam.width = 2048;  // stored intrinsics for a different mode
    cam.height = 1536;
    AprilTagConsumer consumer(7, "bench", cam, mat4_identity(), bus,
                              test_shared(1.0));
    consumer.attach(ch);

    Frame* f = pool.acquire();
    ASSERT_NE(f, nullptr);
    f->set_sequence(1);
    f->set_camera_ts_ns(1'000'000'000ull);
    draw_tag_frame(f);
    ch.publish(f);

    ASSERT_TRUE(wait_until(
        [&] { return consumer.snapshot().reason == "resolution_mismatch"; },
        std::chrono::milliseconds(2000)));
    consumer.detach();
    EXPECT_EQ(consumer.snapshot().published, 0u);
}

TEST(AprilTagConsumerTest, DetachIsIdempotent) {
    FrameChannel ch;
    auto         bus = std::make_shared<TagPoseBus>();
    AprilTagConsumer consumer(1, "x", test_camera(), std::nullopt, bus,
                              test_shared(1.0));
    consumer.attach(ch);
    consumer.detach();
    EXPECT_NO_THROW(consumer.detach());
}

}  // namespace gw::apriltag
