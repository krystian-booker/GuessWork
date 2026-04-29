// test_kimera_integration.cpp — end-to-end integration tests for the
// real Kimera-VIO backend.
//
// COMPILED ONLY WHEN POSEST_BUILD_VIO IS ON. These tests link against
// libKimeraVIO and exercise the actual MIT-SPARK pipeline rather than
// the FakeVioBackend used by test_kimera_vio_consumer.cpp.
//
// What this file covers (above and beyond the lifecycle smoke tests
// in test_kimera_backend.cpp):
//   1. KimeraParamWriter's 7 YAMLs are accepted by VIO::VioParams's
//      parseYAML — i.e., every required field is present with a
//      casing Kimera's parser actually reads. This is the regression
//      surface most likely to silently rot when Kimera is upgraded.
//   2. KimeraVioConsumer + real KimeraBackend wired together can
//      ingest a burst of synthetic frames + 1 kHz IMU samples without
//      hanging or crashing the spin thread.
//   3. The pipeline's output callback fires at least once when fed a
//      moving textured scene and matched IMU — i.e., the wiring from
//      our consumer's tryPushFrame/tryPushImu through Kimera's
//      internal threads back to onBackendOutput is actually closed.
//      Trajectory accuracy is NOT asserted; the goal is "VIO speaks
//      back" not "VIO is metrologically correct".
//
// What this file does NOT cover:
//   - End-to-end fusion (FusionService consuming VioMeasurement).
//     That path is exercised by test_fusion_service.cpp using
//     synthetic VioMeasurement values; we do not spin up the whole
//     daemon here.
//   - Trajectory convergence on real data. That requires a recorded
//     dataset with ground truth and lives outside the unit-test
//     suite (see docs/features for the planned VIO bring-up flow).

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "posest/Frame.h"
#include "posest/MeasurementBus.h"
#include "posest/MeasurementTypes.h"
#include "posest/Timestamp.h"
#include "posest/runtime/RuntimeConfig.h"
#include "posest/vio/IVioBackend.h"
#include "posest/vio/KimeraParamWriter.h"
#include "posest/vio/KimeraVioConfig.h"
#include "posest/vio/KimeraVioConsumer.h"

namespace posest::vio {

// Forward declaration — the factory lives in KimeraBackend.cpp.
std::unique_ptr<IVioBackend> makeKimeraBackend(const std::string& param_dir);

}  // namespace posest::vio

namespace {

using namespace std::chrono_literals;

// 1:1 time converter — Teensy microseconds map directly to a
// steady_clock time point. Production binds this to
// TeensyService::timestampFromTeensyTime; the integration test
// bypasses the time-sync filter entirely so the assertions don't
// depend on the filter's bootstrap state.
posest::Timestamp identityConverter(std::uint64_t teensy_time_us,
                                    posest::Timestamp /*fallback*/) {
    return posest::Timestamp{std::chrono::microseconds(teensy_time_us)};
}

std::filesystem::path makeTempParamDir(const std::string& tag) {
    const auto stamp =
        std::chrono::steady_clock::now().time_since_epoch().count();
    auto p = std::filesystem::temp_directory_path() /
             ("posest_kimera_integration_" + tag + "_" +
              std::to_string(stamp));
    std::filesystem::create_directories(p);
    return p;
}

posest::runtime::RuntimeConfig makeVioReadyConfig() {
    posest::runtime::RuntimeConfig config;

    posest::CameraConfig cam;
    cam.id = "downward";
    cam.type = "v4l2";
    cam.device = "/dev/video0";
    cam.enabled = true;
    cam.format.width = 640;
    cam.format.height = 480;
    cam.format.fps = 100.0;
    cam.format.pixel_format = "mjpeg";
    config.cameras.push_back(cam);

    posest::runtime::CameraCalibrationConfig calib;
    calib.camera_id = "downward";
    calib.version = "v1";
    calib.active = true;
    calib.source_file_path = "/tmp/calib.yaml";
    calib.created_at = "2026-04-29T00:00:00Z";
    calib.image_width = 640;
    calib.image_height = 480;
    calib.camera_model = "pinhole";
    calib.distortion_model = "radtan";
    calib.fx = 500.0;
    calib.fy = 500.0;
    calib.cx = 320.0;
    calib.cy = 240.0;
    calib.distortion_coefficients = {0.0, 0.0, 0.0, 0.0};
    config.calibrations.push_back(calib);

    posest::runtime::CameraImuCalibrationConfig cam_imu;
    cam_imu.camera_id = "downward";
    cam_imu.version = "imu-v1";
    cam_imu.active = true;
    cam_imu.source_file_path = "/tmp/imucam.yaml";
    cam_imu.created_at = "2026-04-29T00:01:00Z";
    // Identity camera↔IMU: the integration test only cares that
    // Kimera spins up and emits, not about the geometry of any one
    // calibration — and identity makes the synthetic frames'
    // ego-motion show up as a pure +X translation in Kimera's body
    // frame.
    cam_imu.camera_to_imu = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    cam_imu.imu_to_camera = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    cam_imu.time_shift_s = 0.0;
    config.camera_imu_calibrations.push_back(cam_imu);

    config.vio.enabled = true;
    config.vio.vio_camera_id = "downward";
    return config;
}

// Build a textured 640×480 frame whose patch shifts horizontally with
// `x_offset_px`. The pattern is dense enough for GFTT to lock onto
// hundreds of corners (a uniform-gray frame would produce zero
// keypoints and Kimera's frontend would never initialize).
cv::Mat makeTexturedFrame(int x_offset_px) {
    constexpr int kW = 640;
    constexpr int kH = 480;
    cv::Mat img(kH, kW, CV_8UC1);

    // Deterministic PRNG so each call produces the same texture
    // shifted, not a fresh noise field. Re-seeding per call also
    // means the test is reproducible across runs.
    std::mt19937 rng(0xC0FFEEu);
    std::uniform_int_distribution<int> noise(0, 40);
    for (int y = 0; y < kH; ++y) {
        for (int x = 0; x < kW; ++x) {
            const int sx = ((x - x_offset_px) % kW + kW) % kW;
            // Checkerboard with random sub-tile noise. The 32 px tile
            // is large enough that GFTT finds corners at every edge,
            // while the noise gives the optical-flow tracker enough
            // local texture to disambiguate a tile from its neighbour.
            const int tile = ((sx / 32) + (y / 32)) & 1;
            const int v = (tile ? 200 : 50) + noise(rng);
            img.at<std::uint8_t>(y, x) =
                static_cast<std::uint8_t>(v & 0xFF);
        }
    }
    // Light blur to smooth the high-frequency noise — keeps the GFTT
    // corner response on the tile edges, not on individual pixels.
    cv::GaussianBlur(img, img, cv::Size(3, 3), 0.0);
    return img;
}

posest::FramePtr makeFrame(std::uint64_t teensy_time_us,
                           std::uint64_t sequence,
                           const cv::Mat& image,
                           std::optional<double> ground_distance_m) {
    auto f = std::make_shared<posest::Frame>();
    f->capture_time =
        posest::Timestamp{std::chrono::microseconds(teensy_time_us)};
    f->sequence = sequence;
    f->camera_id = "downward";
    f->image = image;
    f->teensy_time_us = teensy_time_us;
    f->ground_distance_m = ground_distance_m;
    return f;
}

// Stress driver: pump `n_frames` frames at ~100 Hz with `imu_per_frame`
// IMU samples interleaved between each. Returns when the consumer has
// observed every frame — does not wait for Kimera's backend output.
struct DriveResult {
    std::uint64_t frames_pushed{0};
    std::uint64_t imu_pushed{0};
    std::uint64_t outputs_received{0};
};

DriveResult driveBurst(posest::vio::KimeraVioConsumer& consumer,
                       posest::MeasurementBus& imu_bus,
                       std::size_t n_frames,
                       std::size_t imu_per_frame) {
    constexpr std::uint64_t kFrameDtUs = 10'000;     // 100 Hz
    const std::uint64_t imu_dt_us =
        kFrameDtUs / std::max<std::size_t>(imu_per_frame, 1);

    std::uint64_t teensy_us = 1'000'000;
    for (std::size_t i = 0; i < n_frames; ++i) {
        // IMU first — KimeraVioConsumer's frame worker drains
        // `imu_buffer_` up to the frame's teensy_us at the start of
        // process(), so samples published after the frame would
        // never be paired with it.
        for (std::size_t k = 0; k < imu_per_frame; ++k) {
            posest::ImuSample s;
            const std::uint64_t t = teensy_us + imu_dt_us * k;
            s.timestamp = posest::Timestamp{std::chrono::microseconds(t)};
            s.teensy_time_us = t;
            // Stationary on a level surface: gravity points -Z in the
            // world frame, so the accelerometer reads +9.81 along the
            // body Z axis. Kimera will use this for gravity-vector
            // initialization.
            s.accel_mps2 = {0.0, 0.0, 9.81};
            s.gyro_radps = {0.0, 0.0, 0.0};
            imu_bus.publish(s);
        }

        const int x_offset =
            static_cast<int>(i) * 2;  // 2 px / frame ≈ slow pan
        consumer.deliver(
            makeFrame(teensy_us + kFrameDtUs, i,
                      makeTexturedFrame(x_offset), 0.05));

        // Pace at ~100 Hz wall-clock. ConsumerBase's drop-oldest
        // mailbox would otherwise coalesce frames before Kimera's
        // frontend ever sees them. 10 ms matches kFrameDtUs.
        std::this_thread::sleep_for(10ms);
        teensy_us += kFrameDtUs;
    }

    DriveResult r;
    const auto stats = consumer.stats();
    r.frames_pushed = stats.frames_pushed;
    r.imu_pushed = stats.imu_pushed;
    r.outputs_received = stats.outputs_received;
    return r;
}

class KimeraIntegrationFixture : public ::testing::Test {
protected:
    void SetUp() override {
        param_dir_ = makeTempParamDir(
            ::testing::UnitTest::GetInstance()->current_test_info()->name());
        const auto cfg = makeVioReadyConfig();
        ASSERT_NO_THROW(
            posest::vio::emitKimeraParamYamls(cfg, param_dir_.string()));
    }

    void TearDown() override {
        std::error_code ec;
        std::filesystem::remove_all(param_dir_, ec);
    }

    std::filesystem::path param_dir_;
};

// 1. Every YAML the writer emits must satisfy Kimera's parser. A
// missing field or a casing mismatch causes
// VIO::VioParams::parseYAML to LOG(FATAL) at start() time. Building
// the backend successfully is the strongest cheap signal that the
// schema is intact.
TEST_F(KimeraIntegrationFixture, WriterOutputAcceptedByVioParamsParser) {
    auto backend = posest::vio::makeKimeraBackend(param_dir_.string());
    ASSERT_NE(backend, nullptr);

    std::atomic<int> outputs{0};
    backend->setOutputCallback(
        [&outputs](const posest::vio::VioBackendOutput&) { ++outputs; });

    // start() is what loads + parses every YAML; it returns void on
    // success and LOG(FATAL)s on parse failure (which would terminate
    // the test process and surface as a SIGABRT in ctest).
    backend->start();

    // Two pushes to confirm the input queues exist after parse-time
    // wiring — fail-fast if the pipeline stood up but the data
    // provider module didn't.
    cv::Mat image(480, 640, CV_8UC1, cv::Scalar(0));
    EXPECT_TRUE(backend->tryPushFrame(1'000'000ULL, image));
    EXPECT_TRUE(backend->tryPushImu(1'000'000ULL, {0.0, 0.0, 9.81},
                                    {0.0, 0.0, 0.0}));

    backend->stop();
}

// 2. The KimeraVioConsumer + real KimeraBackend wiring must survive a
// realistic burst (60 frames at ~100 Hz, 8 IMU per frame). The
// success bar is "frames_pushed > 0" — i.e., the wiring doesn't
// deadlock on the first frame.
//
// If frames_pushed stays at 0 after the burst it means
// KimeraVioConsumer::process() never returned from
// backend->tryPushFrame, which would indicate Kimera's
// fillLeftFrameQueue grew a blocking variant.
TEST_F(KimeraIntegrationFixture, ConsumerSurvivesRealisticBurst) {
    posest::MeasurementBus imu_bus(2048);
    posest::MeasurementBus out_bus(64);

    auto backend = posest::vio::makeKimeraBackend(param_dir_.string());
    ASSERT_NE(backend, nullptr);

    posest::vio::KimeraVioConfig cfg;
    cfg.param_dir = param_dir_.string();
    cfg.camera_id = "vio";

    posest::vio::KimeraVioConsumer consumer(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), cfg);
    consumer.start();

    auto r = driveBurst(consumer, imu_bus, /*n_frames=*/60,
                        /*imu_per_frame=*/8);

    EXPECT_GT(r.frames_pushed, 0u)
        << "consumer's frame worker never pushed into the backend — "
           "tryPushFrame is likely blocking";
    EXPECT_GT(r.imu_pushed, 0u)
        << "consumer's IMU drainer never forwarded a sample";

    consumer.stop();
}

// 3. With enough frames + IMU the backend must produce at least one
// output. This is the strongest "Kimera is actually running" signal
// short of asserting trajectory accuracy: it proves
//   producer → KimeraVioConsumer.process()
//     → backend->tryPushFrame
//       → Kimera's MonoVisionImuFrontend
//         → VioBackend
//           → backend output callback
//             → consumer.onBackendOutput
// is closed end-to-end. We poll consumer.stats().outputs_received
// (bumped inside onBackendOutput) since that field is visible
// regardless of whether the output is published downstream — it
// captures the callback-fire event itself, not the post-filter
// VioMeasurement.
//
// This test is intentionally tolerant on the success bar (≥1 output
// over a generous deadline) because Kimera's initialization is
// stochastic on synthetic textures and the goal here is liveness,
// not convergence. Convergence belongs in a recorded-dataset
// regression suite.
TEST_F(KimeraIntegrationFixture, BackendProducesAtLeastOneOutput) {
    posest::MeasurementBus imu_bus(2048);
    posest::MeasurementBus out_bus(64);

    auto backend = posest::vio::makeKimeraBackend(param_dir_.string());
    ASSERT_NE(backend, nullptr);

    posest::vio::KimeraVioConfig cfg;
    cfg.param_dir = param_dir_.string();
    cfg.camera_id = "vio";

    posest::vio::KimeraVioConsumer consumer(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), cfg);
    consumer.start();

    // 300 frames at 100 Hz ≈ 3 s of wall-clock. Kimera's frontend
    // typically initializes within ~50 frames on textured input,
    // and the backend lags the frontend by a keyframe or two.
    driveBurst(consumer, imu_bus, /*n_frames=*/300,
               /*imu_per_frame=*/8);

    // After the burst, give Kimera's internal threads up to 5 s to
    // catch up on the queued frames before sampling stats. Polling
    // is required because we don't know exactly when the backend
    // will emit (it's keyframe-driven, not 1:1 with input frames).
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    std::uint64_t outputs = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        outputs = consumer.stats().outputs_received;
        if (outputs > 0) break;
        std::this_thread::sleep_for(50ms);
    }

    EXPECT_GT(outputs, 0u)
        << "Kimera's backend never invoked the output callback after "
           "300 frames + 2400 IMU samples — most likely the frontend "
           "rejected every keypoint (texture too smooth) or the IMU "
           "stream was rejected (gravity vector / timestamp ordering).";

    consumer.stop();
}

}  // namespace
