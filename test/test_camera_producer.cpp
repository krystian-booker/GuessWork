#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "posest/CameraCapabilities.h"
#include "posest/CameraConfig.h"
#include "posest/CameraProducer.h"
#include "posest/ConsumerBase.h"
#include "posest/MockCameraProducer.h"

using namespace std::chrono_literals;

namespace {

posest::CameraConfig makeConfig(const std::string& id = "mock") {
    posest::CameraConfig cfg;
    cfg.id = id;
    cfg.type = "mock";
    cfg.device = "/dev/null";
    cfg.format.width = 4;
    cfg.format.height = 4;
    cfg.format.fps = 30.0;
    cfg.format.pixel_format = "mjpeg";
    return cfg;
}

class CountingConsumer final : public posest::ConsumerBase {
public:
    explicit CountingConsumer(std::string id) : posest::ConsumerBase(std::move(id)) {}
    std::uint64_t count() const { return count_.load(); }

protected:
    void process(const posest::Frame&) override { count_.fetch_add(1); }

private:
    std::atomic<std::uint64_t> count_{0};
};

void waitFor(const std::function<bool()>& predicate,
             std::chrono::milliseconds timeout = 2s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(1ms);
    }
}

}  // namespace

TEST(CameraProducer, LifecycleHooksFireInOrderOnStart) {
    posest::mock::MockCameraProducer prod(makeConfig());
    prod.setDefaultGrabResult(posest::GrabResult::Stopping);
    prod.start();
    prod.stop();

    const auto history = prod.hookHistory();
    ASSERT_GE(history.size(), 4u);
    EXPECT_EQ(history[0], posest::mock::CameraHook::OpenDevice);
    EXPECT_EQ(history[1], posest::mock::CameraHook::ApplyFormat);
    EXPECT_EQ(history[2], posest::mock::CameraHook::ApplyControls);
    EXPECT_EQ(history[3], posest::mock::CameraHook::StartStream);
}

TEST(CameraProducer, StartStopRestartIsClean) {
    posest::mock::MockCameraProducer prod(makeConfig());
    prod.setDefaultGrabResult(posest::GrabResult::Stopping);
    prod.start();
    prod.stop();
    prod.start();
    prod.stop();
    EXPECT_EQ(prod.openDeviceCount(), 2u);
    EXPECT_EQ(prod.closeDeviceCount(), 2u);
    EXPECT_EQ(prod.startStreamCount(), 2u);
    EXPECT_EQ(prod.stopStreamCount(), 2u);
}

TEST(CameraProducer, ApplyControlsErrorsArePropagated) {
    posest::mock::MockCameraProducer prod(makeConfig());
    prod.setDefaultGrabResult(posest::GrabResult::Stopping);
    prod.scriptApplyControlsErrors({{"gain", 99, "permission denied"}});
    prod.start();
    prod.stop();

    const auto errors = prod.lastApplyErrors();
    ASSERT_EQ(errors.size(), 1u);
    EXPECT_EQ(errors[0].name, "gain");
    EXPECT_EQ(errors[0].requested_value, 99);
    EXPECT_EQ(errors[0].error, "permission denied");
}

TEST(CameraProducer, SetTriggerModeFreeRunIsNoop) {
    posest::mock::MockCameraProducer prod(makeConfig());
    EXPECT_NO_THROW(prod.setTriggerMode(posest::TriggerMode::FreeRun));
}

TEST(CameraProducer, SetTriggerModeExternalThrowsNotSupported) {
    posest::mock::MockCameraProducer prod(makeConfig());
    EXPECT_THROW(prod.setTriggerMode(posest::TriggerMode::External),
                 posest::NotSupportedError);
    EXPECT_THROW(prod.setTriggerMode(posest::TriggerMode::Software),
                 posest::NotSupportedError);
}

TEST(CameraProducer, SetControlDefaultThrowsNotSupported) {
    posest::mock::MockCameraProducer prod(makeConfig());
    EXPECT_THROW(prod.setControl("gain", 5), posest::NotSupportedError);
}

TEST(CameraProducer, GetControlDefaultThrowsNotSupported) {
    posest::mock::MockCameraProducer prod(makeConfig());
    EXPECT_THROW((void)prod.getControl("gain"), posest::NotSupportedError);
}

TEST(CameraProducer, BaseCapabilitiesReportsCurrentFormatAfterStart) {
    posest::mock::MockCameraProducer prod(makeConfig());
    prod.setDefaultGrabResult(posest::GrabResult::Stopping);
    prod.start();
    prod.stop();

    const auto caps = prod.capabilities();
    EXPECT_EQ(caps.camera_id, "mock");
    EXPECT_EQ(caps.backend, "mock");
    ASSERT_EQ(caps.trigger_modes.size(), 1u);
    EXPECT_EQ(caps.trigger_modes[0], posest::TriggerMode::FreeRun);
    EXPECT_FALSE(caps.supports_set_control);
    EXPECT_FALSE(caps.supports_get_control);
    EXPECT_TRUE(caps.supports_reconnect);   // default reconnect.interval_ms = 1000
    ASSERT_TRUE(caps.current_format.has_value());
    EXPECT_EQ(caps.current_format->width, 4);
}

TEST(CameraProducer, TransientErrorTriggersReconnectInOrder) {
    auto cfg = makeConfig();
    cfg.reconnect.interval_ms = 1;     // fast reconnect for the test
    cfg.reconnect.max_attempts = 1;    // give up after one retry on the second failure
    posest::mock::MockCameraProducer prod(cfg);

    // Script: TransientError, then Stopping after the reconnect.
    prod.scriptGrabResults({posest::GrabResult::TransientError});
    prod.setDefaultGrabResult(posest::GrabResult::Stopping);

    auto consumer = std::make_shared<CountingConsumer>("c");
    prod.addConsumer(consumer);
    consumer->start();
    prod.start();
    waitFor([&] { return prod.openDeviceCount() >= 2; });
    prod.stop();
    consumer->stop();

    EXPECT_GE(prod.openDeviceCount(), 2u);
    EXPECT_GE(prod.applyFormatCount(), 2u);
    EXPECT_GE(prod.applyControlsCount(), 2u);
    EXPECT_GE(prod.startStreamCount(), 2u);

    const auto stats = prod.liveStats();
    EXPECT_GE(stats.successful_connects, 2u);
}

TEST(CameraProducer, ReconnectInterruptedByStop) {
    auto cfg = makeConfig();
    cfg.reconnect.interval_ms = 200;     // long enough to observe interruption
    cfg.reconnect.max_attempts = 100;
    posest::mock::MockCameraProducer prod(cfg);

    // Default to TransientError so the worker enters reconnect immediately.
    // openDevice failures are armed AFTER start so the initial open succeeds;
    // the worker will then fail every reconnect attempt and stay in the
    // sleep loop until stop() interrupts.
    prod.setDefaultGrabResult(posest::GrabResult::TransientError);
    prod.start();
    prod.scriptOpenDeviceFailures(100, "device unplugged");

    waitFor([&] { return prod.openDeviceCount() >= 2; });
    const auto t0 = std::chrono::steady_clock::now();
    prod.stop();
    const auto elapsed = std::chrono::steady_clock::now() - t0;
    EXPECT_LT(elapsed, 500ms) << "stop() should interrupt the reconnect sleep";
}

TEST(CameraProducer, MaxAttemptsHonoredAndStateTransitionsToFailed) {
    auto cfg = makeConfig();
    cfg.reconnect.interval_ms = 5;
    cfg.reconnect.max_attempts = 3;
    posest::mock::MockCameraProducer prod(cfg);

    prod.setDefaultGrabResult(posest::GrabResult::TransientError);
    prod.start();
    prod.scriptOpenDeviceFailures(100, "permanent failure");

    waitFor([&] {
        const auto stats = prod.liveStats();
        return stats.state == posest::ConnectionState::Failed;
    });
    const auto stats = prod.liveStats();
    EXPECT_EQ(stats.state, posest::ConnectionState::Failed);
    EXPECT_GE(stats.reconnect_attempts, 3u);
    EXPECT_GE(stats.disconnect_count, 3u);
    prod.stop();
}

TEST(CameraProducer, ReconnectDisabledWhenIntervalIsZero) {
    auto cfg = makeConfig();
    cfg.reconnect.interval_ms = 0;     // reconnect disabled
    posest::mock::MockCameraProducer prod(cfg);

    prod.scriptGrabResults({posest::GrabResult::TransientError});
    prod.setDefaultGrabResult(posest::GrabResult::Stopping);
    prod.start();
    waitFor([&] { return prod.grabFrameCount() >= 1; });
    prod.stop();

    // No reconnect attempts when disabled.
    EXPECT_EQ(prod.openDeviceCount(), 1u);
}
