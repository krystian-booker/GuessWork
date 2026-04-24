#pragma once

#include <cstdint>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "posest/MeasurementBus.h"
#include "posest/fusion/IFusionOutputSink.h"
#include "posest/runtime/RuntimeConfig.h"
#include "posest/teensy/Protocol.h"
#include "posest/teensy/SerialTransport.h"

namespace posest::teensy {

struct TeensyStats {
    bool enabled{false};
    bool connected{false};
    std::uint64_t reconnect_attempts{0};
    std::uint64_t successful_connects{0};
    std::uint64_t disconnects{0};
    std::uint64_t crc_failures{0};
    std::uint64_t sequence_gaps{0};
    std::uint64_t invalid_payloads{0};
    std::uint64_t inbound_imu_samples{0};
    std::uint64_t inbound_wheel_odometry_samples{0};
    std::uint64_t inbound_robot_odometry_samples{0};
    std::uint64_t inbound_measurements_dropped{0};
    std::uint64_t outbound_frames_queued{0};
    std::uint64_t outbound_frames_sent{0};
    std::uint64_t outbound_frames_dropped{0};
    std::optional<Timestamp> last_receive_time;
    std::optional<Timestamp> last_transmit_time;
    std::string last_error;
    bool time_sync_established{false};
    std::int64_t time_sync_offset_us{0};
    std::uint64_t time_sync_round_trip_us{0};
};

class TeensyService final : public fusion::IFusionOutputSink {
public:
    using SerialTransportFactory = std::function<std::unique_ptr<ISerialTransport>()>;

    TeensyService(
        runtime::TeensyConfig config,
        std::vector<runtime::CameraTriggerConfig> camera_triggers,
        IMeasurementSink& measurement_sink,
        SerialTransportFactory transport_factory = makePosixSerialTransport);
    ~TeensyService() override;

    TeensyService(const TeensyService&) = delete;
    TeensyService& operator=(const TeensyService&) = delete;

    void start();
    void stop();

    void publish(FusedPoseEstimate estimate) override;

    std::optional<Frame> takeLastOutboundFrame() const;
    TeensyStats stats() const;

private:
    static std::vector<std::uint8_t> encodeFusedPosePayload(const FusedPoseEstimate& estimate);

    void workerLoop();
    void runConnected(ISerialTransport& transport);
    void enqueueFrame(Frame frame);
    bool flushOneOutbound(ISerialTransport& transport);
    void handleFrame(const Frame& frame);
    void sendTimeSyncRequest(ISerialTransport& transport, Timestamp now);
    void handleTimeSyncResponse(const TimeSyncResponsePayload& payload, Timestamp host_receive);
    Timestamp timestampFromTeensyTime(std::uint64_t teensy_time_us, Timestamp fallback);
    std::vector<std::uint8_t> encodeCameraTriggerConfigPayload() const;
    void sendCameraTriggerConfig(ISerialTransport& transport);
    void markDisconnected(const std::string& error);
    void sleepUntilReconnectOrStop();
    static std::uint64_t steadyMicros(Timestamp timestamp);

    static constexpr std::size_t kOutboundQueueCapacity = 64;
    static constexpr std::chrono::milliseconds kTimeSyncInterval{1000};

    runtime::TeensyConfig config_;
    std::vector<runtime::CameraTriggerConfig> camera_triggers_;
    IMeasurementSink& measurement_sink_;
    SerialTransportFactory transport_factory_;
    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::optional<Frame> last_outbound_frame_;
    std::deque<Frame> outbound_queue_;
    std::unique_ptr<ISerialTransport> current_transport_;
    std::thread worker_;
    bool stop_requested_{false};
    bool started_{false};
    TeensyStats stats_;
    std::optional<std::uint32_t> pending_time_sync_sequence_;
    std::uint64_t pending_time_sync_host_send_us_{0};
    std::atomic<std::uint32_t> next_time_sync_sequence_{0};
    std::atomic<std::uint32_t> next_sequence_{0};
};

}  // namespace posest::teensy
