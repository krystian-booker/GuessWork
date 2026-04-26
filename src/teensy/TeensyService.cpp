#include "posest/teensy/TeensyService.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

#include "posest/CameraTriggerCache.h"

namespace posest::teensy {

namespace {

void appendDouble(std::vector<std::uint8_t>& out, double value) {
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(value));
    for (int shift = 0; shift <= 56; shift += 8) {
        out.push_back(static_cast<std::uint8_t>(
            (bits >> static_cast<unsigned>(shift)) & 0xFFu));
    }
}

void appendU32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFFu));
    out.push_back(static_cast<std::uint8_t>((value >> 8u) & 0xFFu));
    out.push_back(static_cast<std::uint8_t>((value >> 16u) & 0xFFu));
    out.push_back(static_cast<std::uint8_t>((value >> 24u) & 0xFFu));
}

void appendI64(std::vector<std::uint8_t>& out, std::int64_t value) {
    const auto unsigned_value = static_cast<std::uint64_t>(value);
    for (int shift = 0; shift <= 56; shift += 8) {
        out.push_back(static_cast<std::uint8_t>(
            (unsigned_value >> static_cast<unsigned>(shift)) & 0xFFu));
    }
}

}  // namespace

TeensyService::TeensyService(
    runtime::TeensyConfig config,
    std::vector<runtime::CameraTriggerConfig> camera_triggers,
    IMeasurementSink& measurement_sink,
    SerialTransportFactory transport_factory,
    std::shared_ptr<CameraTriggerCache> trigger_cache)
    : config_(std::move(config)),
      camera_triggers_(std::move(camera_triggers)),
      measurement_sink_(measurement_sink),
      transport_factory_(std::move(transport_factory)),
      trigger_cache_(std::move(trigger_cache)) {
    if (!transport_factory_) {
        throw std::invalid_argument("TeensyService requires a serial transport factory");
    }
    stats_.enabled = !config_.serial_port.empty();
}

TeensyService::~TeensyService() {
    stop();
}

void TeensyService::start() {
    std::lock_guard<std::mutex> g(mu_);
    if (started_ || !stats_.enabled) {
        return;
    }
    stop_requested_ = false;
    started_ = true;
    worker_ = std::thread(&TeensyService::workerLoop, this);
}

void TeensyService::stop() {
    {
        std::lock_guard<std::mutex> g(mu_);
        if (!started_) {
            return;
        }
        stop_requested_ = true;
        if (current_transport_) {
            current_transport_->close();
        }
        cv_.notify_all();
    }
    if (worker_.joinable()) {
        worker_.join();
    }
    std::lock_guard<std::mutex> g(mu_);
    started_ = false;
    stats_.connected = false;
    current_transport_.reset();
}

void TeensyService::publish(FusedPoseEstimate estimate) {
    Frame frame;
    frame.type = MessageType::FusedPose;
    frame.sequence = next_sequence_++;
    frame.payload = encodeFusedPosePayload(estimate);
    enqueueFrame(std::move(frame));
}

std::optional<Frame> TeensyService::takeLastOutboundFrame() const {
    std::lock_guard<std::mutex> g(mu_);
    return last_outbound_frame_;
}

TeensyStats TeensyService::stats() const {
    std::lock_guard<std::mutex> g(mu_);
    return stats_;
}

void TeensyService::workerLoop() {
    while (true) {
        {
            std::lock_guard<std::mutex> g(mu_);
            if (stop_requested_) {
                return;
            }
            ++stats_.reconnect_attempts;
            current_transport_ = transport_factory_();
        }

        try {
            current_transport_->open(config_.serial_port, config_.baud_rate);
            {
                std::lock_guard<std::mutex> g(mu_);
                stats_.connected = true;
                stats_.last_error.clear();
                ++stats_.successful_connects;
            }
            sendCameraTriggerConfig(*current_transport_);
            sendImuConfig(*current_transport_);
            runConnected(*current_transport_);
        } catch (const std::exception& e) {
            markDisconnected(e.what());
        } catch (...) {
            markDisconnected("unknown Teensy serial failure");
        }

        {
            std::lock_guard<std::mutex> g(mu_);
            if (current_transport_) {
                current_transport_->close();
                current_transport_.reset();
            }
            if (stop_requested_) {
                return;
            }
        }
        sleepUntilReconnectOrStop();
    }
}

void TeensyService::runConnected(ISerialTransport& transport) {
    StreamDecoder decoder;
    StreamStats previous_decoder_stats;
    auto next_time_sync = std::chrono::steady_clock::now();
    std::array<std::uint8_t, 512> buffer{};
    const auto sync_interval =
        config_.time_sync_interval_ms > 0u
            ? std::chrono::milliseconds(config_.time_sync_interval_ms)
            : kTimeSyncInterval;

    while (true) {
        {
            std::lock_guard<std::mutex> g(mu_);
            if (stop_requested_) {
                return;
            }
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= next_time_sync) {
            sendTimeSyncRequest(transport, now);
            next_time_sync = now + sync_interval;
        }

        while (flushOneOutbound(transport)) {
        }

        const std::size_t n = transport.read(
            buffer.data(),
            buffer.size(),
            std::chrono::milliseconds(config_.read_timeout_ms));
        if (n == 0) {
            continue;
        }

        auto frames = decoder.push(buffer.data(), n);
        {
            std::lock_guard<std::mutex> g(mu_);
            const auto& stream_stats = decoder.stats();
            stats_.crc_failures += stream_stats.crc_failures -
                                   previous_decoder_stats.crc_failures;
            stats_.sequence_gaps += stream_stats.sequence_gaps -
                                    previous_decoder_stats.sequence_gaps;
            previous_decoder_stats = stream_stats;
        }

        for (const auto& frame : frames) {
            {
                std::lock_guard<std::mutex> g(mu_);
                stats_.last_receive_time = std::chrono::steady_clock::now();
            }
            handleFrame(frame);
        }
    }
}

void TeensyService::enqueueFrame(Frame frame) {
    std::lock_guard<std::mutex> g(mu_);
    last_outbound_frame_ = frame;
    if (!stats_.enabled) {
        return;
    }
    if (outbound_queue_.size() >= kOutboundQueueCapacity) {
        ++stats_.outbound_frames_dropped;
        return;
    }
    outbound_queue_.push_back(std::move(frame));
    ++stats_.outbound_frames_queued;
    cv_.notify_all();
}

bool TeensyService::flushOneOutbound(ISerialTransport& transport) {
    Frame frame;
    {
        std::lock_guard<std::mutex> g(mu_);
        if (outbound_queue_.empty()) {
            return false;
        }
        frame = std::move(outbound_queue_.front());
        outbound_queue_.pop_front();
    }

    transport.write(encodeFrame(frame));
    std::lock_guard<std::mutex> g(mu_);
    ++stats_.outbound_frames_sent;
    stats_.last_transmit_time = std::chrono::steady_clock::now();
    return true;
}

void TeensyService::handleFrame(const Frame& frame) {
    const auto now = std::chrono::steady_clock::now();
    switch (frame.type) {
        case MessageType::ImuSample: {
            const auto decoded = decodeImuPayload(frame.payload);
            if (!decoded) {
                std::lock_guard<std::mutex> g(mu_);
                ++stats_.invalid_payloads;
                return;
            }

            ImuSample sample;
            sample.timestamp = timestampFromTeensyTime(decoded->teensy_time_us, now);
            sample.accel_mps2 = decoded->accel_mps2;
            sample.gyro_radps = decoded->gyro_radps;
            sample.temperature_c = decoded->temperature_c;
            sample.status_flags = decoded->status_flags;
            bool time_sync_established = false;
            {
                std::lock_guard<std::mutex> g(mu_);
                time_sync_established = stats_.time_sync_established;
            }
            if (!time_sync_established) {
                sample.status_flags |= kStatusUnsynchronizedTime;
            }
            const bool published = measurement_sink_.publish(sample);
            std::lock_guard<std::mutex> g(mu_);
            if (published) {
                ++stats_.inbound_imu_samples;
            } else {
                ++stats_.inbound_measurements_dropped;
            }
            break;
        }
        case MessageType::WheelOdometry: {
            const auto decoded = decodeWheelOdometryPayload(frame.payload);
            if (!decoded) {
                std::lock_guard<std::mutex> g(mu_);
                ++stats_.invalid_payloads;
                return;
            }

            WheelOdometrySample sample;
            sample.timestamp = timestampFromTeensyTime(decoded->teensy_time_us, now);
            sample.chassis_delta = decoded->chassis_delta;
            sample.wheel_delta_m.assign(
                decoded->wheel_delta_m.begin(),
                decoded->wheel_delta_m.end());
            sample.status_flags = decoded->status_flags;
            bool time_sync_established = false;
            {
                std::lock_guard<std::mutex> g(mu_);
                time_sync_established = stats_.time_sync_established;
            }
            if (!time_sync_established) {
                sample.status_flags |= kStatusUnsynchronizedTime;
            }
            const bool published = measurement_sink_.publish(sample);
            std::lock_guard<std::mutex> g(mu_);
            if (published) {
                ++stats_.inbound_wheel_odometry_samples;
            } else {
                ++stats_.inbound_measurements_dropped;
            }
            break;
        }
        case MessageType::RobotOdometry: {
            const auto decoded = decodeRobotOdometryPayload(frame.payload);
            if (!decoded) {
                std::lock_guard<std::mutex> g(mu_);
                ++stats_.invalid_payloads;
                return;
            }

            RobotOdometrySample sample;
            sample.timestamp = timestampFromTeensyTime(decoded->teensy_time_us, now);
            sample.field_to_robot = decoded->field_to_robot;
            sample.rio_time_us = decoded->rio_time_us;
            sample.status_flags = decoded->status_flags;
            bool time_sync_established = false;
            {
                std::lock_guard<std::mutex> g(mu_);
                time_sync_established = stats_.time_sync_established;
            }
            if (!time_sync_established) {
                sample.status_flags |= kStatusUnsynchronizedTime;
            }
            const bool published = measurement_sink_.publish(sample);
            std::lock_guard<std::mutex> g(mu_);
            if (published) {
                ++stats_.inbound_robot_odometry_samples;
            } else {
                ++stats_.inbound_measurements_dropped;
            }
            break;
        }
        case MessageType::CameraTriggerEvent: {
            const auto decoded = decodeCameraTriggerEventPayload(frame.payload);
            if (!decoded) {
                std::lock_guard<std::mutex> g(mu_);
                ++stats_.invalid_payloads;
                return;
            }

            CameraTriggerEvent event;
            event.timestamp = timestampFromTeensyTime(decoded->teensy_time_us, now);
            event.teensy_time_us = decoded->teensy_time_us;
            event.pin = decoded->pin;
            event.trigger_sequence = decoded->trigger_sequence;
            event.status_flags = decoded->status_flags;
            bool time_sync_established = false;
            {
                std::lock_guard<std::mutex> g(mu_);
                time_sync_established = stats_.time_sync_established;
            }
            if (!time_sync_established) {
                event.status_flags |= kStatusUnsynchronizedTime;
            }
            if (trigger_cache_) {
                trigger_cache_->recordEvent(event);
            }
            const bool published = measurement_sink_.publish(event);
            std::lock_guard<std::mutex> g(mu_);
            if (published) {
                ++stats_.inbound_camera_trigger_events;
            } else {
                ++stats_.inbound_measurements_dropped;
            }
            break;
        }
        case MessageType::ConfigAck: {
            const auto decoded = decodeConfigAckPayload(frame.payload);
            if (!decoded) {
                std::lock_guard<std::mutex> g(mu_);
                ++stats_.invalid_payloads;
                return;
            }
            std::lock_guard<std::mutex> g(mu_);
            if (decoded->kind ==
                static_cast<std::uint32_t>(ConfigCommandKind::CameraTriggers)) {
                stats_.last_trigger_ack = *decoded;
            } else if (decoded->kind ==
                       static_cast<std::uint32_t>(ConfigCommandKind::ImuConfig)) {
                stats_.last_imu_ack = *decoded;
            }
            break;
        }
        case MessageType::TimeSyncResponse: {
            const auto decoded = decodeTimeSyncResponsePayload(frame.payload);
            if (!decoded) {
                std::lock_guard<std::mutex> g(mu_);
                ++stats_.invalid_payloads;
                return;
            }
            handleTimeSyncResponse(*decoded, now);
            break;
        }
        case MessageType::TeensyHealth: {
            const auto decoded = decodeTeensyHealthPayload(frame.payload);
            if (!decoded) {
                std::lock_guard<std::mutex> g(mu_);
                ++stats_.invalid_payloads;
                break;
            }
            std::lock_guard<std::mutex> g(mu_);
            stats_.rio_to_host_offset_us = decoded->rio_offset_us;
            stats_.rio_time_sync_established =
                (decoded->rio_status_flags & kHealthRioUnsynchronized) == 0u;
            break;
        }
        default:
            break;
    }
}

void TeensyService::sendTimeSyncRequest(ISerialTransport& transport, Timestamp now) {
    TimeSyncRequestPayload payload;
    payload.request_sequence = next_time_sync_sequence_++;
    payload.host_send_time_us = steadyMicros(now);

    {
        std::lock_guard<std::mutex> g(mu_);
        pending_time_sync_sequence_ = payload.request_sequence;
        pending_time_sync_host_send_us_ = payload.host_send_time_us;
    }

    Frame frame;
    frame.type = MessageType::TimeSyncRequest;
    frame.sequence = next_sequence_++;
    frame.payload = encodeTimeSyncRequestPayload(payload);
    transport.write(encodeFrame(frame));

    std::lock_guard<std::mutex> g(mu_);
    ++stats_.outbound_frames_sent;
    stats_.last_transmit_time = std::chrono::steady_clock::now();
}

void TeensyService::handleTimeSyncResponse(
    const TimeSyncResponsePayload& payload,
    Timestamp host_receive) {
    const std::uint64_t host_receive_us = steadyMicros(host_receive);
    std::lock_guard<std::mutex> g(mu_);
    if (!pending_time_sync_sequence_ ||
        *pending_time_sync_sequence_ != payload.request_sequence) {
        ++stats_.invalid_payloads;
        return;
    }

    const std::uint64_t host_send_us = pending_time_sync_host_send_us_;
    const std::uint64_t round_trip_us =
        host_receive_us >= host_send_us ? host_receive_us - host_send_us : 0;
    const auto host_midpoint =
        static_cast<std::int64_t>((host_send_us / 2u) + (host_receive_us / 2u));
    const auto teensy_midpoint =
        static_cast<std::int64_t>(
            (payload.teensy_receive_time_us / 2u) +
            (payload.teensy_transmit_time_us / 2u));
    const std::int64_t offset_us = host_midpoint - teensy_midpoint;

    pending_time_sync_sequence_.reset();

    TimeSyncFilter::Sample new_sample;
    new_sample.teensy_midpoint_us = teensy_midpoint;
    new_sample.offset_us = offset_us;
    new_sample.round_trip_us = round_trip_us;

    const bool accepted = time_sync_filter_.update(new_sample);
    if (!accepted) {
        ++stats_.time_sync_samples_rejected;
        return;
    }
    stats_.time_sync_offset_us = time_sync_filter_.offsetUs();
    stats_.time_sync_round_trip_us = time_sync_filter_.lastRoundTripUs();
    stats_.time_sync_skew_ppm = time_sync_filter_.skewPpm();
    stats_.time_sync_samples_accepted = time_sync_filter_.accepted();
    stats_.time_sync_established = time_sync_filter_.established();
}

Timestamp TeensyService::timestampFromTeensyTime(
    std::uint64_t teensy_time_us,
    Timestamp fallback) {
    std::lock_guard<std::mutex> g(mu_);
    if (!time_sync_filter_.established()) {
        return fallback;
    }
    const std::int64_t host_time_us =
        time_sync_filter_.apply(static_cast<std::int64_t>(teensy_time_us));
    if (host_time_us < 0) {
        return fallback;
    }
    return Timestamp{std::chrono::microseconds(host_time_us)};
}

std::vector<std::uint8_t> TeensyService::encodeCameraTriggerConfigPayload() const {
    std::vector<std::uint8_t> payload;
    payload.reserve(8 + camera_triggers_.size() * 28);
    appendU32(payload, static_cast<std::uint32_t>(ConfigCommandKind::CameraTriggers));
    appendU32(payload, static_cast<std::uint32_t>(camera_triggers_.size()));
    for (const auto& trigger : camera_triggers_) {
        appendU32(payload, trigger.enabled ? 1u : 0u);
        appendU32(payload, static_cast<std::uint32_t>(trigger.teensy_pin));
        appendDouble(payload, trigger.rate_hz);
        appendU32(payload, trigger.pulse_width_us);
        appendI64(payload, trigger.phase_offset_us);
    }
    return payload;
}

void TeensyService::sendCameraTriggerConfig(ISerialTransport& transport) {
    Frame frame;
    frame.type = MessageType::ConfigCommand;
    frame.sequence = next_sequence_++;
    frame.payload = encodeCameraTriggerConfigPayload();
    transport.write(encodeFrame(frame));

    std::lock_guard<std::mutex> g(mu_);
    ++stats_.outbound_frames_sent;
    stats_.last_transmit_time = std::chrono::steady_clock::now();
}

std::vector<std::uint8_t> TeensyService::encodeImuConfigCommandPayload() const {
    std::vector<std::uint8_t> payload;
    appendU32(payload, static_cast<std::uint32_t>(ConfigCommandKind::ImuConfig));
    ImuConfigPayload imu;
    imu.accel_range_g = config_.imu.accel_range_g;
    imu.accel_odr_hz = config_.imu.accel_odr_hz;
    imu.accel_bandwidth_code = config_.imu.accel_bandwidth_code;
    imu.gyro_range_dps = config_.imu.gyro_range_dps;
    imu.gyro_bandwidth_code = config_.imu.gyro_bandwidth_code;
    imu.data_sync_rate_hz = config_.imu.data_sync_rate_hz;
    imu.run_selftest_on_boot = config_.imu.run_selftest_on_boot ? 1u : 0u;
    auto body = encodeImuConfigPayload(imu);
    payload.insert(payload.end(), body.begin(), body.end());
    return payload;
}

void TeensyService::sendImuConfig(ISerialTransport& transport) {
    Frame frame;
    frame.type = MessageType::ConfigCommand;
    frame.sequence = next_sequence_++;
    frame.payload = encodeImuConfigCommandPayload();
    transport.write(encodeFrame(frame));

    std::lock_guard<std::mutex> g(mu_);
    ++stats_.outbound_frames_sent;
    stats_.last_transmit_time = std::chrono::steady_clock::now();
}

void TeensyService::markDisconnected(const std::string& error) {
    std::lock_guard<std::mutex> g(mu_);
    if (stats_.connected) {
        ++stats_.disconnects;
    }
    stats_.connected = false;
    stats_.last_error = error;
}

void TeensyService::sleepUntilReconnectOrStop() {
    std::unique_lock<std::mutex> lock(mu_);
    cv_.wait_for(
        lock,
        std::chrono::milliseconds(config_.reconnect_interval_ms),
        [&] { return stop_requested_; });
}

std::uint64_t TeensyService::steadyMicros(Timestamp timestamp) {
    const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
        timestamp.time_since_epoch());
    return static_cast<std::uint64_t>(micros.count());
}

std::vector<std::uint8_t> TeensyService::encodeFusedPosePayload(
    const FusedPoseEstimate& estimate) {
    std::vector<std::uint8_t> payload;
    payload.reserve(4 + 3 * sizeof(double));
    appendDouble(payload, estimate.field_to_robot.x_m);
    appendDouble(payload, estimate.field_to_robot.y_m);
    appendDouble(payload, estimate.field_to_robot.theta_rad);
    appendU32(payload, estimate.status_flags);
    return payload;
}

}  // namespace posest::teensy
