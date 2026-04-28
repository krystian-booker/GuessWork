#include "posest/teensy/TeensyService.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

#include "posest/CameraTriggerCache.h"
#include "posest/ToFSampleCache.h"

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

void appendU64(std::vector<std::uint8_t>& out, std::uint64_t value) {
    for (int shift = 0; shift <= 56; shift += 8) {
        out.push_back(static_cast<std::uint8_t>(
            (value >> static_cast<unsigned>(shift)) & 0xFFu));
    }
}

}  // namespace

TeensyService::TeensyService(
    runtime::TeensyConfig config,
    std::vector<runtime::CameraTriggerConfig> camera_triggers,
    IMeasurementSink& measurement_sink,
    SerialTransportFactory transport_factory,
    std::shared_ptr<CameraTriggerCache> trigger_cache,
    runtime::VioConfig vio_config,
    std::shared_ptr<ToFSampleCache> tof_cache)
    : config_(std::move(config)),
      camera_triggers_(std::move(camera_triggers)),
      vio_config_(std::move(vio_config)),
      measurement_sink_(measurement_sink),
      transport_factory_(std::move(transport_factory)),
      trigger_cache_(std::move(trigger_cache)),
      tof_cache_(std::move(tof_cache)) {
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
    // F-6: stamp the encode moment in steady_clock micros so the firmware can
    // diff host_send_time_us against its time-synced view at CAN-TX.
    const std::uint64_t host_send_time_us = steadyMicros(
        std::chrono::steady_clock::now());
    Frame frame;
    frame.type = MessageType::FusedPose;
    frame.sequence = next_sequence_++;
    frame.payload = encodeFusedPosePayload(estimate, host_send_time_us);
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
            if (vio_config_.enabled) {
                sendVioCompanionConfig(*current_transport_);
            }
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
            sample.teensy_time_us = decoded->teensy_time_us;
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
        case MessageType::ChassisSpeeds: {
            const auto decoded = decodeChassisSpeedsPayload(frame.payload);
            if (!decoded) {
                std::lock_guard<std::mutex> g(mu_);
                ++stats_.invalid_payloads;
                return;
            }

            // The canonical timestamp for a chassis-speed measurement is the
            // RoboRIO FPGA instant when kinematics was computed, mapped into
            // host steady_clock via two offsets:
            //   teensy_time = rio_time + rio_to_teensy_offset    (CanBridge EMA)
            //   host_time   = TimeSyncFilter::apply(teensy_time) (host EMA + skew)
            // Both legs must be established before publishing; we drop samples
            // during the bootstrap window rather than fall back on Teensy-side
            // receive time (which would mix CAN bus latency into the factor).
            Timestamp host_timestamp;
            {
                std::lock_guard<std::mutex> g(mu_);
                if (!time_sync_filter_.established() ||
                    !stats_.rio_time_sync_established) {
                    ++stats_.inbound_chassis_speeds_dropped_pre_sync;
                    return;
                }
                const std::int64_t teensy_time_signed =
                    static_cast<std::int64_t>(decoded->rio_time_us) +
                    stats_.rio_to_teensy_offset_us;
                const std::int64_t host_time_us =
                    time_sync_filter_.apply(teensy_time_signed);
                host_timestamp = Timestamp{std::chrono::microseconds(host_time_us)};
            }

            ChassisSpeedsSample sample;
            sample.timestamp = host_timestamp;
            sample.vx_mps = decoded->vx_mps;
            sample.vy_mps = decoded->vy_mps;
            sample.omega_radps = decoded->omega_radps;
            sample.rio_time_us = decoded->rio_time_us;
            sample.status_flags = decoded->status_flags;
            const bool published = measurement_sink_.publish(sample);
            std::lock_guard<std::mutex> g(mu_);
            if (published) {
                ++stats_.inbound_chassis_speeds_samples;
            } else {
                ++stats_.inbound_measurements_dropped;
            }
            break;
        }
        case MessageType::ToFSample: {
            const auto decoded = decodeToFSamplePayload(frame.payload);
            if (!decoded) {
                std::lock_guard<std::mutex> g(mu_);
                ++stats_.invalid_payloads;
                return;
            }

            ToFSample sample;
            sample.timestamp = timestampFromTeensyTime(decoded->teensy_time_us, now);
            sample.teensy_time_us = decoded->teensy_time_us;
            sample.trigger_sequence = decoded->trigger_sequence;
            // Raw chip distance in meters (mm → m); pre-mounting-offset.
            sample.raw_distance_m = static_cast<double>(decoded->distance_mm) * 1e-3;
            sample.signal_rate_kcps = static_cast<double>(decoded->signal_rate_kcps);
            sample.ambient_rate_kcps = static_cast<double>(decoded->ambient_rate_kcps);
            sample.ranging_duration_us = decoded->ranging_duration_us;
            sample.range_status = decoded->range_status;
            sample.status_flags = decoded->firmware_status_flags;

            // Apply host-side mounting offset and clamp to expected range.
            // Separation of concerns: firmware reports raw chip output; the
            // mounting calibration lives in host config so a remount only
            // touches SQLite, not firmware.
            double corrected =
                sample.raw_distance_m + vio_config_.tof_mounting_offset_m;
            bool clamped = false;
            if (corrected < vio_config_.tof_expected_min_m) {
                corrected = vio_config_.tof_expected_min_m;
                clamped = true;
            } else if (corrected > vio_config_.tof_expected_max_m) {
                corrected = vio_config_.tof_expected_max_m;
                clamped = true;
            }
            if (clamped) {
                sample.status_flags |= kToFStatusClampedHostSide;
            }
            sample.distance_m = corrected;

            bool time_sync_established = false;
            {
                std::lock_guard<std::mutex> g(mu_);
                time_sync_established = stats_.time_sync_established;
            }
            if (!time_sync_established) {
                sample.status_flags |= kStatusUnsynchronizedTime;
            }

            // Dual-publish, mirroring the CameraTriggerEvent path: bus for
            // FusionService and any other generic consumers, cache for the
            // ProducerBase per-frame join used by the VIO pipeline.
            if (tof_cache_) {
                tof_cache_->recordSample(sample);
            }
            const bool published = measurement_sink_.publish(sample);
            std::lock_guard<std::mutex> g(mu_);
            if (published) {
                ++stats_.inbound_tof_samples;
            } else {
                ++stats_.inbound_measurements_dropped;
            }
            if (clamped) {
                ++stats_.inbound_tof_clamped;
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
            } else if (decoded->kind ==
                       static_cast<std::uint32_t>(ConfigCommandKind::VioCompanion)) {
                stats_.last_vio_ack = *decoded;
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
            stats_.rio_to_teensy_offset_us = decoded->rio_offset_us;
            stats_.rio_time_sync_established =
                (decoded->rio_status_flags & kHealthRioUnsynchronized) == 0u;
            stats_.fused_pose_decode_to_tx_min_us =
                decoded->fused_pose_decode_to_tx_min_us;
            stats_.fused_pose_decode_to_tx_avg_us =
                decoded->fused_pose_decode_to_tx_avg_us;
            stats_.fused_pose_decode_to_tx_max_us =
                decoded->fused_pose_decode_to_tx_max_us;
            stats_.fused_pose_latency_samples =
                decoded->fused_pose_latency_samples;
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

std::vector<std::uint8_t> TeensyService::encodeVioCompanionConfigCommandPayload() const {
    std::vector<std::uint8_t> payload;
    appendU32(payload, static_cast<std::uint32_t>(ConfigCommandKind::VioCompanion));
    VioCompanionConfigPayload body;
    body.vio_slot_index = static_cast<std::uint32_t>(vio_config_.vio_slot_index);
    body.led_enabled = vio_config_.ir_led_enabled ? 1u : 0u;
    body.led_pulse_width_us = vio_config_.ir_led_pulse_width_us;
    body.tof_enabled = vio_config_.tof_enabled ? 1u : 0u;
    body.tof_i2c_address = vio_config_.tof_i2c_address;
    body.tof_timing_budget_ms = vio_config_.tof_timing_budget_ms;
    body.tof_intermeasurement_period_ms = vio_config_.tof_intermeasurement_period_ms;
    body.tof_offset_after_flash_us = vio_config_.tof_offset_after_flash_us;
    body.tof_divisor = vio_config_.tof_divisor;
    auto wire = encodeVioCompanionConfigPayload(body);
    payload.insert(payload.end(), wire.begin(), wire.end());
    return payload;
}

void TeensyService::sendVioCompanionConfig(ISerialTransport& transport) {
    Frame frame;
    frame.type = MessageType::ConfigCommand;
    frame.sequence = next_sequence_++;
    frame.payload = encodeVioCompanionConfigCommandPayload();
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
    return encodeFusedPosePayload(estimate, /*host_send_time_us=*/0u);
}

std::vector<std::uint8_t> TeensyService::encodeFusedPosePayload(
    const FusedPoseEstimate& estimate,
    std::uint64_t host_send_time_us) {
    // v4 wire layout. The fields below must stay 1:1 with
    // firmware/teensy41/src/Protocol.cpp::decodeFusedPosePayload (376 bytes):
    // host_send_time_us (8) + Pose3 (48) + velocity (24) + has_velocity (4) +
    // status (4) + covariance (288) = 376. The leading host_send_time_us is
    // F-6's end-to-end latency probe — the firmware compares it against
    // micros() at CAN-FD-TX (with the host↔Teensy time-sync offset applied)
    // to surface decode-to-TX latency in TeensyHealthPayload.
    std::vector<std::uint8_t> payload;
    payload.reserve(376);
    appendU64(payload, host_send_time_us);
    // Pose3
    appendDouble(payload, estimate.field_to_robot.x_m);
    appendDouble(payload, estimate.field_to_robot.y_m);
    appendDouble(payload, 0.0);  // z
    appendDouble(payload, 0.0);  // roll
    appendDouble(payload, 0.0);  // pitch
    appendDouble(payload, estimate.field_to_robot.theta_rad);  // yaw
    // Velocity (has_velocity=0 means RIO ignores the bytes)
    const bool has_velocity = estimate.velocity.has_value();
    appendDouble(payload, has_velocity ? estimate.velocity->x : 0.0);
    appendDouble(payload, has_velocity ? estimate.velocity->y : 0.0);
    appendDouble(payload, has_velocity ? estimate.velocity->z : 0.0);
    appendU32(payload, has_velocity ? 1u : 0u);
    appendU32(payload, estimate.status_flags);
    // Covariance — gtsam Pose3 tangent order, row-major 6×6.
    for (std::size_t i = 0; i < estimate.covariance.size(); ++i) {
        appendDouble(payload, estimate.covariance[i]);
    }
    return payload;
}

}  // namespace posest::teensy
