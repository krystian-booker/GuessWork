#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "core/imu_types.hpp"
#include "core/measurement_bus.hpp"
#include "core/odom_types.hpp"
#include "producer/pulse_stamper.hpp"

namespace gw::server {

// Host-side mirror of the firmware CAN modes (can_bridge.h). DB mapping:
// 'roborio' = Classic, 'systemcore' = Fd.
enum class CanMode : uint8_t { Off = 0, Classic = 1, Fd = 2 };

// Owns the USB serial connections to the Teensy 4.1 trigger device:
//   - discovers /dev/cu.usbmodem* and identifies the ASCII command interface
//     by PING/PONG probe (fw≥2 enumerates two CDC interfaces),
//   - pushes group config + ARM/STOP and the CAN mode on the command
//     interface,
//   - receives TRIG events and routes them into per-output-pin pulse rings
//     that the SpinnakerProducer consumes to re-stamp hw-sync frames,
//   - reads the binary telemetry interface (IMU batches, CAN-forwarded
//     chassis speeds, heartbeats — see src/server/telemetry_decoder.hpp) and
//     publishes to imu_bus() / odom_bus(); chassis-speeds samples get their
//     RIO timestamps mapped onto the Teensy clock by an internal
//     RioClockSync (fallback: CAN arrival stamp),
//   - writes fused-pose packets back down the telemetry interface
//     (send_pose), which the firmware relays onto the CAN bus.
//
// The host is the source of truth for configuration: on every (re)connect the
// manager re-pushes whatever the most recent push_config()/set_can_mode()
// set. The Teensy itself boots quiescent (CAN off).
class TeensyManager : public gw::IPulseStamper {
public:
    struct GroupConfig {
        std::string          name;
        double               fps;
        std::vector<uint8_t> output_pins;   // 1..6
    };

    struct Status {
        bool                       connected         = false;
        std::optional<std::string> port;             // device path when connected
        bool                       armed             = false;
        std::optional<int64_t>     last_pulse_age_ms;
        uint64_t                   total_pulses      = 0;
        std::optional<std::string> last_error;
        std::optional<int>         fw_version;       // from READY/PONG

        // Binary telemetry / IMU.
        bool                   telemetry_connected = false;
        bool                   imu_ok              = false;  // firmware heartbeat flag
        double                 imu_rate_hz         = 0.0;    // measured over ~1 s
        std::optional<int64_t> imu_last_sample_age_ms;
        uint64_t               imu_samples         = 0;      // host-decoded total
        uint64_t               imu_fw_drops        = 0;      // firmware-side drops
        uint64_t               imu_crc_errors      = 0;

        // CAN bridge (fw≥3 heartbeat + ODOM stream).
        bool    can_ok           = false;  // heartbeat flag: mode active + bus configured
        CanMode can_mode_desired = CanMode::Off;
        int     can_mode_fw      = -1;     // last heartbeat-reported mode; -1 unknown
        uint64_t can_rx           = 0;     // firmware: accepted chassis frames
        uint64_t can_rx_drops     = 0;     // firmware: ring overflows + stamp mismatches
        uint64_t odom_tx_fw_drops = 0;     // firmware: ODOM lost to USB backpressure
        uint64_t pose_tx_fw       = 0;     // firmware: pose frames onto the bus

        double                 odom_rate_hz = 0.0;  // measured over ~1 s
        uint64_t               odom_packets = 0;    // host-decoded total
        std::optional<int64_t> odom_last_age_ms;
        std::optional<gw::ChassisSpeeds> odom_last;

        uint64_t pose_sent        = 0;  // host-side send_pose successes
        uint64_t pose_send_errors = 0;

        // RIO ↔ Teensy clock sync (core/rio_clock_sync.hpp).
        bool     sync_healthy   = false;
        double   sync_offset_us = 0.0;
        double   sync_drift_ppm = 0.0;
        uint64_t sync_samples   = 0;
        uint64_t sync_resets    = 0;
    };

    TeensyManager();
    ~TeensyManager();

    TeensyManager(const TeensyManager&)            = delete;
    TeensyManager& operator=(const TeensyManager&) = delete;

    // Spawn the background I/O thread. Discovery starts immediately.
    void start();
    void stop();

    Status status() const;

    // Push the desired group config to the Teensy: CFG_CLEAR, one CFG per
    // group, ARM. The manager remembers the config and re-pushes on every
    // reconnect. Returns false (and sets err) if the Teensy isn't connected
    // or rejected one of the commands.
    bool push_config(const std::vector<GroupConfig>& groups, std::string& err);

    // Sends STOP. The remembered config is preserved, so a subsequent
    // push_config (or reconnect-triggered re-push) will resume.
    bool stop_outputs(std::string& err);

    // Wipes the remembered config and asks the Teensy to clear too.
    bool clear_config(std::string& err);

    // IPulseStamper — pop the next pulse timestamp (in nanoseconds, on the
    // Teensy clock — wrap-extended 64-bit micros * 1000) for the given
    // output pin. See the interface header for the alignment contract.
    uint64_t pop_pulse_ns(uint8_t output_pin, uint64_t camera_frame_id) override;
    void     reset_pin_state(uint8_t output_pin) override;

    // Decoded IMU samples (Teensy-clock timestamps). Subscribers: calibration
    // recording, OpenVINS, fusion. Valid for the manager's lifetime.
    gw::MeasurementBus<gw::ImuSample>& imu_bus();

    // Decoded chassis-speeds samples (Teensy-clock timestamps — mapped RIO
    // sample time when clock sync is healthy, CAN arrival otherwise).
    // Subscriber: fusion (Phase 6). Valid for the manager's lifetime.
    gw::OdomBus& odom_bus();

    // Remember the desired CAN mode and, when the Teensy is connected with
    // fw≥3, push the CAN_MODE command immediately. Returns false with err set
    // when the push didn't happen (offline / old firmware / rejected) — the
    // mode is still remembered and re-pushed on the next (re)connect.
    bool set_can_mode(CanMode mode, std::string& err);

    // Ship a fused pose to the controller: maps t_ns into the RIO clock
    // (0 = unmapped when sync is unhealthy — still sent; the controller's
    // staleness check is counter-based), frames a POSE packet and writes it
    // to the telemetry interface for the firmware to relay onto the CAN bus.
    // Fails when the telemetry interface is down.
    bool send_pose(const gw::FusedPose& pose, std::string& err);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace gw::server
