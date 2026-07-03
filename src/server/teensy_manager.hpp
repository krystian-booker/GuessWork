#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "core/imu_types.hpp"
#include "core/measurement_bus.hpp"
#include "producer/pulse_stamper.hpp"

namespace gw::server {

// Owns the USB serial connections to the Teensy 4.1 trigger device:
//   - discovers /dev/cu.usbmodem* and identifies the ASCII command interface
//     by PING/PONG probe (fw≥2 enumerates two CDC interfaces),
//   - pushes group config + ARM/STOP on the command interface,
//   - receives TRIG events and routes them into per-output-pin pulse rings
//     that the SpinnakerProducer consumes to re-stamp hw-sync frames,
//   - reads the binary telemetry interface (IMU batches, heartbeats — see
//     src/server/telemetry_decoder.hpp) and publishes to imu_bus(),
//   - maintains the host↔Teensy clock mapping (core/clock_sync.hpp) from
//     the telemetry/TRIG arrival stamps; RobotLink chains it with its own
//     RIO↔host sync to put chassis speeds on the Teensy clock (robot
//     communication itself is UDP — docs/ethernet-protocol.md).
//
// The host is the source of truth for configuration: on every (re)connect the
// manager re-pushes whatever the most recent push_config() set.
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

        // Host ↔ Teensy clock sync (core/clock_sync.hpp), fed by IMU/TRIG
        // arrival stamps. RobotLink chains this with its RIO↔host sync.
        bool     sync_healthy   = false;
        double   sync_offset_us = 0.0;  // host − teensy at the latest sample
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

    // Host↔Teensy clock mapping (thread-safe snapshots of the internal
    // ClockSync, fed at up to 400 Hz by IMU/TRIG arrival stamps). nullopt
    // while the fit is unhealthy — callers fall back or drop. RobotLink
    // chains these with its RIO↔host sync so chassis speeds land on the
    // Teensy clock without any direct RIO↔Teensy link.
    std::optional<uint64_t> host_to_teensy_ns(uint64_t host_ns) const;
    std::optional<uint64_t> teensy_to_host_ns(uint64_t teensy_ns) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace gw::server
