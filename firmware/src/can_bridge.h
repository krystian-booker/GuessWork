#pragma once

#include <Arduino.h>

#include "can_payloads.h"

// CAN bridge on CAN3 (pin 30 = CRX3, pin 31 = CTX3 — the only FD-capable
// FlexCAN on Teensy 4.x), TJA1051T/3 transceiver. Receives chassis-speeds
// frames from the robot controller, stamps arrival on the wrap-extended
// 64-bit clock, and forwards them as ODOM telemetry on SerialUSB1; transmits
// the fused field pose back onto the bus. Frame layouts and arbitration IDs:
// can_payloads.h / docs/can-protocol.md.
//
// Two runtime modes (host-selected via the CAN_MODE command, boots Off):
//   Classic — CAN 2.0B @ 1 Mbps (RoboRIO), two-frame splits
//   Fd      — CAN FD, 1 Mbps arbitration / 4 Mbps data (SystemCore)
//
// ISR/loop split mirrors bmi088_imu/trigger_engine: the RX interrupt only
// stamps + copies the frame into an SPSC ring; poll() (main loop) does the
// decode, classic STAMP/SPEEDS pairing, and USB write.

namespace gw_fw {

enum class CanMode : uint8_t {
    Off     = canp::kModeOff,
    Classic = canp::kModeClassic,
    Fd      = canp::kModeFd,
};

class CanBridge {
public:
    // Idempotent; CAN stays off until set_mode(). Call once in setup().
    void begin();

    // Reconfigure the controller for `mode`. Returns false with `err` set on
    // failure (bad FD timings). Switching between live modes is best-effort —
    // see the _CAN3 dispatch note in can_bridge.cpp; a power cycle is the
    // documented fallback (the host re-pushes the mode on reconnect).
    bool set_mode(CanMode mode, const char*& err);

    // Drain the RX ring: pair classic STAMP/SPEEDS, emit ODOM telemetry
    // frames on SerialUSB1. Call every loop iteration.
    void poll();

    // Transmit a pose onto the bus (one FD frame, or the classic XY/THETA
    // pair). No-op (returns false) when mode is Off.
    bool send_pose(const canp::PoseWire& pose);

    CanMode mode() const { return mode_; }
    // Mode is active and the controller configured successfully.
    bool ok() const { return mode_ != CanMode::Off && configured_; }

    uint32_t rx_count() const;        // accepted chassis-speeds samples
    uint32_t rx_drops() const;        // ring overflows + stamp mismatches
    uint32_t odom_tx_drops() const;   // ODOM frames lost to USB backpressure
    uint32_t pose_tx() const;         // pose transmissions onto the bus
    uint32_t pose_tx_drops() const;   // pose write() failures

private:
    CanMode mode_       = CanMode::Off;
    bool    configured_ = false;
};

}  // namespace gw_fw
