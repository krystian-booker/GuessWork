// Teensy 4.1 hardware-sync trigger + sensor bridge.
//
// Drives up to 4 named, independently-configured trigger groups on outputs
// 1..6 (Teensy physical pins 2..7) and streams a TRIG line per rising edge
// over USB-CDC. The host (`TeensyManager` on the macOS side) is the source
// of truth for configuration; this firmware boots quiescent and waits for
// CFG + ARM before producing any pulses.
//
// A BMI088 IMU on SPI0 streams 400 Hz samples — timestamped on the same
// wrap-extended 64-bit clock as the trigger pulses — as binary packets on
// the second USB-CDC interface (`SerialUSB1`), alongside a 1 Hz HEARTBEAT.
//
// A CAN bridge on CAN3 (pins 30/31, TJA1051T/3) forwards controller chassis
// speeds as ODOM telemetry and transmits the host's fused pose back onto the
// bus; the host's POSE packets arrive on SerialUSB1's RX side.
//
// See firmware/src/serial_proto.h for the ASCII protocol,
// firmware/src/binary_proto.h for the telemetry framing,
// firmware/src/can_bridge.h + docs/can-protocol.md for the CAN side, and
// firmware/src/trigger_engine.h for the IntervalTimer + ring-buffer setup.

#include <Arduino.h>

#include "bmi088_imu.h"
#include "binary_proto.h"
#include "can_bridge.h"
#include "can_payloads.h"
#include "serial_proto.h"
#include "time64.h"
#include "trigger_engine.h"

namespace {

gw_fw::TriggerEngine   engine;
gw_fw::CanBridge       can;
gw_fw::SerialProto     proto(engine, can);
gw_fw::Bmi088Streamer  imu;
gw_fw::BinRxParser     bin_rx;

uint64_t last_heartbeat_us = 0;

void send_heartbeat(uint64_t t_us) {
    uint8_t payload[8 + 1 + 4 + 4 + 4 + 4 + 4 + 4 + 1];
    size_t  off = 0;
    off = gw_fw::put_u64(payload, off, t_us);
    uint8_t flags = imu.ok() ? 0x01 : 0x00;
    if (can.ok()) flags |= 0x02;
    off = gw_fw::put_u8(payload, off, flags);
    off = gw_fw::put_u32(payload, off, imu.sample_count());
    off = gw_fw::put_u32(payload, off, imu.drop_count());
    off = gw_fw::put_u32(payload, off, can.rx_count());
    off = gw_fw::put_u32(payload, off, can.rx_drops());
    off = gw_fw::put_u32(payload, off, can.odom_tx_drops());
    off = gw_fw::put_u32(payload, off, can.pose_tx());
    off = gw_fw::put_u8(payload, off, static_cast<uint8_t>(can.mode()));

    uint8_t frame[6 + sizeof(payload)];
    const size_t n = gw_fw::build_frame(gw_fw::BinType::Heartbeat, payload,
                                        static_cast<uint8_t>(off), frame);
    if (SerialUSB1.availableForWrite() >= static_cast<int>(n)) {
        SerialUSB1.write(frame, n);
    }
}

// Drain host → Teensy bytes on the telemetry CDC; a CRC-valid POSE packet
// goes straight onto the CAN bus.
void poll_pose_downlink() {
    while (SerialUSB1.available() > 0) {
        const int c = SerialUSB1.read();
        if (c < 0) break;
        bin_rx.feed(static_cast<uint8_t>(c));
    }
    uint8_t type = 0;
    uint8_t payload[gw_fw::BinRxParser::kMaxPayload];
    uint8_t len = 0;
    if (bin_rx.take(type, payload, len)) {
        gw_fw::canp::PoseWire pose;
        if (type == static_cast<uint8_t>(gw_fw::BinType::Pose) &&
            gw_fw::canp::decode_pose_telemetry(payload, len, pose)) {
            can.send_pose(pose);
        }
    }
}

}  // namespace

void setup() {
    Serial.begin(1'000'000);      // baud is ignored over USB-CDC
    SerialUSB1.begin(1'000'000);  // binary telemetry channel
    engine.begin();
    // An absent/miswired IMU leaves the streamer inert; HEARTBEAT reports
    // imu_ok=0 so the host can tell "no IMU" from "no Teensy".
    imu.begin();
    // CAN stays off until the host pushes CAN_MODE.
    can.begin();
    // Host needs a beat to open the port before the greeting, otherwise the
    // first byte is often dropped on first boot.
    delay(50);
    proto.begin();
}

void loop() {
    // Keep the 64-bit timebase's wrap detector alive even when nothing is
    // armed — micros() rolls over every ~71.6 minutes and the extension only
    // observes a rollover when somebody reads the clock.
    const uint64_t now_us = gw_fw::now_us64();

    proto.poll();
    proto.flush_pulse_events();
    imu.poll();
    can.poll();
    poll_pose_downlink();

    if (now_us - last_heartbeat_us >= 1'000'000) {
        last_heartbeat_us = now_us;
        send_heartbeat(now_us);
    }
}
