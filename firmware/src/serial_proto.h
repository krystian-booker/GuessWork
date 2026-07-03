#pragma once

#include <Arduino.h>

#include "trigger_engine.h"

// Line-based ASCII protocol over the Teensy's USB-CDC `Serial` interface.
//
// Host → Teensy commands (one per line, '\n' terminated, '\r' ignored):
//   PING                                       -> "PONG fw=4"
//   STATUS                                     -> one or more lines, ends with "OK"
//   CFG_CLEAR                                  -> "OK" (stops first if armed)
//   CFG name=<g> fps=<f> pins=<1,3,4>          -> "OK" or "ERR <msg>"
//   ARM                                        -> "OK" or "ERR <msg>"
//   STOP                                       -> "OK"
//
// Teensy → host streams (after ARM):
//   TRIG g=<name> idx=<n> t_us=<u>            -> one per rising edge per group
//
// Boot greeting:
//   READY fw=4 outputs=6
//
// fw=2 changes vs fw=1: t_us in TRIG lines is a wrap-extended 64-bit
// microsecond value (see time64.h) instead of raw 32-bit micros(); binary
// telemetry (IMU) streams on the second USB-CDC interface (see
// binary_proto.h).
// fw=3 added a CAN bridge (mode command, extra STATUS line, ODOM/POSE
// telemetry packets, 34-byte HEARTBEAT).
// fw=4 removes it again — robot communication moved to UDP on the host (see
// docs/ethernet-protocol.md); the HEARTBEAT payload is back to the 17-byte
// fw=2 layout and the fw=3 CAN commands are unknown commands.
//
// Parsing is allocation-free: a single 256-byte line buffer accumulates the
// command, then a small inline tokenizer reads `key=value` pairs.

namespace gw_fw {

constexpr int     kSerialLineBufLen = 256;
constexpr uint8_t kFirmwareVersion  = 4;

class SerialProto {
public:
    explicit SerialProto(TriggerEngine& engine) : engine_(engine) {}

    void begin();   // emits the READY greeting
    void poll();    // drain incoming serial, parse complete lines
    void flush_pulse_events();  // emit TRIG lines for any queued ISR events

private:
    void handle_line(char* line);  // mutates `line` (in-place tokenisation)
    void handle_cfg(char* args);
    void handle_status();
    void handle_test_pin(char* args);  // bench diagnostic: steady pin drive

    TriggerEngine& engine_;
    char           buf_[kSerialLineBufLen];
    int            buf_len_ = 0;
};

}  // namespace gw_fw
