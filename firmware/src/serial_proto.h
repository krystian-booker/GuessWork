#pragma once

#include <Arduino.h>

#include "can_bridge.h"
#include "trigger_engine.h"

// Line-based ASCII protocol over the Teensy's USB-CDC `Serial` interface.
//
// Host → Teensy commands (one per line, '\n' terminated, '\r' ignored):
//   PING                                       -> "PONG fw=3"
//   STATUS                                     -> one or more lines, ends with "OK"
//   CFG_CLEAR                                  -> "OK" (stops first if armed)
//   CFG name=<g> fps=<f> pins=<1,3,4>          -> "OK" or "ERR <msg>"
//   ARM                                        -> "OK" or "ERR <msg>"
//   STOP                                       -> "OK"
//   CAN_MODE mode=off|classic|fd               -> "OK" or "ERR <msg>"
//
// Teensy → host streams (after ARM):
//   TRIG g=<name> idx=<n> t_us=<u>            -> one per rising edge per group
//
// Boot greeting:
//   READY fw=3 outputs=6
//
// fw=2 changes vs fw=1: t_us in TRIG lines is a wrap-extended 64-bit
// microsecond value (see time64.h) instead of raw 32-bit micros(); binary
// telemetry (IMU/odometry) streams on the second USB-CDC interface (see
// binary_proto.h).
// fw=3 changes vs fw=2: CAN bridge (CAN_MODE command, `CAN mode=… ok=…`
// STATUS line, ODOM 0x03 + host→Teensy POSE 0x10 telemetry packets, the
// HEARTBEAT payload grows 17 → 34 bytes — see binary_proto.h and
// docs/can-protocol.md).
//
// Parsing is allocation-free: a single 256-byte line buffer accumulates the
// command, then a small inline tokenizer reads `key=value` pairs.

namespace gw_fw {

constexpr int     kSerialLineBufLen = 256;
constexpr uint8_t kFirmwareVersion  = 3;

class SerialProto {
public:
    SerialProto(TriggerEngine& engine, CanBridge& can)
        : engine_(engine), can_(can) {}

    void begin();   // emits the READY greeting
    void poll();    // drain incoming serial, parse complete lines
    void flush_pulse_events();  // emit TRIG lines for any queued ISR events

private:
    void handle_line(char* line);  // mutates `line` (in-place tokenisation)
    void handle_cfg(char* args);
    void handle_can_mode(char* args);
    void handle_status();

    TriggerEngine& engine_;
    CanBridge&     can_;
    char           buf_[kSerialLineBufLen];
    int            buf_len_ = 0;
};

}  // namespace gw_fw
