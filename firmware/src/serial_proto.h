#pragma once

#include <Arduino.h>

#include "trigger_engine.h"

// Line-based ASCII protocol over the Teensy's USB-CDC `Serial` interface.
//
// Host → Teensy commands (one per line, '\n' terminated, '\r' ignored):
//   PING                                       -> "PONG fw=1"
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
//   READY fw=1 outputs=6
//
// Parsing is allocation-free: a single 256-byte line buffer accumulates the
// command, then a small inline tokenizer reads `key=value` pairs.

namespace gw_fw {

constexpr int     kSerialLineBufLen = 256;
constexpr uint8_t kFirmwareVersion  = 1;

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

    TriggerEngine& engine_;
    char           buf_[kSerialLineBufLen];
    int            buf_len_ = 0;
};

}  // namespace gw_fw
