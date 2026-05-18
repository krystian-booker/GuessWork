// Teensy 4.1 hardware-sync trigger.
//
// Drives up to 4 named, independently-configured trigger groups on outputs
// 1..6 (Teensy physical pins 2..7) and streams a TRIG line per rising edge
// over USB-CDC. The host (`TeensyManager` on the macOS side) is the source
// of truth for configuration; this firmware boots quiescent and waits for
// CFG + ARM before producing any pulses.
//
// See firmware/src/serial_proto.h for the on-wire protocol and
// firmware/src/trigger_engine.h for the IntervalTimer + ring-buffer setup.

#include <Arduino.h>

#include "trigger_engine.h"
#include "serial_proto.h"

namespace {

gw_fw::TriggerEngine engine;
gw_fw::SerialProto   proto(engine);

}  // namespace

void setup() {
    Serial.begin(1'000'000);  // baud is ignored over USB-CDC
    engine.begin();
    // Host needs a beat to open the port before the greeting, otherwise the
    // first byte is often dropped on first boot.
    delay(50);
    proto.begin();
}

void loop() {
    proto.poll();
    proto.flush_pulse_events();
}
