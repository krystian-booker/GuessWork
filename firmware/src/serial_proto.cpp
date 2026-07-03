#include "serial_proto.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

namespace gw_fw {

namespace {

// Walk a "key=value" token. On entry, `tok` points at the start of a token.
// On success, fills `key` and `value` with NUL-terminated substrings (in place)
// and returns true. The caller has guaranteed the token has already been
// strtok_r'd out, so it contains no whitespace.
bool split_kv(char* tok, const char*& key, const char*& value) {
    char* eq = strchr(tok, '=');
    if (!eq) return false;
    *eq   = '\0';
    key   = tok;
    value = eq + 1;
    return true;
}

// Parse a comma-separated list of pin numbers (1..6) into a bitmask. Returns
// false on any malformed entry. Empty list returns 0.
bool parse_pins_csv(const char* csv, uint8_t& mask) {
    mask = 0;
    if (!csv || !*csv) return false;
    char buf[32];
    strncpy(buf, csv, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char* save = nullptr;
    for (char* part = strtok_r(buf, ",", &save); part; part = strtok_r(nullptr, ",", &save)) {
        char* end = nullptr;
        long n = strtol(part, &end, 10);
        if (end == part || *end != '\0') return false;
        if (n < 1 || n > 6) return false;
        const uint8_t bit = 1u << (n - 1);
        if (mask & bit) return false;  // duplicate
        mask |= bit;
    }
    return mask != 0;
}

}  // namespace

void SerialProto::begin() {
    Serial.print(F("READY fw="));
    Serial.print(kFirmwareVersion);
    Serial.print(F(" outputs="));
    Serial.println(kMaxOutputs);
}

void SerialProto::poll() {
    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (c < 0) break;
        if (c == '\r') continue;
        if (c == '\n') {
            buf_[buf_len_] = '\0';
            if (buf_len_ > 0) handle_line(buf_);
            buf_len_ = 0;
            continue;
        }
        if (buf_len_ + 1 < kSerialLineBufLen) {
            buf_[buf_len_++] = static_cast<char>(c);
        } else {
            // Overrun: discard the whole pending line and complain.
            buf_len_ = 0;
            Serial.println(F("ERR line too long"));
        }
    }
}

void SerialProto::handle_line(char* line) {
    // Split off the verb. strtok_r is fine here — we just mangled buf_ which
    // is reset on every '\n'.
    char* save = nullptr;
    char* verb = strtok_r(line, " ", &save);
    if (!verb) return;
    // Uppercase the verb so we accept "ping" / "Ping" / "PING" alike.
    for (char* p = verb; *p; ++p) *p = static_cast<char>(toupper(*p));
    char* args = save;  // remainder of the line, may be null

    if (strcmp(verb, "PING") == 0) {
        Serial.print(F("PONG fw="));
        Serial.println(kFirmwareVersion);
        return;
    }
    if (strcmp(verb, "STATUS") == 0) { handle_status();           return; }
    if (strcmp(verb, "CFG_CLEAR") == 0) {
        engine_.clear_config();
        Serial.println(F("OK"));
        return;
    }
    if (strcmp(verb, "CFG") == 0) { handle_cfg(args);             return; }
    if (strcmp(verb, "ARM") == 0) {
        const char* err = nullptr;
        if (engine_.arm(err)) Serial.println(F("OK"));
        else { Serial.print(F("ERR ")); Serial.println(err ? err : "arm failed"); }
        return;
    }
    if (strcmp(verb, "STOP") == 0) {
        engine_.stop();
        Serial.println(F("OK"));
        return;
    }
    Serial.print(F("ERR unknown command: "));
    Serial.println(verb);
}

void SerialProto::handle_cfg(char* args) {
    if (!args) {
        Serial.println(F("ERR CFG requires name=<g> fps=<f> pins=<csv>"));
        return;
    }
    const char* name = nullptr;
    float       fps  = 0.0f;
    uint8_t     mask = 0;
    bool        have_name = false, have_fps = false, have_pins = false;

    char* save = nullptr;
    for (char* tok = strtok_r(args, " ", &save); tok; tok = strtok_r(nullptr, " ", &save)) {
        const char* key   = nullptr;
        const char* value = nullptr;
        if (!split_kv(tok, key, value)) {
            Serial.print(F("ERR malformed token: "));
            Serial.println(tok);
            return;
        }
        if (strcmp(key, "name") == 0) { name = value;                have_name = true; }
        else if (strcmp(key, "fps") == 0) { fps = strtof(value, nullptr); have_fps  = true; }
        else if (strcmp(key, "pins") == 0) {
            if (!parse_pins_csv(value, mask)) {
                Serial.print(F("ERR bad pins: "));
                Serial.println(value);
                return;
            }
            have_pins = true;
        } else {
            Serial.print(F("ERR unknown key: "));
            Serial.println(key);
            return;
        }
    }
    if (!(have_name && have_fps && have_pins)) {
        Serial.println(F("ERR CFG requires name, fps, pins"));
        return;
    }
    const char* err = nullptr;
    if (engine_.set_config(name, fps, mask, err)) {
        Serial.println(F("OK"));
    } else {
        Serial.print(F("ERR "));
        Serial.println(err ? err : "config rejected");
    }
}

void SerialProto::handle_status() {
    Serial.print(F("STATUS armed="));
    Serial.print(engine_.is_armed() ? 1 : 0);
    Serial.print(F(" groups="));
    Serial.println(engine_.group_count());
    for (int i = 0; i < engine_.group_count(); ++i) {
        Serial.print(F("GROUP name="));
        Serial.print(engine_.group_name(i));
        Serial.print(F(" fps="));
        Serial.print(engine_.group_fps(i), 3);
        Serial.print(F(" pins="));
        const uint8_t m = engine_.group_pin_mask(i);
        bool first = true;
        for (int p = 1; p <= kMaxOutputs; ++p) {
            if (m & (1u << (p - 1))) {
                if (!first) Serial.print(',');
                Serial.print(p);
                first = false;
            }
        }
        Serial.println();
    }
    Serial.println(F("OK"));
}

void SerialProto::flush_pulse_events() {
    // Worst-case TRIG line: "TRIG g=" + name + " idx=" + u32 + " t_us=" +
    // u64 + CRLF. USB CDC writes with a full TX buffer BLOCK, and this runs
    // in the main loop — a host that stops draining the command port must
    // not stall IMU polling or trigger stamping. Deferred events stay in the
    // engine's ISR ring; if the host is truly gone the ring overwrites
    // oldest, which loses nothing the host would have read anyway.
    constexpr int kTrigLineMaxBytes = 64;
    for (int i = 0; i < engine_.group_count(); ++i) {
        PulseEvent e;
        while (Serial.availableForWrite() >= kTrigLineMaxBytes &&
               engine_.drain_event(i, e)) {
            Serial.print(F("TRIG g="));
            Serial.print(engine_.group_name(i));
            Serial.print(F(" idx="));
            Serial.print(e.idx);
            Serial.print(F(" t_us="));
            Serial.println(e.t_us);
        }
    }
}

}  // namespace gw_fw
