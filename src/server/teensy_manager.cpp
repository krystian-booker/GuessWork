#include "server/teensy_manager.hpp"

#include <fcntl.h>
#include <glob.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>
#include <unordered_map>

namespace gw::server {

namespace {

constexpr int  kReconnectDelaySec = 3;
constexpr int  kReadTimeoutMs     = 200;
constexpr int  kCommandAckMs      = 1000;
constexpr int  kPulseRingMax      = 256;
constexpr char kDeviceGlob[]      = "/dev/cu.usbmodem*";

// Trim ASCII whitespace from both ends.
std::string_view trim(std::string_view s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t' || s.front() == '\r')) s.remove_prefix(1);
    while (!s.empty() && (s.back()  == ' ' || s.back()  == '\t' || s.back()  == '\r')) s.remove_suffix(1);
    return s;
}

// Parse "key=value" tokens out of a line; populates out with the latest value
// for each key. Tokens are space-separated.
void parse_kv_tokens(std::string_view body, std::unordered_map<std::string, std::string>& out) {
    size_t i = 0;
    while (i < body.size()) {
        while (i < body.size() && body[i] == ' ') ++i;
        const size_t start = i;
        while (i < body.size() && body[i] != ' ') ++i;
        if (i == start) break;
        std::string_view tok = body.substr(start, i - start);
        const size_t eq = tok.find('=');
        if (eq == std::string_view::npos) continue;
        out[std::string(tok.substr(0, eq))] = std::string(tok.substr(eq + 1));
    }
}

}  // namespace

struct PinState {
    std::deque<std::pair<uint32_t, uint32_t>> pulses;  // {teensy_idx, t_us}
    std::optional<uint64_t> last_camera_frame_id;
};

struct TeensyManager::Impl {
    std::thread                io_thread;
    std::atomic<bool>          stop_flag{false};

    // I/O state — touched only by io_thread except where noted.
    int                        fd = -1;
    std::string                line_buf;
    std::optional<std::string> open_port;

    // Configuration cache (push_config payload). Re-sent after every
    // (re)connect. Guarded by cfg_mu.
    std::mutex                                  cfg_mu;
    std::vector<TeensyManager::GroupConfig>     desired_cfg;
    bool                                        want_armed = false;

    // Group → pin-mask lookup so each TRIG event can fan out to its pins.
    std::mutex                                  group_pins_mu;
    std::unordered_map<std::string, std::vector<uint8_t>> group_pins;

    // Per-pin pulse rings. Guarded by pulses_mu.
    mutable std::mutex                          pulses_mu;
    std::unordered_map<uint8_t, PinState>       pulses;

    // Async command/response coordination. The HTTP thread acquires cmd_mu,
    // writes a command, then waits on cmd_cv for the matching OK/ERR line
    // up to kCommandAckMs. The io_thread sets cmd_result + notifies.
    std::mutex                  cmd_mu;
    std::condition_variable     cmd_cv;
    bool                        cmd_pending  = false;
    std::optional<std::string>  cmd_result;     // empty on OK, message on ERR

    // Observable status.
    mutable std::mutex          status_mu;
    bool                        connected = false;
    bool                        armed     = false;
    std::chrono::steady_clock::time_point last_pulse_at{};
    bool                        last_pulse_seen = false;
    uint64_t                    total_pulses = 0;
    std::optional<std::string>  last_error;

    void run();
    bool try_connect();
    void close_fd();
    bool write_line_locked(std::string_view s, std::string& err);
    bool send_command(std::string_view cmd, std::string& err);  // takes cmd_mu
    void handle_incoming_line(std::string_view line);
    bool send_full_config(const std::vector<TeensyManager::GroupConfig>& groups,
                          bool want_armed, std::string& err);
    void resync_config();           // pushes the cfg_mu-stashed snapshot
    void note_error(std::string msg);

    void set_group_pins(const std::vector<TeensyManager::GroupConfig>& groups);
};

TeensyManager::TeensyManager()  : impl_(std::make_unique<Impl>()) {}
TeensyManager::~TeensyManager() { stop(); }

void TeensyManager::start() {
    if (impl_->io_thread.joinable()) return;
    impl_->stop_flag.store(false);
    impl_->io_thread = std::thread([this] { impl_->run(); });
}

void TeensyManager::stop() {
    if (!impl_->io_thread.joinable()) return;
    impl_->stop_flag.store(true);
    impl_->io_thread.join();
    impl_->close_fd();
}

TeensyManager::Status TeensyManager::status() const {
    Status s;
    std::lock_guard lk(impl_->status_mu);
    s.connected   = impl_->connected;
    s.port        = impl_->open_port;
    s.armed       = impl_->armed;
    s.total_pulses = impl_->total_pulses;
    s.last_error  = impl_->last_error;
    if (impl_->last_pulse_seen) {
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - impl_->last_pulse_at).count();
        s.last_pulse_age_ms = ms;
    }
    return s;
}

bool TeensyManager::push_config(const std::vector<GroupConfig>& groups, std::string& err) {
    {
        std::lock_guard lk(impl_->cfg_mu);
        impl_->desired_cfg = groups;
        impl_->want_armed  = !groups.empty();
    }
    impl_->set_group_pins(groups);

    // Clear remembered pin state — frame-id baselines must rebase after a
    // config change so the alignment stays meaningful.
    {
        std::lock_guard lk(impl_->pulses_mu);
        impl_->pulses.clear();
    }

    if (!impl_->connected) {
        err = "Teensy not connected";
        return false;
    }
    return impl_->send_full_config(groups, !groups.empty(), err);
}

bool TeensyManager::stop_outputs(std::string& err) {
    {
        std::lock_guard lk(impl_->cfg_mu);
        impl_->want_armed = false;
    }
    if (!impl_->connected) { err = "Teensy not connected"; return false; }
    const bool ok = impl_->send_command("STOP", err);
    if (ok) {
        std::lock_guard lk(impl_->status_mu);
        impl_->armed = false;
    }
    return ok;
}

bool TeensyManager::clear_config(std::string& err) {
    {
        std::lock_guard lk(impl_->cfg_mu);
        impl_->desired_cfg.clear();
        impl_->want_armed = false;
    }
    impl_->set_group_pins({});
    {
        std::lock_guard lk(impl_->pulses_mu);
        impl_->pulses.clear();
    }
    if (!impl_->connected) { err = "Teensy not connected"; return false; }
    const bool ok = impl_->send_command("CFG_CLEAR", err);
    if (ok) {
        std::lock_guard lk(impl_->status_mu);
        impl_->armed = false;
    }
    return ok;
}

void TeensyManager::reset_pin_state(uint8_t pin) {
    std::lock_guard lk(impl_->pulses_mu);
    auto it = impl_->pulses.find(pin);
    if (it != impl_->pulses.end()) {
        it->second.last_camera_frame_id.reset();
        it->second.pulses.clear();
    }
}

uint64_t TeensyManager::pop_pulse_ns(uint8_t pin, uint64_t camera_frame_id) {
    std::lock_guard lk(impl_->pulses_mu);
    auto& state = impl_->pulses[pin];
    if (state.pulses.empty()) return 0;

    // Determine how many pulses to consume. On the first call after a reset
    // we simply take the oldest queued pulse. After that, each step in the
    // camera FrameID corresponds to one pulse — gaps (dropped frames) cause
    // us to skip extra pulses to keep the streams aligned.
    size_t to_consume = 1;
    if (state.last_camera_frame_id) {
        const uint64_t prev = *state.last_camera_frame_id;
        if (camera_frame_id <= prev) {
            // Out-of-order or duplicate FrameID — don't advance.
            return 0;
        }
        to_consume = static_cast<size_t>(camera_frame_id - prev);
    }
    if (state.pulses.size() < to_consume) {
        // Ring underflowed (we never saw enough pulses for the gap). Best
        // we can do is consume what we have and re-baseline.
        to_consume = state.pulses.size();
    }
    for (size_t i = 0; i + 1 < to_consume; ++i) state.pulses.pop_front();
    const auto [idx, t_us] = state.pulses.front();
    (void)idx;
    state.pulses.pop_front();
    state.last_camera_frame_id = camera_frame_id;
    return static_cast<uint64_t>(t_us) * 1000ull;
}

// ---------------------------------------------------------------------------
// Impl::run — single I/O thread (discover, read, dispatch)
// ---------------------------------------------------------------------------

void TeensyManager::Impl::run() {
    while (!stop_flag.load(std::memory_order_acquire)) {
        if (fd < 0) {
            if (!try_connect()) {
                std::this_thread::sleep_for(std::chrono::seconds(kReconnectDelaySec));
                continue;
            }
            // Push the most recent desired config on (re)connect. We do this
            // off the io_thread because send_command takes cmd_mu and blocks
            // waiting for our own reader to signal — which is us. Detach
            // briefly via a worker thread.
            std::thread([this] {
                resync_config();
            }).detach();
        }

        struct pollfd pfd{ fd, POLLIN, 0 };
        const int pr = ::poll(&pfd, 1, kReadTimeoutMs);
        if (pr < 0) {
            if (errno == EINTR) continue;
            note_error(std::string("poll: ") + std::strerror(errno));
            close_fd();
            continue;
        }
        if (pr == 0) continue;
        if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
            note_error("serial disconnected");
            close_fd();
            continue;
        }
        if (!(pfd.revents & POLLIN)) continue;

        char chunk[256];
        const ssize_t n = ::read(fd, chunk, sizeof(chunk));
        if (n <= 0) {
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) continue;
            note_error("serial read EOF");
            close_fd();
            continue;
        }
        for (ssize_t i = 0; i < n; ++i) {
            const char c = chunk[i];
            if (c == '\r') continue;
            if (c == '\n') {
                if (!line_buf.empty()) handle_incoming_line(line_buf);
                line_buf.clear();
                continue;
            }
            if (line_buf.size() < 1024) line_buf.push_back(c);
        }
    }
}

bool TeensyManager::Impl::try_connect() {
    glob_t gl{};
    if (::glob(kDeviceGlob, 0, nullptr, &gl) != 0) return false;
    std::optional<std::string> picked;
    for (size_t i = 0; i < gl.gl_pathc; ++i) {
        picked = gl.gl_pathv[i];
        break;  // first match wins; user can plug in just the Teensy if there
                // are multiple CDC devices around.
    }
    ::globfree(&gl);
    if (!picked) return false;

    const int candidate = ::open(picked->c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (candidate < 0) {
        note_error("open " + *picked + ": " + std::strerror(errno));
        return false;
    }
    // termios: raw 8N1, no flow control, no canonical processing.
    termios tio{};
    if (::tcgetattr(candidate, &tio) != 0) {
        ::close(candidate);
        return false;
    }
    cfmakeraw(&tio);
    tio.c_cflag |=  CLOCAL | CREAD;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;
    // USB-CDC ignores the configured baud; pick a standard rate so termios is
    // happy. The actual link is 12 Mbps full-speed or 480 Mbps high-speed USB.
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    ::tcsetattr(candidate, TCSANOW, &tio);

    fd        = candidate;
    open_port = *picked;
    line_buf.clear();
    {
        std::lock_guard lk(status_mu);
        connected  = true;
        last_error.reset();
    }
    std::cerr << "TeensyManager: connected " << *picked << "\n";
    return true;
}

void TeensyManager::Impl::close_fd() {
    if (fd >= 0) ::close(fd);
    fd = -1;
    open_port.reset();
    std::lock_guard lk(status_mu);
    connected = false;
    armed     = false;
}

bool TeensyManager::Impl::write_line_locked(std::string_view s, std::string& err) {
    if (fd < 0) { err = "not connected"; return false; }
    std::string line(s);
    line.push_back('\n');
    size_t off = 0;
    while (off < line.size()) {
        const ssize_t n = ::write(fd, line.data() + off, line.size() - off);
        if (n <= 0) {
            if (errno == EINTR) continue;
            err = std::string("write: ") + std::strerror(errno);
            return false;
        }
        off += static_cast<size_t>(n);
    }
    return true;
}

bool TeensyManager::Impl::send_command(std::string_view cmd, std::string& err) {
    std::unique_lock lk(cmd_mu);
    cmd_pending = true;
    cmd_result.reset();
    if (!write_line_locked(cmd, err)) {
        cmd_pending = false;
        return false;
    }
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(kCommandAckMs);
    while (cmd_pending) {
        if (cmd_cv.wait_until(lk, deadline) == std::cv_status::timeout) {
            cmd_pending = false;
            err = "Teensy command timeout";
            return false;
        }
    }
    if (cmd_result) { err = *cmd_result; return false; }
    return true;
}

void TeensyManager::Impl::handle_incoming_line(std::string_view line) {
    line = trim(line);
    if (line.empty()) return;

    // Trigger events are the hot path.
    if (line.substr(0, 5) == "TRIG ") {
        std::unordered_map<std::string, std::string> kv;
        parse_kv_tokens(line.substr(5), kv);
        const auto g_it = kv.find("g");
        const auto i_it = kv.find("idx");
        const auto t_it = kv.find("t_us");
        if (g_it == kv.end() || i_it == kv.end() || t_it == kv.end()) return;

        std::vector<uint8_t> pins;
        {
            std::lock_guard lk(group_pins_mu);
            const auto pit = group_pins.find(g_it->second);
            if (pit == group_pins.end()) return;  // unknown group — drop
            pins = pit->second;
        }
        const uint32_t idx  = static_cast<uint32_t>(std::strtoul(i_it->second.c_str(), nullptr, 10));
        const uint32_t t_us = static_cast<uint32_t>(std::strtoul(t_it->second.c_str(), nullptr, 10));
        {
            std::lock_guard lk(pulses_mu);
            for (auto p : pins) {
                auto& state = pulses[p];
                state.pulses.emplace_back(idx, t_us);
                while (state.pulses.size() > kPulseRingMax) state.pulses.pop_front();
            }
        }
        {
            std::lock_guard lk(status_mu);
            last_pulse_at   = std::chrono::steady_clock::now();
            last_pulse_seen = true;
            ++total_pulses;
        }
        return;
    }

    if (line == "OK" || line.substr(0, 4) == "ERR ") {
        std::lock_guard lk(cmd_mu);
        if (!cmd_pending) return;  // stray ack (race) — ignore
        cmd_pending = false;
        if (line == "OK") cmd_result.reset();
        else              cmd_result = std::string(line.substr(4));
        cmd_cv.notify_all();
        return;
    }

    // READY / PONG / STATUS / GROUP lines are informational right now.
    if (line.substr(0, 6) == "READY ") {
        std::cerr << "TeensyManager: " << line << "\n";
        return;
    }
}

bool TeensyManager::Impl::send_full_config(
        const std::vector<TeensyManager::GroupConfig>& groups,
        bool want_armed_target, std::string& err) {
    if (!send_command("CFG_CLEAR", err)) return false;
    for (const auto& g : groups) {
        std::ostringstream cmd;
        cmd << "CFG name=" << g.name << " fps=" << g.fps << " pins=";
        for (size_t i = 0; i < g.output_pins.size(); ++i) {
            if (i) cmd << ',';
            cmd << static_cast<int>(g.output_pins[i]);
        }
        if (!send_command(cmd.str(), err)) return false;
    }
    if (want_armed_target && !groups.empty()) {
        if (!send_command("ARM", err)) return false;
        std::lock_guard lk(status_mu);
        armed = true;
    }
    return true;
}

void TeensyManager::Impl::resync_config() {
    std::vector<TeensyManager::GroupConfig> snapshot;
    bool want_armed_snap = false;
    {
        std::lock_guard lk(cfg_mu);
        snapshot       = desired_cfg;
        want_armed_snap = want_armed;
    }
    if (snapshot.empty()) return;
    std::string err;
    if (!send_full_config(snapshot, want_armed_snap, err)) {
        note_error("resync: " + err);
    }
}

void TeensyManager::Impl::note_error(std::string msg) {
    std::cerr << "TeensyManager: " << msg << "\n";
    std::lock_guard lk(status_mu);
    last_error = std::move(msg);
}

void TeensyManager::Impl::set_group_pins(const std::vector<TeensyManager::GroupConfig>& groups) {
    std::lock_guard lk(group_pins_mu);
    group_pins.clear();
    for (const auto& g : groups) group_pins[g.name] = g.output_pins;
}

}  // namespace gw::server
