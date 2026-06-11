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

#include "core/rio_clock_sync.hpp"
#include "firmware/src/can_payloads.h"
#include "server/telemetry_decoder.hpp"

namespace gw::server {

namespace {

constexpr int  kReconnectDelaySec = 3;
constexpr int  kReadTimeoutMs     = 200;
constexpr int  kCommandAckMs      = 1000;
constexpr int  kPingProbeMs       = 500;
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

const char* can_mode_arg(CanMode m) {
    switch (m) {
        case CanMode::Classic: return "classic";
        case CanMode::Fd:      return "fd";
        case CanMode::Off:     break;
    }
    return "off";
}

}  // namespace

struct PinState {
    std::deque<std::pair<uint32_t, uint64_t>> pulses;  // {teensy_idx, t_us}
    std::optional<uint64_t> last_camera_frame_id;
};

struct TeensyManager::Impl {
    std::thread                io_thread;
    std::atomic<bool>          stop_flag{false};

    // I/O state — touched only by io_thread except where noted.
    int                        fd = -1;
    std::string                line_buf;
    std::optional<std::string> open_port;

    // Binary telemetry interface (second USB-CDC, fw≥2). Absent on fw=1.
    // Read by io_thread; written (POSE downlink) by HTTP threads under
    // pose_tx_mu — close_telemetry_fd() also takes pose_tx_mu so the fd
    // can't be recycled under a writer.
    int                                    telemetry_fd = -1;
    std::optional<std::string>             telemetry_port;
    std::chrono::steady_clock::time_point  last_telemetry_attempt{};
    TelemetryDecoder                       decoder;
    gw::MeasurementBus<gw::ImuSample>      imu_bus;
    gw::OdomBus                            odom_bus;

    // RIO ↔ Teensy clock sync. Fed on io_thread (on_odom), read by status()
    // and send_pose() on HTTP threads.
    mutable std::mutex sync_mu;
    RioClockSync       rio_sync;
    uint64_t           sync_last_arrival_us = 0;  // "now" proxy for healthy()

    // POSE downlink serialization (counter + fd writes).
    std::mutex            pose_tx_mu;
    uint8_t               pose_counter = 0;
    std::atomic<uint64_t> pose_sent{0};
    std::atomic<uint64_t> pose_send_errors{0};

    // IMU rate window + heartbeat-derived state. Guarded by status_mu.
    std::chrono::steady_clock::time_point imu_window_start{};
    uint64_t                              imu_window_count = 0;
    double                                imu_rate_hz      = 0.0;
    std::chrono::steady_clock::time_point imu_last_sample_at{};
    bool                                  imu_sample_seen = false;
    bool                                  imu_ok          = false;
    uint64_t                              imu_fw_drops    = 0;
    uint64_t                              imu_samples_total = 0;  // mirror of decoder stats
    uint64_t                              imu_crc_errors    = 0;  // (decoder runs unlocked
    std::optional<int>                    fw_version;             //  on io_thread)

    // CAN / odometry state. Guarded by status_mu.
    bool     can_ok           = false;
    int      can_mode_fw      = -1;
    uint64_t can_rx           = 0;
    uint64_t can_rx_drops     = 0;
    uint64_t odom_tx_fw_drops = 0;
    uint64_t pose_tx_fw       = 0;
    std::chrono::steady_clock::time_point odom_window_start{};
    uint64_t                              odom_window_count = 0;
    double                                odom_rate_hz      = 0.0;
    std::chrono::steady_clock::time_point odom_last_at{};
    bool                                  odom_seen = false;
    std::optional<gw::ChassisSpeeds>      odom_last;
    uint64_t                              odom_packets_total = 0;  // decoder mirror

    // Configuration cache (push_config payload + CAN mode). Re-sent after
    // every (re)connect. Guarded by cfg_mu.
    std::mutex                                  cfg_mu;
    std::vector<TeensyManager::GroupConfig>     desired_cfg;
    bool                                        want_armed = false;
    CanMode                                     desired_can_mode = CanMode::Off;

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
    void try_connect_telemetry();
    void close_fd();
    void close_telemetry_fd();
    bool write_line_locked(std::string_view s, std::string& err);
    bool send_command(std::string_view cmd, std::string& err);  // takes cmd_mu
    void handle_incoming_line(std::string_view line);
    void note_fw_version(std::string_view line);  // parses "fw=<n>" tokens
    bool send_full_config(const std::vector<TeensyManager::GroupConfig>& groups,
                          bool want_armed, std::string& err);
    bool send_can_mode(CanMode mode, std::string& err);  // CAN_MODE command, fw≥3 only
    void resync_config();           // pushes the cfg_mu-stashed snapshot
    void note_error(std::string msg);
    void setup_decoder_callbacks();

    void set_group_pins(const std::vector<TeensyManager::GroupConfig>& groups);
};

TeensyManager::TeensyManager()  : impl_(std::make_unique<Impl>()) {
    impl_->setup_decoder_callbacks();
}
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
    impl_->close_telemetry_fd();
    impl_->close_fd();
}

gw::MeasurementBus<gw::ImuSample>& TeensyManager::imu_bus() {
    return impl_->imu_bus;
}

gw::OdomBus& TeensyManager::odom_bus() {
    return impl_->odom_bus;
}

bool TeensyManager::set_can_mode(CanMode mode, std::string& err) {
    {
        std::lock_guard lk(impl_->cfg_mu);
        impl_->desired_can_mode = mode;
    }
    if (!impl_->connected) {
        err = "Teensy not connected";
        return false;
    }
    return impl_->send_can_mode(mode, err);
}

bool TeensyManager::send_pose(const gw::FusedPose& pose, std::string& err) {
    gw_fw::canp::PoseWire wire;
    wire.x       = pose.x_m;
    wire.y       = pose.y_m;
    wire.theta   = pose.theta_rad;
    wire.quality = pose.quality;
    {
        std::lock_guard lk(impl_->sync_mu);
        // 0 = unmapped is part of the wire contract; the controller's
        // staleness detection is counter-based, so still send.
        wire.rio_time_us = impl_->rio_sync.to_rio_us(pose.t_ns).value_or(0);
    }

    std::lock_guard lk(impl_->pose_tx_mu);
    if (impl_->telemetry_fd < 0) {
        err = "telemetry interface not connected";
        impl_->pose_send_errors.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    wire.counter = ++impl_->pose_counter;

    uint8_t payload[gw_fw::canp::kPoseTelemetryPayloadLen];
    gw_fw::canp::encode_pose_telemetry(wire, payload);
    uint8_t frame[6 + sizeof(payload)];
    const size_t frame_len = gw_fw::canp::build_telemetry_frame(
        gw_fw::canp::kBinTypePose, payload, sizeof(payload), frame);

    size_t off = 0;
    while (off < frame_len) {
        const ssize_t n =
            ::write(impl_->telemetry_fd, frame + off, frame_len - off);
        if (n <= 0) {
            if (n < 0 && errno == EINTR) continue;
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                // Non-blocking fd with a full USB buffer — drop this pose
                // (the next one supersedes it anyway).
                err = "telemetry write would block";
            } else {
                err = std::string("telemetry write: ") + std::strerror(errno);
            }
            impl_->pose_send_errors.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        off += static_cast<size_t>(n);
    }
    impl_->pose_sent.fetch_add(1, std::memory_order_relaxed);
    return true;
}

TeensyManager::Status TeensyManager::status() const {
    Status s;
    std::lock_guard lk(impl_->status_mu);
    s.connected   = impl_->connected;
    s.port        = impl_->open_port;
    s.armed       = impl_->armed;
    s.total_pulses = impl_->total_pulses;
    s.last_error  = impl_->last_error;
    s.fw_version  = impl_->fw_version;
    if (impl_->last_pulse_seen) {
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - impl_->last_pulse_at).count();
        s.last_pulse_age_ms = ms;
    }

    s.telemetry_connected = impl_->telemetry_port.has_value();
    s.imu_ok              = impl_->imu_ok;
    s.imu_rate_hz         = impl_->imu_rate_hz;
    s.imu_samples         = impl_->imu_samples_total;
    s.imu_fw_drops        = impl_->imu_fw_drops;
    s.imu_crc_errors      = impl_->imu_crc_errors;
    if (impl_->imu_sample_seen) {
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - impl_->imu_last_sample_at).count();
        s.imu_last_sample_age_ms = ms;
    }

    s.can_ok           = impl_->can_ok;
    s.can_mode_fw      = impl_->can_mode_fw;
    s.can_rx           = impl_->can_rx;
    s.can_rx_drops     = impl_->can_rx_drops;
    s.odom_tx_fw_drops = impl_->odom_tx_fw_drops;
    s.pose_tx_fw       = impl_->pose_tx_fw;
    s.odom_rate_hz     = impl_->odom_rate_hz;
    s.odom_packets     = impl_->odom_packets_total;
    s.odom_last        = impl_->odom_last;
    if (impl_->odom_seen) {
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - impl_->odom_last_at).count();
        s.odom_last_age_ms = ms;
    }
    s.pose_sent        = impl_->pose_sent.load(std::memory_order_relaxed);
    s.pose_send_errors = impl_->pose_send_errors.load(std::memory_order_relaxed);
    {
        std::lock_guard sync_lk(impl_->sync_mu);
        s.sync_healthy   = impl_->rio_sync.healthy(impl_->sync_last_arrival_us);
        s.sync_offset_us = impl_->rio_sync.offset_us();
        s.sync_drift_ppm = impl_->rio_sync.drift_ppm();
        s.sync_samples   = impl_->rio_sync.samples();
        s.sync_resets    = impl_->rio_sync.resets();
    }
    {
        std::lock_guard cfg_lk(impl_->cfg_mu);
        s.can_mode_desired = impl_->desired_can_mode;
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
    return t_us * 1000ull;
}

// ---------------------------------------------------------------------------
// Impl::run — single I/O thread (discover, read, dispatch)
// ---------------------------------------------------------------------------

void TeensyManager::Impl::run() {
    while (!stop_flag.load(std::memory_order_acquire)) {
        if (fd < 0) {
            close_telemetry_fd();
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
        if (telemetry_fd < 0) try_connect_telemetry();

        struct pollfd pfds[2];
        pfds[0] = { fd, POLLIN, 0 };
        nfds_t nfds = 1;
        if (telemetry_fd >= 0) {
            pfds[1] = { telemetry_fd, POLLIN, 0 };
            nfds = 2;
        }
        const int pr = ::poll(pfds, nfds, kReadTimeoutMs);
        if (pr < 0) {
            if (errno == EINTR) continue;
            note_error(std::string("poll: ") + std::strerror(errno));
            close_fd();
            continue;
        }
        if (pr == 0) continue;

        // Command/TRIG interface (ASCII lines).
        if (pfds[0].revents & (POLLHUP | POLLERR | POLLNVAL)) {
            note_error("serial disconnected");
            close_fd();
            continue;
        }
        if (pfds[0].revents & POLLIN) {
            char chunk[256];
            const ssize_t n = ::read(fd, chunk, sizeof(chunk));
            if (n <= 0) {
                if (!(n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
                    note_error("serial read EOF");
                    close_fd();
                    continue;
                }
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

        // Telemetry interface (binary frames). Telemetry loss is not fatal
        // to triggering — close just this fd and retry.
        if (nfds == 2) {
            if (pfds[1].revents & (POLLHUP | POLLERR | POLLNVAL)) {
                close_telemetry_fd();
            } else if (pfds[1].revents & POLLIN) {
                uint8_t chunk[1024];
                const ssize_t n = ::read(telemetry_fd, chunk, sizeof(chunk));
                if (n <= 0) {
                    if (!(n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
                        close_telemetry_fd();
                    }
                } else {
                    decoder.feed(chunk, static_cast<size_t>(n));
                    std::lock_guard lk(status_mu);
                    imu_samples_total  = decoder.stats().imu_samples;
                    imu_crc_errors     = decoder.stats().crc_errors;
                    odom_packets_total = decoder.stats().odom_packets;
                }
            }
        }
    }
}

namespace {

// Open a serial device raw 8N1, no flow control, non-blocking. Returns -1
// on failure. USB-CDC ignores the configured baud; pick a standard rate so
// termios is happy — the actual link is 12/480 Mbps USB.
int open_serial(const std::string& path) {
    const int candidate = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (candidate < 0) return -1;
    termios tio{};
    if (::tcgetattr(candidate, &tio) != 0) {
        ::close(candidate);
        return -1;
    }
    cfmakeraw(&tio);
    tio.c_cflag |=  CLOCAL | CREAD;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    ::tcsetattr(candidate, TCSANOW, &tio);
    return candidate;
}

std::vector<std::string> glob_devices() {
    std::vector<std::string> out;
    glob_t gl{};
    if (::glob(kDeviceGlob, 0, nullptr, &gl) == 0) {
        for (size_t i = 0; i < gl.gl_pathc; ++i) out.emplace_back(gl.gl_pathv[i]);
    }
    ::globfree(&gl);
    return out;
}

// Writes PING and waits up to kPingProbeMs for a PONG line. Only the ASCII
// command interface answers — the binary telemetry interface never parses
// input, so this distinguishes the two CDC interfaces of one Teensy (and
// rejects unrelated usbmodem devices). Returns the matched PONG line via
// `pong_line` for fw-version extraction.
bool probe_command_port(int fd, std::string& pong_line) {
    const char ping[] = "PING\n";
    if (::write(fd, ping, sizeof(ping) - 1) != static_cast<ssize_t>(sizeof(ping) - 1)) {
        return false;
    }
    std::string acc;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(kPingProbeMs);
    while (std::chrono::steady_clock::now() < deadline) {
        struct pollfd pfd{ fd, POLLIN, 0 };
        const int pr = ::poll(&pfd, 1, 50);
        if (pr < 0 && errno != EINTR) return false;
        if (pr <= 0 || !(pfd.revents & POLLIN)) continue;
        char chunk[256];
        const ssize_t n = ::read(fd, chunk, sizeof(chunk));
        if (n <= 0) continue;
        acc.append(chunk, static_cast<size_t>(n));
        size_t pos = 0, nl;
        while ((nl = acc.find('\n', pos)) != std::string::npos) {
            std::string_view line = trim(std::string_view(acc).substr(pos, nl - pos));
            if (line.substr(0, 4) == "PONG") {
                pong_line = std::string(line);
                return true;
            }
            pos = nl + 1;
        }
        acc.erase(0, pos);
        if (acc.size() > 4096) acc.clear();  // binary noise — keep bounded
    }
    return false;
}

}  // namespace

bool TeensyManager::Impl::try_connect() {
    for (const auto& path : glob_devices()) {
        const int candidate = open_serial(path);
        if (candidate < 0) continue;
        std::string pong;
        if (!probe_command_port(candidate, pong)) {
            ::close(candidate);
            continue;
        }
        fd        = candidate;
        open_port = path;
        line_buf.clear();
        {
            std::lock_guard lk(status_mu);
            connected = true;
            last_error.reset();
        }
        note_fw_version(pong);
        std::cerr << "TeensyManager: connected " << path << " (" << pong << ")\n";
        return true;
    }
    return false;
}

void TeensyManager::Impl::try_connect_telemetry() {
    if (fd < 0 || !open_port) return;
    const auto now = std::chrono::steady_clock::now();
    if (now - last_telemetry_attempt < std::chrono::seconds(kReconnectDelaySec)) return;
    last_telemetry_attempt = now;

    // The two CDC interfaces of one Teensy enumerate as sibling device nodes
    // differing only in the trailing interface digit (e.g. …01 / …03).
    for (const auto& path : glob_devices()) {
        if (path == *open_port) continue;
        if (path.size() != open_port->size()) continue;
        if (path.compare(0, path.size() - 1, *open_port, 0,
                         open_port->size() - 1) != 0) continue;
        const int candidate = open_serial(path);
        if (candidate < 0) continue;
        telemetry_fd = candidate;
        {
            std::lock_guard lk(status_mu);
            telemetry_port = path;
        }
        std::cerr << "TeensyManager: telemetry connected " << path << "\n";
        return;
    }
}

void TeensyManager::Impl::close_fd() {
    if (fd >= 0) ::close(fd);
    fd = -1;
    open_port.reset();
    std::lock_guard lk(status_mu);
    connected = false;
    armed     = false;
}

void TeensyManager::Impl::close_telemetry_fd() {
    {
        // send_pose writes this fd from HTTP threads; don't recycle it under
        // a writer.
        std::lock_guard pose_lk(pose_tx_mu);
        if (telemetry_fd >= 0) ::close(telemetry_fd);
        telemetry_fd = -1;
    }
    std::lock_guard lk(status_mu);
    telemetry_port.reset();
    imu_ok          = false;
    imu_rate_hz     = 0.0;
    imu_window_count = 0;
    imu_sample_seen = false;
    can_ok           = false;
    can_mode_fw      = -1;
    odom_rate_hz     = 0.0;
    odom_window_count = 0;
    odom_seen        = false;
}

void TeensyManager::Impl::setup_decoder_callbacks() {
    decoder.on_imu = [this](const gw::ImuSample& s) {
        imu_bus.publish(s);
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard lk(status_mu);
        imu_last_sample_at = now;
        imu_sample_seen    = true;
        if (imu_window_count == 0) imu_window_start = now;
        ++imu_window_count;
        const auto elapsed = now - imu_window_start;
        if (elapsed >= std::chrono::seconds(1)) {
            imu_rate_hz = static_cast<double>(imu_window_count) /
                          std::chrono::duration<double>(elapsed).count();
            imu_window_count = 0;
        }
    };
    decoder.on_heartbeat = [this](const TelemetryDecoder::Heartbeat& hb) {
        std::lock_guard lk(status_mu);
        imu_ok       = hb.imu_ok;
        imu_fw_drops = hb.imu_drops;
        if (hb.can_present) {
            can_ok           = hb.can_ok;
            can_mode_fw      = hb.can_mode;
            can_rx           = hb.can_rx;
            can_rx_drops     = hb.can_rx_drops;
            odom_tx_fw_drops = hb.odom_tx_drops;
            pose_tx_fw       = hb.pose_tx;
        }
    };
    decoder.on_odom = [this](const TelemetryDecoder::Odom& o) {
        gw::ChassisSpeeds s;
        s.t_arrival_ns = o.t_arrival_us * 1000ull;
        s.rio_time_us  = o.rio_time_us;
        s.vx_mps       = o.vx;
        s.vy_mps       = o.vy;
        s.omega_radps  = o.omega;
        s.status_flags = o.status_flags;
        s.counter      = o.counter;
        {
            // Scoped: status() nests sync_mu inside status_mu, so never hold
            // sync_mu while acquiring status_mu below.
            std::lock_guard sync_lk(sync_mu);
            if (o.rio_time_us != 0) {
                rio_sync.feed(o.rio_time_us, o.t_arrival_us);
            }
            sync_last_arrival_us = o.t_arrival_us;
            // Best-estimate sample time: mapped RIO sample time when the
            // sync fit is usable, CAN arrival stamp otherwise.
            s.t_ns = s.t_arrival_ns;
            if (o.rio_time_us != 0 && rio_sync.healthy(o.t_arrival_us)) {
                if (const auto mapped = rio_sync.to_teensy_ns(o.rio_time_us)) {
                    s.t_ns = *mapped;
                }
            }
        }
        odom_bus.publish(s);

        const auto now = std::chrono::steady_clock::now();
        std::lock_guard lk(status_mu);
        odom_last_at = now;
        odom_seen    = true;
        odom_last    = s;
        if (odom_window_count == 0) odom_window_start = now;
        ++odom_window_count;
        const auto elapsed = now - odom_window_start;
        if (elapsed >= std::chrono::seconds(1)) {
            odom_rate_hz = static_cast<double>(odom_window_count) /
                           std::chrono::duration<double>(elapsed).count();
            odom_window_count = 0;
        }
    };
}

void TeensyManager::Impl::note_fw_version(std::string_view line) {
    const size_t pos = line.find("fw=");
    if (pos == std::string_view::npos) return;
    const int v = std::atoi(std::string(line.substr(pos + 3)).c_str());
    if (v <= 0) return;
    std::lock_guard lk(status_mu);
    fw_version = v;
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
        // fw=2 sends a wrap-extended 64-bit microsecond timestamp; fw=1 sent
        // raw 32-bit micros(). Parsing as u64 accepts both wire formats.
        const uint64_t t_us = std::strtoull(t_it->second.c_str(), nullptr, 10);
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
    if (line.substr(0, 6) == "READY " || line.substr(0, 5) == "PONG ") {
        note_fw_version(line);
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

bool TeensyManager::Impl::send_can_mode(CanMode mode, std::string& err) {
    {
        std::lock_guard lk(status_mu);
        if (!fw_version || *fw_version < 3) {
            err = "firmware too old for CAN (need fw>=3)";
            return false;
        }
    }
    std::string cmd = "CAN_MODE mode=";
    cmd += can_mode_arg(mode);
    return send_command(cmd, err);
}

void TeensyManager::Impl::resync_config() {
    std::vector<TeensyManager::GroupConfig> snapshot;
    bool    want_armed_snap = false;
    CanMode can_mode_snap   = CanMode::Off;
    {
        std::lock_guard lk(cfg_mu);
        snapshot        = desired_cfg;
        want_armed_snap = want_armed;
        can_mode_snap   = desired_can_mode;
    }
    // CAN mode first — independent of trigger config, and cheap. The Teensy
    // boots with CAN off, so Off needs no push.
    if (can_mode_snap != CanMode::Off) {
        std::string err;
        if (!send_can_mode(can_mode_snap, err)) {
            note_error("resync CAN_MODE: " + err);
        }
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
