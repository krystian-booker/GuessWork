#include "server/sync_controller_manager.hpp"

#include <fcntl.h>
#include <glob.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <iostream>
#include <mutex>
#include <thread>

#include "core/clock.hpp"
#include "core/clock_sync.hpp"
#include "firmware/include/sync_controller_protocol.h"
#include "server/sync_protocol_decoder.hpp"

namespace gw::server {

namespace {

constexpr int kReconnectDelayMs = 1000;
constexpr int kReadTimeoutMs    = 200;
constexpr int kCommandAckMs     = 1000;
constexpr int kProbeMs          = 600;
constexpr int kPulseRingMax     = 256;

int open_serial(const std::string& path) {
    const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;
    termios tio{};
    if (::tcgetattr(fd, &tio) != 0) {
        ::close(fd);
        return -1;
    }
    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    (void)::tcsetattr(fd, TCSANOW, &tio);
    return fd;
}

std::vector<std::string> glob_devices(const std::string& pattern) {
    std::vector<std::string> out;
    glob_t gl{};
    if (::glob(pattern.c_str(), 0, nullptr, &gl) == 0) {
        for (size_t i = 0; i < gl.gl_pathc; ++i) out.emplace_back(gl.gl_pathv[i]);
    }
    ::globfree(&gl);
    return out;
}

bool write_all(int fd, const uint8_t* data, size_t len, std::string& err) {
    size_t off = 0;
    while (off < len) {
        const ssize_t n = ::write(fd, data + off, len - off);
        if (n > 0) {
            off += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            pollfd pfd{fd, POLLOUT, 0};
            if (::poll(&pfd, 1, 100) > 0) continue;
        }
        err = std::string("USB write: ") + std::strerror(errno);
        return false;
    }
    return true;
}

bool probe_controller(int fd, gw_sync::DeviceInfo& info) {
    uint8_t request[gw_sync::kMaxFrameBytes];
    constexpr uint16_t kProbeId = 0x4757;
    const size_t request_len = gw_sync::build_frame(
        gw_sync::MessageType::Hello, kProbeId, nullptr, 0, request);
    std::string err;
    if (!write_all(fd, request, request_len, err)) return false;

    bool matched = false;
    SyncProtocolDecoder decoder;
    decoder.on_device_info = [&](uint16_t request_id,
                                 const gw_sync::DeviceInfo& candidate) {
        constexpr uint32_t kRequiredCapabilities =
            gw_sync::kCapabilityTriggers |
            gw_sync::kCapabilityImu |
            gw_sync::kCapabilityAtomicConfig |
            gw_sync::kCapabilityTestOutput |
            gw_sync::kCapabilityBoardFrameImu;
        if (request_id == kProbeId &&
            candidate.board_id == gw_sync::kBoardIdMicoAirF405V2 &&
            candidate.output_count >= gw_sync::kOutputCount &&
            candidate.max_groups >= gw_sync::kMaxGroups &&
            (candidate.capabilities & kRequiredCapabilities) ==
                kRequiredCapabilities) {
            info = candidate;
            matched = true;
        }
    };
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(kProbeMs);
    while (!matched && std::chrono::steady_clock::now() < deadline) {
        pollfd pfd{fd, POLLIN, 0};
        const int pr = ::poll(&pfd, 1, 50);
        if (pr < 0 && errno != EINTR) return false;
        if (pr <= 0 || !(pfd.revents & POLLIN)) continue;
        uint8_t bytes[512];
        const ssize_t n = ::read(fd, bytes, sizeof(bytes));
        if (n > 0) decoder.feed(bytes, static_cast<size_t>(n));
    }
    return matched;
}

const char* ack_status_string(gw_sync::AckStatus status) {
    switch (status) {
        case gw_sync::AckStatus::Ok:            return "ok";
        case gw_sync::AckStatus::BadMessage:    return "bad message";
        case gw_sync::AckStatus::BadConfig:     return "bad config";
        case gw_sync::AckStatus::Busy:          return "controller busy";
        case gw_sync::AckStatus::NoConfig:      return "no trigger groups configured";
        case gw_sync::AckStatus::Unsupported:   return "unsupported command";
        case gw_sync::AckStatus::InternalError: return "controller internal error";
    }
    return "unknown controller error";
}

struct PinState {
    std::deque<std::pair<uint32_t, uint64_t>> pulses;
    std::optional<uint64_t> last_camera_frame_id;
};

}  // namespace

struct SyncControllerManager::Impl {
    std::string device_glob;
    std::thread io_thread;
    std::thread resync_thread;
    std::atomic<bool> stop_flag{false};

    int fd = -1;
    std::optional<std::string> open_port;
    SyncProtocolDecoder decoder;

    mutable std::mutex sync_mu;
    gw::ClockSync controller_sync;
    uint64_t last_imu_feed_host_us = 0;

    mutable std::mutex status_mu;
    std::atomic<bool> connected{false};
    bool armed = false;
    std::optional<std::string> board;
    std::optional<int> firmware_version;
    uint32_t reset_reason = 0;
    std::optional<std::string> last_error;
    std::chrono::steady_clock::time_point last_pulse_at{};
    bool last_pulse_seen = false;
    uint64_t total_pulses = 0;
    bool imu_ok = false;
    uint64_t imu_fw_drops = 0;
    uint64_t trigger_fw_drops = 0;
    uint64_t usb_errors = 0;
    uint64_t imu_samples_total = 0;
    uint64_t imu_crc_errors = 0;
    std::chrono::steady_clock::time_point imu_window_start{};
    uint64_t imu_window_count = 0;
    double imu_rate_hz = 0.0;
    std::chrono::steady_clock::time_point imu_last_sample_at{};
    bool imu_sample_seen = false;

    gw::MeasurementBus<gw::ImuSample> imu_bus;

    std::mutex cfg_mu;
    std::vector<SyncControllerManager::GroupConfig> desired_cfg;
    bool want_armed = false;
    std::atomic<bool> resync_needed{false};
    std::mutex resync_mu;
    std::condition_variable resync_cv;

    std::mutex slot_pins_mu;
    std::array<std::vector<uint8_t>, gw_sync::kMaxGroups> slot_pins;

    mutable std::mutex pulses_mu;
    std::array<PinState, gw_sync::kOutputCount + 1> pins;

    std::mutex cmd_mu;
    std::condition_variable cmd_cv;
    uint16_t next_request_id = 1;
    bool cmd_pending = false;
    uint16_t pending_request_id = 0;
    gw_sync::MessageType pending_command = gw_sync::MessageType::Hello;
    std::optional<gw_sync::AckStatus> cmd_status;

    void run();
    void run_resync();
    bool try_connect();
    void close_fd();
    void setup_decoder_callbacks();
    void request_resync();
    bool send_command(gw_sync::MessageType type, const uint8_t* payload,
                      uint16_t payload_len, std::string& err);
    bool send_command_once(gw_sync::MessageType type, const uint8_t* payload,
                           uint16_t payload_len, std::string& err);
    bool send_full_config(const std::vector<GroupConfig>& groups,
                          bool arm_target, std::string& err);
    void set_slot_pins(const std::vector<GroupConfig>& groups);
    void feed_controller_sync(uint64_t controller_us, bool from_imu);
    void note_error(std::string error);
};

SyncControllerManager::SyncControllerManager(std::string device_glob)
    : impl_(std::make_unique<Impl>()) {
    impl_->device_glob = std::move(device_glob);
    impl_->setup_decoder_callbacks();
}

SyncControllerManager::~SyncControllerManager() { stop(); }

void SyncControllerManager::start() {
    if (impl_->io_thread.joinable()) return;
    impl_->stop_flag.store(false);
    impl_->io_thread = std::thread([this] { impl_->run(); });
    impl_->resync_thread = std::thread([this] { impl_->run_resync(); });
}

void SyncControllerManager::stop() {
    if (!impl_->io_thread.joinable()) return;
    impl_->stop_flag.store(true);
    impl_->resync_cv.notify_all();
    impl_->cmd_cv.notify_all();
    impl_->io_thread.join();
    impl_->resync_thread.join();
    impl_->close_fd();
}

gw::MeasurementBus<gw::ImuSample>& SyncControllerManager::imu_bus() {
    return impl_->imu_bus;
}

std::optional<uint64_t> SyncControllerManager::host_to_controller_ns(
        uint64_t host_ns) const {
    std::lock_guard lk(impl_->sync_mu);
    const auto us = impl_->controller_sync.to_remote_us(host_ns);
    if (!us) return std::nullopt;
    return *us * 1000ull;
}

std::optional<uint64_t> SyncControllerManager::controller_to_host_ns(
        uint64_t controller_ns) const {
    std::lock_guard lk(impl_->sync_mu);
    return impl_->controller_sync.to_local_ns(controller_ns / 1000ull);
}

SyncControllerManager::Status SyncControllerManager::status() const {
    Status out;
    std::lock_guard lk(impl_->status_mu);
    out.connected           = impl_->connected.load();
    out.port                = impl_->open_port;
    out.board               = impl_->board;
    out.armed               = impl_->armed;
    out.total_pulses        = impl_->total_pulses;
    out.last_error          = impl_->last_error;
    out.firmware_version    = impl_->firmware_version;
    out.protocol_version    = impl_->connected.load()
        ? std::optional<int>(gw_sync::kProtocolVersion) : std::nullopt;
    out.reset_reason        = impl_->reset_reason;
    out.imu_ok              = impl_->imu_ok;
    out.imu_rate_hz         = impl_->imu_rate_hz;
    out.imu_samples         = impl_->imu_samples_total;
    out.imu_fw_drops        = impl_->imu_fw_drops;
    out.imu_crc_errors      = impl_->imu_crc_errors;
    out.trigger_fw_drops    = impl_->trigger_fw_drops;
    out.usb_errors          = impl_->usb_errors;
    if (impl_->last_pulse_seen) {
        out.last_pulse_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - impl_->last_pulse_at).count();
    }
    if (impl_->imu_sample_seen) {
        out.imu_last_sample_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - impl_->imu_last_sample_at).count();
    }
    {
        std::lock_guard sync_lk(impl_->sync_mu);
        out.sync_healthy = impl_->controller_sync.healthy(gw::Clock::now_ns() / 1000);
        out.sync_offset_us = impl_->controller_sync.offset_us();
        out.sync_drift_ppm = impl_->controller_sync.drift_ppm();
        out.sync_samples = impl_->controller_sync.samples();
        out.sync_resets = impl_->controller_sync.resets();
    }
    return out;
}

bool SyncControllerManager::push_config(const std::vector<GroupConfig>& groups,
                                        std::string& err) {
    if (groups.size() > gw_sync::kMaxGroups) {
        err = "controller supports at most four trigger groups";
        return false;
    }
    uint8_t assigned_outputs = 0;
    for (const auto& group : groups) {
        if (!(group.fps >= 0.001) || group.fps > 1000.0) {
            err = "trigger rate must be in [0.001, 1000] Hz";
            return false;
        }
        if (group.output_pins.empty()) {
            err = "each trigger group needs at least one output";
            return false;
        }
        for (const uint8_t output : group.output_pins) {
            if (output < 1 || output > gw_sync::kOutputCount) {
                err = "trigger output must be 1..6";
                return false;
            }
            const uint8_t bit = static_cast<uint8_t>(1u << (output - 1));
            if (assigned_outputs & bit) {
                err = "trigger output assigned more than once";
                return false;
            }
            assigned_outputs = static_cast<uint8_t>(assigned_outputs | bit);
        }
    }
    {
        std::lock_guard lk(impl_->cfg_mu);
        impl_->desired_cfg = groups;
        impl_->want_armed = !groups.empty();
    }
    impl_->set_slot_pins(groups);
    {
        std::lock_guard lk(impl_->pulses_mu);
        for (auto& pin : impl_->pins) pin = {};
    }
    if (!impl_->connected.load()) {
        err = "sync controller not connected";
        impl_->request_resync();
        return false;
    }
    const bool ok = impl_->send_full_config(groups, !groups.empty(), err);
    if (!ok) impl_->request_resync();
    return ok;
}

bool SyncControllerManager::stop_outputs(std::string& err) {
    {
        std::lock_guard lk(impl_->cfg_mu);
        impl_->want_armed = false;
    }
    if (!impl_->connected.load()) {
        err = "sync controller not connected";
        return false;
    }
    const bool ok = impl_->send_command(gw_sync::MessageType::Stop,
                                         nullptr, 0, err);
    if (ok) {
        std::lock_guard lk(impl_->status_mu);
        impl_->armed = false;
    }
    return ok;
}

bool SyncControllerManager::clear_config(std::string& err) {
    {
        std::lock_guard lk(impl_->cfg_mu);
        impl_->desired_cfg.clear();
        impl_->want_armed = false;
    }
    impl_->set_slot_pins({});
    uint8_t payload[4] = {};
    if (!impl_->connected.load()) {
        err = "sync controller not connected";
        return false;
    }
    const bool ok = impl_->send_command(gw_sync::MessageType::SetConfig,
                                         payload, sizeof(payload), err);
    if (ok) {
        std::lock_guard lk(impl_->status_mu);
        impl_->armed = false;
    }
    return ok;
}

bool SyncControllerManager::test_output(uint8_t output_pin, std::string& err) {
    if (output_pin < 1 || output_pin > gw_sync::kOutputCount) {
        err = "trigger output must be 1..6";
        return false;
    }
    if (!impl_->connected.load()) {
        err = "sync controller not connected";
        return false;
    }
    // TEST_OUTPUT is deliberately not retried: if its ACK alone is lost, a
    // retry would generate a second physical camera edge.
    return impl_->send_command_once(gw_sync::MessageType::TestOutput,
                                    &output_pin, 1, err);
}

void SyncControllerManager::reset_pin_state(uint8_t output_pin) {
    if (output_pin < 1 || output_pin > gw_sync::kOutputCount) return;
    std::lock_guard lk(impl_->pulses_mu);
    impl_->pins[output_pin] = {};
}

uint64_t SyncControllerManager::pop_pulse_ns(uint8_t output_pin,
                                             uint64_t camera_frame_id) {
    if (output_pin < 1 || output_pin > gw_sync::kOutputCount) return 0;
    std::lock_guard lk(impl_->pulses_mu);
    auto& state = impl_->pins[output_pin];
    if (state.pulses.empty()) return 0;
    size_t consume = 1;
    if (state.last_camera_frame_id) {
        if (camera_frame_id <= *state.last_camera_frame_id) return 0;
        consume = static_cast<size_t>(camera_frame_id - *state.last_camera_frame_id);
    }
    consume = std::min(consume, state.pulses.size());
    for (size_t i = 1; i < consume; ++i) state.pulses.pop_front();
    const uint64_t t_us = state.pulses.front().second;
    state.pulses.pop_front();
    state.last_camera_frame_id = camera_frame_id;
    return t_us * 1000ull;
}

void SyncControllerManager::Impl::run() {
    while (!stop_flag.load(std::memory_order_acquire)) {
        if (fd < 0) {
            if (!try_connect()) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(kReconnectDelayMs));
                continue;
            }
            request_resync();
        }
        pollfd pfd{fd, POLLIN, 0};
        const int pr = ::poll(&pfd, 1, kReadTimeoutMs);
        if (pr < 0) {
            if (errno == EINTR) continue;
            note_error(std::string("USB poll: ") + std::strerror(errno));
            close_fd();
            continue;
        }
        if (pr <= 0) continue;
        if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
            note_error("sync controller disconnected");
            close_fd();
            continue;
        }
        if (pfd.revents & POLLIN) {
            uint8_t bytes[1024];
            const ssize_t n = ::read(fd, bytes, sizeof(bytes));
            if (n > 0) {
                decoder.feed(bytes, static_cast<size_t>(n));
                std::lock_guard lk(status_mu);
                imu_samples_total = decoder.stats().imu_samples;
                imu_crc_errors = decoder.stats().crc_errors;
            } else if (!(n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
                note_error("sync controller read EOF");
                close_fd();
            }
        }
    }
}

void SyncControllerManager::Impl::run_resync() {
    while (!stop_flag.load(std::memory_order_acquire)) {
        std::unique_lock lk(resync_mu);
        resync_cv.wait_for(lk, std::chrono::seconds(3), [&] {
            return stop_flag.load() || resync_needed.load();
        });
        if (stop_flag.load()) return;
        if (!resync_needed.exchange(false)) continue;
        lk.unlock();

        std::vector<GroupConfig> groups;
        bool arm_target = false;
        {
            std::lock_guard cfg_lk(cfg_mu);
            groups = desired_cfg;
            arm_target = want_armed;
        }
        if (groups.empty() || !connected.load()) continue;
        std::string err;
        if (!send_full_config(groups, arm_target, err)) {
            note_error("configuration resync: " + err + " (will retry)");
            resync_needed.store(true);
        }
    }
}

bool SyncControllerManager::Impl::try_connect() {
    for (const auto& path : glob_devices(device_glob)) {
        const int candidate = open_serial(path);
        if (candidate < 0) continue;
        gw_sync::DeviceInfo info;
        if (!probe_controller(candidate, info)) {
            ::close(candidate);
            continue;
        }
        fd = candidate;
        decoder.reset();
        {
            std::lock_guard lk(status_mu);
            open_port = path;
            board = "micoair_f405_v2";
            firmware_version = info.firmware_version;
            reset_reason = info.reset_reason;
            last_error.reset();
            connected.store(true);
        }
        std::cerr << "SyncControllerManager: connected " << path
                  << " (MicoAir F405 V2 fw=" << info.firmware_version << ")\n";
        return true;
    }
    return false;
}

void SyncControllerManager::Impl::close_fd() {
    std::lock_guard cmd_lk(cmd_mu);
    if (fd >= 0) ::close(fd);
    fd = -1;
    cmd_pending = false;
    cmd_cv.notify_all();
    std::lock_guard status_lk(status_mu);
    open_port.reset();
    connected.store(false);
    armed = false;
    imu_ok = false;
    imu_rate_hz = 0.0;
    imu_window_count = 0;
    imu_sample_seen = false;
}

void SyncControllerManager::Impl::setup_decoder_callbacks() {
    decoder.on_device_info = [this](uint16_t, const gw_sync::DeviceInfo& info) {
        if (info.board_id != gw_sync::kBoardIdMicoAirF405V2) return;
        std::lock_guard lk(status_mu);
        board = "micoair_f405_v2";
        firmware_version = info.firmware_version;
        reset_reason = info.reset_reason;
    };
    decoder.on_ack = [this](const SyncProtocolDecoder::Ack& ack) {
        std::lock_guard lk(cmd_mu);
        if (!cmd_pending || ack.request_id != pending_request_id ||
            ack.command != pending_command) return;
        cmd_status = ack.status;
        cmd_pending = false;
        cmd_cv.notify_all();
    };
    decoder.on_trigger = [this](const gw_sync::TriggerEvent& event) {
        if (event.slot >= gw_sync::kMaxGroups) return;
        std::vector<uint8_t> outputs;
        {
            std::lock_guard lk(slot_pins_mu);
            outputs = slot_pins[event.slot];
        }
        if (outputs.empty()) return;
        feed_controller_sync(event.t_us, false);
        {
            std::lock_guard lk(pulses_mu);
            for (uint8_t output : outputs) {
                auto& state = pins[output];
                state.pulses.emplace_back(event.index, event.t_us);
                while (state.pulses.size() > kPulseRingMax) state.pulses.pop_front();
            }
        }
        {
            std::lock_guard lk(status_mu);
            last_pulse_at = std::chrono::steady_clock::now();
            last_pulse_seen = true;
            ++total_pulses;
        }
    };
    decoder.on_imu = [this](const gw::ImuSample& sample) {
        imu_bus.publish(sample);
        feed_controller_sync(sample.t_ns / 1000ull, true);
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard lk(status_mu);
        imu_last_sample_at = now;
        imu_sample_seen = true;
        if (imu_window_count == 0) imu_window_start = now;
        ++imu_window_count;
        const auto elapsed = now - imu_window_start;
        if (elapsed >= std::chrono::seconds(1)) {
            imu_rate_hz = static_cast<double>(imu_window_count) /
                          std::chrono::duration<double>(elapsed).count();
            imu_window_count = 0;
        }
    };
    decoder.on_heartbeat = [this](const gw_sync::Heartbeat& heartbeat) {
        std::lock_guard lk(status_mu);
        imu_ok = (heartbeat.flags & gw_sync::kHeartbeatImuOk) != 0;
        armed = (heartbeat.flags & gw_sync::kHeartbeatArmed) != 0;
        imu_fw_drops = heartbeat.imu_drops;
        trigger_fw_drops = heartbeat.trigger_drops;
        usb_errors = heartbeat.usb_errors;
    };
}

void SyncControllerManager::Impl::request_resync() {
    resync_needed.store(true);
    resync_cv.notify_one();
}

bool SyncControllerManager::Impl::send_command(gw_sync::MessageType type,
                                                const uint8_t* payload,
                                                uint16_t payload_len,
                                                std::string& err) {
    if (send_command_once(type, payload, payload_len, err)) return true;
    std::cerr << "SyncControllerManager: retrying command after '" << err << "'\n";
    return send_command_once(type, payload, payload_len, err);
}

bool SyncControllerManager::Impl::send_command_once(gw_sync::MessageType type,
                                                     const uint8_t* payload,
                                                     uint16_t payload_len,
                                                     std::string& err) {
    err.clear();
    std::unique_lock lk(cmd_mu);
    if (fd < 0) {
        err = "sync controller not connected";
        return false;
    }
    uint16_t request_id = next_request_id++;
    if (request_id == 0) request_id = next_request_id++;
    uint8_t frame[gw_sync::kMaxFrameBytes];
    const size_t len = gw_sync::build_frame(type, request_id, payload,
                                             payload_len, frame);
    if (len == 0) {
        err = "command payload too large";
        return false;
    }
    cmd_pending = true;
    pending_request_id = request_id;
    pending_command = type;
    cmd_status.reset();
    if (!write_all(fd, frame, len, err)) {
        cmd_pending = false;
        return false;
    }
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(kCommandAckMs);
    while (cmd_pending && !stop_flag.load()) {
        if (cmd_cv.wait_until(lk, deadline) == std::cv_status::timeout) {
            cmd_pending = false;
            err = "sync controller command timeout";
            return false;
        }
    }
    if (!cmd_status) {
        err = "sync controller disconnected";
        return false;
    }
    if (*cmd_status != gw_sync::AckStatus::Ok) {
        err = ack_status_string(*cmd_status);
        return false;
    }
    return true;
}

bool SyncControllerManager::Impl::send_full_config(
        const std::vector<GroupConfig>& groups, bool arm_target,
        std::string& err) {
    if (groups.size() > gw_sync::kMaxGroups) {
        err = "too many trigger groups";
        return false;
    }
    uint8_t payload[4 + gw_sync::kMaxGroups * 8] = {};
    payload[0] = static_cast<uint8_t>(groups.size());
    uint8_t all_pins = 0;
    size_t off = 4;
    for (size_t slot = 0; slot < groups.size(); ++slot) {
        gw_sync::GroupConfig wire;
        wire.slot = static_cast<uint8_t>(slot);
        wire.rate_millihz = static_cast<uint32_t>(
            std::llround(groups[slot].fps * 1000.0));
        for (uint8_t output : groups[slot].output_pins) {
            if (output < 1 || output > gw_sync::kOutputCount) {
                err = "trigger output must be 1..6";
                return false;
            }
            const uint8_t bit = static_cast<uint8_t>(1u << (output - 1));
            if (all_pins & bit) {
                err = "trigger output assigned to multiple groups";
                return false;
            }
            all_pins = static_cast<uint8_t>(all_pins | bit);
            wire.pin_mask = static_cast<uint8_t>(wire.pin_mask | bit);
        }
        if (wire.pin_mask == 0 || wire.rate_millihz == 0 ||
            wire.rate_millihz > gw_sync::kMaxRateMilliHz) {
            err = "invalid trigger group";
            return false;
        }
        off += gw_sync::encode_group_config(wire, payload + off);
    }
    if (!send_command(gw_sync::MessageType::SetConfig, payload,
                      static_cast<uint16_t>(off), err)) return false;
    if (arm_target && !groups.empty()) {
        if (!send_command(gw_sync::MessageType::Arm, nullptr, 0, err)) return false;
        std::lock_guard lk(status_mu);
        armed = true;
    }
    return true;
}

void SyncControllerManager::Impl::set_slot_pins(
        const std::vector<GroupConfig>& groups) {
    std::lock_guard lk(slot_pins_mu);
    for (auto& outputs : slot_pins) outputs.clear();
    for (size_t i = 0; i < groups.size() && i < slot_pins.size(); ++i) {
        slot_pins[i] = groups[i].output_pins;
    }
}

void SyncControllerManager::Impl::feed_controller_sync(uint64_t controller_us,
                                                        bool from_imu) {
    const uint64_t host_us = gw::Clock::now_ns() / 1000ull;
    std::lock_guard lk(sync_mu);
    if (from_imu) {
        last_imu_feed_host_us = host_us;
    } else if (host_us < last_imu_feed_host_us + 500'000) {
        return;
    }
    controller_sync.feed(controller_us, host_us);
}

void SyncControllerManager::Impl::note_error(std::string error) {
    std::cerr << "SyncControllerManager: " << error << "\n";
    std::lock_guard lk(status_mu);
    last_error = std::move(error);
}

}  // namespace gw::server
