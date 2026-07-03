#include <gtest/gtest.h>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <util.h>  // openpty (macOS)

#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "firmware/src/telemetry_payloads.h"
#include "server/teensy_manager.hpp"

// TeensyManager integration tests against a PTY-backed fake Teensy: the
// manager's device glob is pointed at symlinks to PTY slaves, so the whole
// serial layer runs for real — discovery/PING classification, the command
// ack protocol (incl. the lost-ack retry), the failed-push convergence
// resync, TRIG pulse routing, and the telemetry decode path through the
// REAL shared firmware framing header.

namespace gw::server {

namespace {

using namespace std::chrono_literals;

bool wait_for(const std::function<bool()>& pred, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) return true;
        std::this_thread::sleep_for(10ms);
    }
    return pred();
}

// One PTY-backed fake serial device. The responder thread implements just
// enough of the fw=4 command protocol; knobs simulate the failure modes.
class FakePort {
public:
    // `respond_commands` false = the binary telemetry interface (never
    // answers PING — exactly how the manager classifies the two).
    FakePort(const std::filesystem::path& link, bool respond_commands)
        : link_(link), is_command_(respond_commands) {
        int slave = -1;
        char name[128];
        if (::openpty(&master_, &slave, name, nullptr, nullptr) != 0) {
            throw std::runtime_error("openpty failed");
        }
        termios tio{};
        ::tcgetattr(slave, &tio);
        ::cfmakeraw(&tio);
        ::tcsetattr(slave, TCSANOW, &tio);
        slave_fd_ = slave;  // keep open so the master never sees EOF
        std::filesystem::remove(link_);
        if (::symlink(name, link_.c_str()) != 0) {
            throw std::runtime_error("symlink failed");
        }
        reader_ = std::thread([this] { run(); });
    }

    ~FakePort() {
        stop_.store(true);
        if (master_ >= 0) ::close(master_);  // wakes the reader
        if (reader_.joinable()) reader_.join();
        if (slave_fd_ >= 0) ::close(slave_fd_);
        std::filesystem::remove(link_);
    }

    // ---- knobs ----
    std::atomic<int>  swallow_acks{0};   // eat responses for N commands
    std::atomic<bool> respond{true};     // false = dead firmware

    // ---- observability ----
    std::vector<std::string> received() const {
        std::lock_guard lk(mu_);
        return received_;
    }
    size_t count_received(const std::string& prefix) const {
        std::lock_guard lk(mu_);
        size_t n = 0;
        for (const auto& l : received_) {
            if (l.rfind(prefix, 0) == 0) ++n;
        }
        return n;
    }
    bool armed() const { return armed_.load(); }

    // ---- fake-side output ----
    void write_raw(const void* data, size_t len) {
        (void)!::write(master_, data, len);
    }
    void write_line(const std::string& s) {
        const std::string line = s + "\r\n";
        write_raw(line.data(), line.size());
    }

private:
    void run() {
        std::string acc;
        char        buf[512];
        while (!stop_.load()) {
            pollfd pfd{master_, POLLIN, 0};
            if (::poll(&pfd, 1, 50) <= 0) continue;
            const ssize_t n = ::read(master_, buf, sizeof(buf));
            if (n <= 0) {
                if (stop_.load()) return;
                continue;
            }
            acc.append(buf, static_cast<size_t>(n));
            size_t nl;
            while ((nl = acc.find('\n')) != std::string::npos) {
                std::string line = acc.substr(0, nl);
                acc.erase(0, nl + 1);
                while (!line.empty() &&
                       (line.back() == '\r' || line.back() == ' ')) {
                    line.pop_back();
                }
                if (!line.empty()) handle(line);
            }
        }
    }

    void handle(const std::string& line) {
        {
            std::lock_guard lk(mu_);
            received_.push_back(line);
        }
        if (!is_command_ || !respond.load()) return;
        if (swallow_acks.load() > 0) {
            swallow_acks.fetch_sub(1);
            return;
        }
        if (line == "PING") {
            write_line("PONG fw=4 outputs=6");
        } else if (line == "CFG_CLEAR") {
            groups_.store(0);
            armed_.store(false);
            write_line("OK");
        } else if (line.rfind("CFG ", 0) == 0) {
            groups_.fetch_add(1);
            write_line("OK");
        } else if (line == "ARM") {
            if (groups_.load() > 0) {
                armed_.store(true);
                write_line("OK");
            } else {
                write_line("ERR no groups configured");
            }
        } else if (line == "STOP") {
            armed_.store(false);
            write_line("OK");
        } else {
            write_line("ERR unknown command: " + line);
        }
    }

    std::filesystem::path link_;
    bool                  is_command_;
    int                   master_   = -1;
    int                   slave_fd_ = -1;
    std::thread           reader_;
    std::atomic<bool>     stop_{false};
    std::atomic<int>      groups_{0};
    std::atomic<bool>     armed_{false};
    mutable std::mutex       mu_;
    std::vector<std::string> received_;
};

class TeensyManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        static std::atomic<int> counter{0};
        dir_ = std::filesystem::temp_directory_path() /
               ("gw_teensy_test_" + std::to_string(::getpid()) + "_" +
                std::to_string(counter.fetch_add(1)));
        std::filesystem::create_directories(dir_);
    }
    void TearDown() override { std::filesystem::remove_all(dir_); }

    std::string glob() const { return (dir_ / "tty_*").string(); }
    // Sibling-pairing rule: telemetry is the path differing only in the
    // final character.
    std::filesystem::path cmd_link() const { return dir_ / "tty_1"; }
    std::filesystem::path tel_link() const { return dir_ / "tty_3"; }

    std::filesystem::path dir_;
};

TeensyManager::GroupConfig bench_group() {
    return {"bench", 30.0, {1}};
}

}  // namespace

TEST_F(TeensyManagerTest, ConnectsClassifiesAndArms) {
    FakePort fake(cmd_link(), /*respond_commands=*/true);
    TeensyManager mgr(glob());
    mgr.start();
    ASSERT_TRUE(wait_for([&] { return mgr.status().connected; }, 3000));
    EXPECT_EQ(mgr.status().fw_version.value_or(0), 4);

    std::string err;
    ASSERT_TRUE(mgr.push_config({bench_group()}, err)) << err;
    EXPECT_TRUE(mgr.status().armed);
    EXPECT_TRUE(fake.armed());
    EXPECT_EQ(fake.count_received("CFG_CLEAR"), 1u);
    EXPECT_EQ(fake.count_received("CFG name=bench"), 1u);
    EXPECT_EQ(fake.count_received("ARM"), 1u);
    mgr.stop();
}

TEST_F(TeensyManagerTest, RetriesCommandWhenAckIsLost) {
    FakePort fake(cmd_link(), true);
    TeensyManager mgr(glob());
    mgr.start();
    ASSERT_TRUE(wait_for([&] { return mgr.status().connected; }, 3000));

    // The fake eats exactly one response — the USB CDC stuck-partial-packet
    // failure this retry exists for. Idempotent commands make the second
    // attempt safe; the push must still succeed end-to-end.
    fake.swallow_acks.store(1);
    std::string err;
    ASSERT_TRUE(mgr.push_config({bench_group()}, err)) << err;
    EXPECT_TRUE(fake.armed());
    // Whichever command lost its ack was sent twice.
    const size_t total = fake.count_received("CFG_CLEAR") +
                         fake.count_received("CFG name=") +
                         fake.count_received("ARM");
    EXPECT_EQ(total, 4u);  // 3 commands + 1 retry
    mgr.stop();
}

TEST_F(TeensyManagerTest, ResyncConvergesAfterDeadFirmwareWindow) {
    FakePort fake(cmd_link(), true);
    TeensyManager mgr(glob());
    mgr.start();
    ASSERT_TRUE(wait_for([&] { return mgr.status().connected; }, 3000));

    // Firmware goes completely unresponsive (both attempts fail) — the push
    // reports failure but the desired state is remembered…
    fake.respond.store(false);
    std::string err;
    EXPECT_FALSE(mgr.push_config({bench_group()}, err));
    EXPECT_FALSE(fake.armed());

    // …and once the firmware answers again the io_thread's retry resync
    // converges without any further API involvement (field requirement:
    // transient faults must not leave the robot disarmed).
    fake.respond.store(true);
    EXPECT_TRUE(wait_for([&] { return fake.armed(); }, 10000));
    mgr.stop();
}

TEST_F(TeensyManagerTest, TrigEventsRouteToPulseRingWithFrameIdAlignment) {
    FakePort fake(cmd_link(), true);
    TeensyManager mgr(glob());
    mgr.start();
    ASSERT_TRUE(wait_for([&] { return mgr.status().connected; }, 3000));
    std::string err;
    ASSERT_TRUE(mgr.push_config({bench_group()}, err)) << err;

    for (uint32_t i = 1; i <= 4; ++i) {
        fake.write_line("TRIG g=bench idx=" + std::to_string(i) +
                        " t_us=" + std::to_string(i * 100));
    }
    ASSERT_TRUE(wait_for([&] { return mgr.status().total_pulses >= 4; }, 2000));

    // First pop baselines; consecutive frame ids consume one pulse each; a
    // frame-id gap (dropped camera frame) skips pulses to stay aligned.
    EXPECT_EQ(mgr.pop_pulse_ns(1, 100), 100'000u);
    EXPECT_EQ(mgr.pop_pulse_ns(1, 101), 200'000u);
    EXPECT_EQ(mgr.pop_pulse_ns(1, 103), 400'000u);  // skips t=300
    EXPECT_EQ(mgr.pop_pulse_ns(1, 104), 0u);        // ring drained
    mgr.stop();
}

TEST_F(TeensyManagerTest, TelemetrySiblingPairsAndDecodesRealFraming) {
    FakePort cmd(cmd_link(), true);
    FakePort tel(tel_link(), /*respond_commands=*/false);
    TeensyManager mgr(glob());
    mgr.start();
    ASSERT_TRUE(wait_for([&] { return mgr.status().connected; }, 3000));
    ASSERT_TRUE(
        wait_for([&] { return mgr.status().telemetry_connected; }, 6000));

    auto sub = mgr.imu_bus().subscribe(64);

    // IMU batch built with the REAL firmware framing header — the exact
    // bytes the Teensy would emit.
    namespace telem = gw_fw::telem;
    uint8_t payload[1 + 8 + 6 * 4];
    size_t  off    = 0;
    payload[off++] = 1;
    off            = telem::le_put_u64(payload, off, 123'456ull);
    for (float v : {1.f, 2.f, 9.8f, 0.1f, 0.2f, 0.3f}) {
        off = telem::le_put_f32(payload, off, v);
    }
    uint8_t frame[6 + sizeof(payload)];
    const size_t n = telem::build_telemetry_frame(telem::kBinTypeImuBatch,
                                                  payload, sizeof(payload),
                                                  frame);
    tel.write_raw(frame, n);

    gw::ImuSample s;
    ASSERT_TRUE(mgr.imu_bus().wait_pop(sub, s));
    EXPECT_EQ(s.t_ns, 123'456ull * 1000);
    EXPECT_FLOAT_EQ(s.accel[2], 9.8f);
    EXPECT_FLOAT_EQ(s.gyro[2], 0.3f);
    EXPECT_GE(mgr.status().imu_samples, 1u);

    mgr.imu_bus().unsubscribe(sub);
    mgr.stop();
}

}  // namespace gw::server
