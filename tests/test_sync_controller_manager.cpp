#include <gtest/gtest.h>

#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <util.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "firmware/include/sync_controller_protocol.h"
#include "server/sync_controller_manager.hpp"

namespace gw::server {
namespace {

using namespace std::chrono_literals;

bool wait_for(const std::function<bool()>& predicate, int timeout_ms = 4000) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) return true;
        std::this_thread::sleep_for(10ms);
    }
    return predicate();
}

class FakeController {
public:
    explicit FakeController(const std::filesystem::path& link) : link_(link) {
        int slave = -1;
        char name[128]{};
        if (::openpty(&master_, &slave, name, nullptr, nullptr) != 0) {
            throw std::runtime_error("openpty failed");
        }
        termios tio{};
        ::tcgetattr(slave, &tio);
        ::cfmakeraw(&tio);
        ::tcsetattr(slave, TCSANOW, &tio);
        slave_ = slave;
        std::filesystem::remove(link_);
        if (::symlink(name, link_.c_str()) != 0) {
            throw std::runtime_error("symlink failed");
        }
        thread_ = std::thread([this] { run(); });
    }

    ~FakeController() {
        stopping_.store(true);
        if (master_ >= 0) ::close(master_);
        if (thread_.joinable()) thread_.join();
        if (slave_ >= 0) ::close(slave_);
        std::filesystem::remove(link_);
    }

    std::atomic<bool> respond{true};
    std::atomic<int> swallow_acks{0};

    bool armed() const { return armed_.load(); }
    size_t received(gw_sync::MessageType type) const {
        std::lock_guard lock(state_mu_);
        size_t count = 0;
        for (auto value : received_) if (value == type) ++count;
        return count;
    }

    void send_trigger(uint8_t slot, uint32_t index, uint64_t t_us) {
        uint8_t payload[16]{};
        payload[0] = slot;
        gw_sync::put_u32(payload, 4, index);
        gw_sync::put_u64(payload, 8, t_us);
        write_frame(gw_sync::MessageType::Trigger, 0, payload, sizeof(payload));
    }

    void send_imu(uint64_t t_us) {
        uint8_t payload[36]{};
        payload[0] = 1;
        size_t off = 4;
        off = gw_sync::put_u64(payload, off, t_us);
        for (float value : {1.f, 2.f, 9.8f, .1f, .2f, .3f}) {
            off = gw_sync::put_f32(payload, off, value);
        }
        write_frame(gw_sync::MessageType::ImuBatch, 0, payload,
                    static_cast<uint16_t>(off));
    }

    void send_heartbeat(bool imu_ok, bool armed) {
        uint8_t payload[32]{};
        size_t off = 0;
        off = gw_sync::put_u64(payload, off, 1'000'000);
        uint32_t flags = imu_ok ? gw_sync::kHeartbeatImuOk : 0;
        if (armed) flags |= gw_sync::kHeartbeatArmed;
        off = gw_sync::put_u32(payload, off, flags);
        off = gw_sync::put_u32(payload, off, 400);
        off = gw_sync::put_u32(payload, off, 2);
        off = gw_sync::put_u32(payload, off, 3);
        gw_sync::put_u32(payload, off, 4);
        write_frame(gw_sync::MessageType::Heartbeat, 0, payload, sizeof(payload));
    }

private:
    void write_frame(gw_sync::MessageType type, uint16_t request_id,
                     const uint8_t* payload, uint16_t payload_len) {
        uint8_t frame[gw_sync::kMaxFrameBytes];
        const size_t n = gw_sync::build_frame(type, request_id, payload,
                                               payload_len, frame);
        std::lock_guard lock(write_mu_);
        size_t offset = 0;
        while (offset < n) {
            const ssize_t written = ::write(master_, frame + offset, n - offset);
            if (written <= 0) return;
            offset += static_cast<size_t>(written);
        }
    }

    void reply_info(uint16_t request_id) {
        gw_sync::DeviceInfo info;
        info.board_id = gw_sync::kBoardIdMicoAirF405V2;
        info.firmware_version = gw_sync::kFirmwareVersion;
        info.output_count = gw_sync::kOutputCount;
        info.max_groups = gw_sync::kMaxGroups;
        info.capabilities = 0x1F;
        uint8_t payload[28];
        const size_t n = gw_sync::encode_device_info(info, payload);
        write_frame(gw_sync::MessageType::DeviceInfo, request_id, payload,
                    static_cast<uint16_t>(n));
    }

    void reply_ack(uint16_t request_id, gw_sync::MessageType command,
                   gw_sync::AckStatus status = gw_sync::AckStatus::Ok) {
        if (swallow_acks.load() > 0) {
            swallow_acks.fetch_sub(1);
            return;
        }
        const uint8_t payload[4] = {
            static_cast<uint8_t>(command), static_cast<uint8_t>(status), 0, 0};
        write_frame(gw_sync::MessageType::Ack, request_id, payload,
                    sizeof(payload));
    }

    void handle(gw_sync::MessageType type, uint16_t request_id,
                const uint8_t* payload, uint16_t payload_len) {
        {
            std::lock_guard lock(state_mu_);
            received_.push_back(type);
        }
        if (!respond.load()) return;
        if (type == gw_sync::MessageType::Hello) {
            reply_info(request_id);
        } else if (type == gw_sync::MessageType::SetConfig) {
            configured_.store(payload_len >= 4 && payload[0] > 0);
            armed_.store(false);
            reply_ack(request_id, type);
        } else if (type == gw_sync::MessageType::Arm) {
            const bool configured = configured_.load();
            armed_.store(configured);
            reply_ack(request_id, type, configured ? gw_sync::AckStatus::Ok
                                                   : gw_sync::AckStatus::NoConfig);
        } else if (type == gw_sync::MessageType::Stop) {
            armed_.store(false);
            reply_ack(request_id, type);
        } else if (type == gw_sync::MessageType::TestOutput) {
            const bool valid = payload_len == 1 && payload[0] >= 1 && payload[0] <= 6;
            reply_ack(request_id, type, valid ? gw_sync::AckStatus::Ok
                                              : gw_sync::AckStatus::BadConfig);
        }
    }

    void consume() {
        size_t consumed = 0;
        while (rx_.size() - consumed >= gw_sync::kHeaderBytes) {
            const uint8_t* p = rx_.data() + consumed;
            if (p[0] != gw_sync::kMagic0 || p[1] != gw_sync::kMagic1) {
                ++consumed;
                continue;
            }
            const uint16_t payload_len = gw_sync::get_u16(p, 6);
            const size_t n = gw_sync::kHeaderBytes + payload_len +
                             gw_sync::kTrailerBytes;
            if (rx_.size() - consumed < n) break;
            if (gw_sync::get_u16(p, gw_sync::kHeaderBytes + payload_len) ==
                gw_sync::crc16_ccitt(p + 2, 6 + payload_len)) {
                handle(static_cast<gw_sync::MessageType>(p[3]),
                       gw_sync::get_u16(p, 4), p + gw_sync::kHeaderBytes,
                       payload_len);
                consumed += n;
            } else {
                ++consumed;
            }
        }
        rx_.erase(rx_.begin(), rx_.begin() + static_cast<ptrdiff_t>(consumed));
    }

    void run() {
        while (!stopping_.load()) {
            pollfd poll_fd{master_, POLLIN, 0};
            if (::poll(&poll_fd, 1, 50) <= 0) continue;
            uint8_t bytes[512];
            const ssize_t n = ::read(master_, bytes, sizeof(bytes));
            if (n <= 0) continue;
            rx_.insert(rx_.end(), bytes, bytes + n);
            consume();
        }
    }

    std::filesystem::path link_;
    int master_ = -1;
    int slave_ = -1;
    std::thread thread_;
    std::atomic<bool> stopping_{false};
    std::atomic<bool> configured_{false};
    std::atomic<bool> armed_{false};
    mutable std::mutex state_mu_;
    std::vector<gw_sync::MessageType> received_;
    std::mutex write_mu_;
    std::vector<uint8_t> rx_;
};

class SyncControllerManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        static std::atomic<unsigned> counter{0};
        dir_ = std::filesystem::temp_directory_path() /
               ("gw_sync_controller_" + std::to_string(::getpid()) + "_" +
                std::to_string(counter.fetch_add(1)));
        std::filesystem::create_directories(dir_);
    }
    void TearDown() override { std::filesystem::remove_all(dir_); }
    std::filesystem::path port() const { return dir_ / "tty_controller"; }
    std::string glob() const { return (dir_ / "tty_*").string(); }
    std::filesystem::path dir_;
};

SyncControllerManager::GroupConfig group() {
    return {"cameras", 30.0, {1, 3}};
}

}  // namespace

TEST_F(SyncControllerManagerTest, ConnectsConfiguresAndArmsSinglePort) {
    FakeController fake(port());
    SyncControllerManager manager(glob());
    manager.start();
    ASSERT_TRUE(wait_for([&] { return manager.status().connected; }));
    EXPECT_EQ(manager.status().firmware_version, gw_sync::kFirmwareVersion);
    EXPECT_EQ(manager.status().board, "micoair_f405_v2");
    std::string error;
    ASSERT_TRUE(manager.push_config({group()}, error)) << error;
    EXPECT_TRUE(fake.armed());
    EXPECT_TRUE(manager.status().armed);
    EXPECT_EQ(fake.received(gw_sync::MessageType::SetConfig), 1u);
    EXPECT_EQ(fake.received(gw_sync::MessageType::Arm), 1u);
    manager.stop();
}

TEST_F(SyncControllerManagerTest, RetriesLostAckIdempotently) {
    FakeController fake(port());
    SyncControllerManager manager(glob());
    manager.start();
    ASSERT_TRUE(wait_for([&] { return manager.status().connected; }));
    fake.swallow_acks.store(1);
    std::string error;
    ASSERT_TRUE(manager.push_config({group()}, error)) << error;
    EXPECT_EQ(fake.received(gw_sync::MessageType::SetConfig), 2u);
    EXPECT_TRUE(fake.armed());
    manager.stop();
}

TEST_F(SyncControllerManagerTest, TestOutputUsesCorrelatedBinaryCommand) {
    FakeController fake(port());
    SyncControllerManager manager(glob());
    manager.start();
    ASSERT_TRUE(wait_for([&] { return manager.status().connected; }));
    std::string error;
    EXPECT_TRUE(manager.test_output(6, error)) << error;
    EXPECT_FALSE(manager.test_output(7, error));
    EXPECT_EQ(fake.received(gw_sync::MessageType::TestOutput), 1u);
    manager.stop();
}

TEST_F(SyncControllerManagerTest, TestOutputDoesNotDuplicateEdgeWhenAckIsLost) {
    FakeController fake(port());
    SyncControllerManager manager(glob());
    manager.start();
    ASSERT_TRUE(wait_for([&] { return manager.status().connected; }));
    fake.swallow_acks.store(1);
    std::string error;
    EXPECT_FALSE(manager.test_output(2, error));
    EXPECT_EQ(error, "sync controller command timeout");
    EXPECT_EQ(fake.received(gw_sync::MessageType::TestOutput), 1u);
    manager.stop();
}

TEST_F(SyncControllerManagerTest, ReconnectResyncsDesiredBootConfiguration) {
    SyncControllerManager manager(glob());
    std::string error;
    EXPECT_FALSE(manager.push_config({group()}, error));
    manager.start();
    FakeController fake(port());
    ASSERT_TRUE(wait_for([&] { return manager.status().connected; }, 6000));
    ASSERT_TRUE(wait_for([&] { return fake.armed(); }, 6000));
    EXPECT_TRUE(manager.status().armed);
    manager.stop();
}

TEST_F(SyncControllerManagerTest, MultiplexesTriggerImuAndHealth) {
    FakeController fake(port());
    SyncControllerManager manager(glob());
    auto subscription = manager.imu_bus().subscribe(8);
    manager.start();
    ASSERT_TRUE(wait_for([&] { return manager.status().connected; }));
    std::string error;
    ASSERT_TRUE(manager.push_config({group()}, error)) << error;

    fake.send_trigger(0, 1, 100);
    fake.send_trigger(0, 2, 200);
    fake.send_imu((1ull << 32) + 12);
    fake.send_heartbeat(true, true);
    ASSERT_TRUE(wait_for([&] {
        const auto status = manager.status();
        return status.total_pulses == 2 && status.imu_samples == 1 &&
               status.imu_ok && status.usb_errors == 4;
    }));
    EXPECT_EQ(manager.pop_pulse_ns(1, 10), 100'000u);
    EXPECT_EQ(manager.pop_pulse_ns(3, 10), 100'000u);
    EXPECT_EQ(manager.pop_pulse_ns(1, 11), 200'000u);
    gw::ImuSample sample;
    ASSERT_TRUE(manager.imu_bus().try_pop(subscription, sample));
    EXPECT_EQ(sample.t_ns, ((1ull << 32) + 12) * 1000);
    EXPECT_FLOAT_EQ(sample.accel[2], 9.8f);
    manager.imu_bus().unsubscribe(subscription);
    manager.stop();
}

}  // namespace gw::server
