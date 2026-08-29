#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>


#include "core/imu_types.hpp"
#include "server/imu_log_recorder.hpp"
#include "server/sync_controller_manager.hpp"

namespace gw::server {

namespace {

class ImuLogRecorderTest : public ::testing::Test {
protected:
    void SetUp() override {
        dir_ = std::filesystem::temp_directory_path() /
               ("gw_imu_log_test_" + std::to_string(::getpid()) + "_" +
                ::testing::UnitTest::GetInstance()->current_test_info()->name());
        std::filesystem::remove_all(dir_);
    }
    void TearDown() override { std::filesystem::remove_all(dir_); }

    // SyncControllerManager is constructed but never start()ed — its imu_bus works
    // without the I/O thread, and tests must not probe real serial devices.
    SyncControllerManager         controller_;
    std::filesystem::path dir_;
};

gw::ImuSample sample(uint64_t t_ns, float seed) {
    gw::ImuSample s;
    s.t_ns = t_ns;
    for (int i = 0; i < 3; ++i) {
        s.accel[i] = seed + static_cast<float>(i);
        s.gyro[i]  = -seed - static_cast<float>(i);
    }
    return s;
}

// Polls until the recorder reports `samples` consumed (drain thread races
// the publisher).
void wait_for_samples(ImuLogRecorder& rec, uint64_t samples) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (rec.status().samples < samples &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

}  // namespace

TEST_F(ImuLogRecorderTest, RecordsBitExactRecords) {
    ImuLogRecorder rec(controller_, dir_);
    std::string    err;
    ASSERT_TRUE(rec.start(3600, err)) << err;

    constexpr int kN = 1000;
    for (int i = 0; i < kN; ++i) {
        controller_.imu_bus().publish(
            sample(1'000'000'000ull + static_cast<uint64_t>(i) * 2'500'000ull,
                   static_cast<float>(i) * 0.5f));
    }
    wait_for_samples(rec, kN);
    EXPECT_TRUE(rec.stop());

    const auto st = rec.status();
    EXPECT_FALSE(st.recording);
    EXPECT_EQ(st.samples, static_cast<uint64_t>(kN));
    EXPECT_EQ(st.bytes, static_cast<uint64_t>(kN) * 32);

    const auto log = rec.newest_log();
    ASSERT_TRUE(log.has_value());
    EXPECT_EQ(std::filesystem::file_size(*log), static_cast<uint64_t>(kN) * 32);

    // First + last records parse back bit-exact.
    std::ifstream in(*log, std::ios::binary);
    ASSERT_TRUE(in.is_open());
    uint8_t rec0[32], recN[32];
    in.read(reinterpret_cast<char*>(rec0), 32);
    in.seekg(static_cast<std::streamoff>((kN - 1) * 32));
    in.read(reinterpret_cast<char*>(recN), 32);

    uint64_t t0, tN;
    float    a0[3], gN[3];
    std::memcpy(&t0, rec0, 8);
    std::memcpy(a0, rec0 + 8, 12);
    std::memcpy(&tN, recN, 8);
    std::memcpy(gN, recN + 20, 12);
    EXPECT_EQ(t0, 1'000'000'000ull);
    EXPECT_FLOAT_EQ(a0[0], 0.0f);
    EXPECT_EQ(tN, 1'000'000'000ull + 999ull * 2'500'000ull);
    EXPECT_FLOAT_EQ(gN[0], -999.0f * 0.5f);
}

TEST_F(ImuLogRecorderTest, DoubleStartRejected) {
    ImuLogRecorder rec(controller_, dir_);
    std::string    err;
    ASSERT_TRUE(rec.start(60, err));
    EXPECT_FALSE(rec.start(60, err));
    EXPECT_NE(err.find("in progress"), std::string::npos);
    rec.stop();
}

TEST_F(ImuLogRecorderTest, AutoStopsAtDuration) {
    ImuLogRecorder rec(controller_, dir_);
    std::string    err;
    ASSERT_TRUE(rec.start(1, err));  // 1 s of sync controller-clock span

    // 600 samples spanning 1.5 fake seconds at 400 Hz — auto-stop fires at
    // the 1 s mark (~401 samples consumed).
    for (int i = 0; i < 600; ++i) {
        controller_.imu_bus().publish(
            sample(static_cast<uint64_t>(i) * 2'500'000ull + 1, 0.0f));
    }
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (rec.status().recording &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto st = rec.status();
    EXPECT_FALSE(st.recording);
    EXPECT_GE(st.samples, 400u);
    EXPECT_LT(st.samples, 450u);
}

TEST_F(ImuLogRecorderTest, NewestLogPicksLatest) {
    ImuLogRecorder rec(controller_, dir_);
    std::filesystem::create_directories(dir_);
    std::ofstream(dir_ / "1000000000.bin").put('x');
    std::ofstream(dir_ / "2000000000.bin").put('x');
    const auto newest = rec.newest_log();
    ASSERT_TRUE(newest.has_value());
    EXPECT_EQ(newest->filename().string(), "2000000000.bin");
}

TEST_F(ImuLogRecorderTest, StopWhenIdleReturnsFalse) {
    ImuLogRecorder rec(controller_, dir_);
    EXPECT_FALSE(rec.stop());
}

}  // namespace gw::server
