#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include "server/subprocess_job.hpp"

namespace gw::server {

namespace {

class SubprocessJobTest : public ::testing::Test {
protected:
    void SetUp() override {
        log_path_ = std::filesystem::temp_directory_path() /
                    ("gw_subproc_test_" + std::to_string(::getpid()) + "_" +
                     ::testing::UnitTest::GetInstance()->current_test_info()->name() +
                     ".log");
        std::filesystem::remove(log_path_);
    }
    void TearDown() override {
        std::filesystem::remove(log_path_);
    }

    template <typename Pred>
    bool wait_until(Pred pred, std::chrono::milliseconds timeout) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (pred()) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return pred();
    }

    std::filesystem::path log_path_;
};

}  // namespace

TEST_F(SubprocessJobTest, EchoExitsZeroAndCapturesStdout) {
    SubprocessJob job({"/bin/echo", "hello world"}, {}, log_path_);
    job.start();
    EXPECT_TRUE(wait_until(
        [&] { return job.state() != SubprocessState::Running; },
        std::chrono::seconds(5)));
    EXPECT_EQ(job.state(), SubprocessState::Succeeded);
    EXPECT_EQ(job.exit_code(), 0);
    EXPECT_NE(job.log_snapshot().find("hello world"), std::string::npos);
}

TEST_F(SubprocessJobTest, NonzeroExitMarksFailed) {
    SubprocessJob job({"/bin/sh", "-c", "exit 7"}, {}, log_path_);
    job.start();
    EXPECT_TRUE(wait_until(
        [&] { return job.state() != SubprocessState::Running; },
        std::chrono::seconds(5)));
    EXPECT_EQ(job.state(), SubprocessState::Failed);
    EXPECT_EQ(job.exit_code(), 7);
}

TEST_F(SubprocessJobTest, MergesStdoutAndStderr) {
    // Both streams written; the merged log must contain both. Don't assume
    // ordering — small writes from separate descriptors can interleave.
    SubprocessJob job({"/bin/sh", "-c", "echo OUT; echo ERR 1>&2"}, {}, log_path_);
    job.start();
    EXPECT_TRUE(wait_until(
        [&] { return job.state() != SubprocessState::Running; },
        std::chrono::seconds(5)));
    EXPECT_EQ(job.state(), SubprocessState::Succeeded);
    const auto log = job.log_snapshot();
    EXPECT_NE(log.find("OUT"), std::string::npos);
    EXPECT_NE(log.find("ERR"), std::string::npos);
}

TEST_F(SubprocessJobTest, EnvOverridePropagatesToChild) {
    SubprocessJob job({"/bin/sh", "-c", "echo TOKEN=$GW_PROBE"},
                      {{"GW_PROBE", "1234"}}, log_path_);
    job.start();
    EXPECT_TRUE(wait_until(
        [&] { return job.state() != SubprocessState::Running; },
        std::chrono::seconds(5)));
    EXPECT_EQ(job.state(), SubprocessState::Succeeded);
    EXPECT_NE(job.log_snapshot().find("TOKEN=1234"), std::string::npos);
}

TEST_F(SubprocessJobTest, LogFileMirrorsBuffer) {
    SubprocessJob job({"/bin/sh", "-c", "printf 'line1\\nline2\\n'"}, {}, log_path_);
    job.start();
    EXPECT_TRUE(wait_until(
        [&] { return job.state() != SubprocessState::Running; },
        std::chrono::seconds(5)));
    ASSERT_TRUE(std::filesystem::exists(log_path_));
    std::ifstream f(log_path_, std::ios::binary);
    const std::string disk((std::istreambuf_iterator<char>(f)),
                            std::istreambuf_iterator<char>());
    EXPECT_EQ(disk, job.log_snapshot());
}

TEST_F(SubprocessJobTest, CancelDuringSleepTerminatesPromptly) {
    SubprocessJob job({"/bin/sh", "-c", "sleep 30"}, {}, log_path_);
    job.start();
    // Give the child a beat to actually be in sleep().
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_EQ(job.state(), SubprocessState::Running);
    job.cancel();
    EXPECT_TRUE(wait_until(
        [&] { return job.state() != SubprocessState::Running; },
        std::chrono::seconds(3)));
    EXPECT_EQ(job.state(), SubprocessState::Cancelled);
}

TEST_F(SubprocessJobTest, WaitForLogUnblocksOnNewOutput) {
    SubprocessJob job({"/bin/sh", "-c",
                       "echo first; sleep 0.2; echo second; sleep 0.2; echo third"},
                      {}, log_path_);
    job.start();
    // First batch: "first\n" should appear within a couple hundred ms.
    const size_t after_first = job.wait_for_log(0, std::chrono::seconds(2));
    EXPECT_GT(after_first, 0u);
    // Subsequent waits should each advance.
    const size_t after_second = job.wait_for_log(after_first, std::chrono::seconds(2));
    EXPECT_GT(after_second, after_first);

    EXPECT_TRUE(wait_until(
        [&] { return job.state() != SubprocessState::Running; },
        std::chrono::seconds(5)));
    EXPECT_EQ(job.state(), SubprocessState::Succeeded);
}

TEST_F(SubprocessJobTest, WaitForLogReturnsAtTerminal) {
    SubprocessJob job({"/bin/sh", "-c", "exit 0"}, {}, log_path_);
    job.start();
    // Spin until terminal so the wait below short-circuits via the state
    // predicate rather than the timeout predicate.
    EXPECT_TRUE(wait_until(
        [&] { return job.state() != SubprocessState::Running; },
        std::chrono::seconds(5)));
    const auto t0 = std::chrono::steady_clock::now();
    const size_t bytes = job.wait_for_log(0, std::chrono::seconds(5));
    const auto elapsed = std::chrono::steady_clock::now() - t0;
    EXPECT_LT(elapsed, std::chrono::milliseconds(500))
        << "wait_for_log should return immediately when terminal";
    (void)bytes;
}

TEST_F(SubprocessJobTest, OnExitFiresExactlyOnce) {
    std::atomic<int> calls{0};
    SubprocessState  observed = SubprocessState::Pending;
    int              observed_code = -1;

    SubprocessJob job({"/bin/sh", "-c", "exit 3"}, {}, log_path_);
    job.set_on_exit([&](SubprocessState s, int code) {
        ++calls;
        observed      = s;
        observed_code = code;
    });
    job.start();
    EXPECT_TRUE(wait_until(
        [&] { return job.state() != SubprocessState::Running; },
        std::chrono::seconds(5)));
    EXPECT_EQ(calls.load(), 1);
    EXPECT_EQ(observed, SubprocessState::Failed);
    EXPECT_EQ(observed_code, 3);
}

TEST_F(SubprocessJobTest, DestructorCancelsAndJoins) {
    {
        SubprocessJob job({"/bin/sh", "-c", "sleep 30"}, {}, log_path_);
        job.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        EXPECT_EQ(job.state(), SubprocessState::Running);
        // Falls off the scope; destructor must cancel + join the reader.
    }
    // If we made it here without hanging, the destructor cleaned up.
    SUCCEED();
}

TEST_F(SubprocessJobTest, ExecFailureMarksFailed) {
    SubprocessJob job({"/no/such/program/exists/anywhere"}, {}, log_path_);
    job.start();
    EXPECT_TRUE(wait_until(
        [&] { return job.state() != SubprocessState::Running; },
        std::chrono::seconds(5)));
    // Exit code 127 from our _exit(127) in the child after exec failure.
    EXPECT_EQ(job.state(), SubprocessState::Failed);
    EXPECT_EQ(job.exit_code(), 127);
    EXPECT_NE(job.log_snapshot().find("execvp"), std::string::npos);
}

}  // namespace gw::server
