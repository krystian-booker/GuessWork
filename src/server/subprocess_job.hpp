#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <sys/types.h>   // pid_t
#include <thread>
#include <utility>
#include <vector>

namespace gw::server {

// Failed covers non-zero exit and signal-killed (when we didn't send the
// signal); Cancelled is reserved for cancel()-initiated terminations.
enum class SubprocessState { Pending, Running, Succeeded, Failed, Cancelled };

const char* to_string(SubprocessState s);

// Runs an external command, captures combined stdout+stderr into both an
// in-memory buffer and a log file, and exposes a wait-for-new-log API for
// SSE pumps / test polling.
//
// Thread model:
//   - Reader thread drains the pipe under mu_, mirrors to log_file_, notifies cv_.
//   - On pipe EOF the reader waitpid()s the child, transitions state_, fires
//     on_exit_ (no lock held), then a final cv_ notify.
//   - cancel() SIGTERMs the child's process group; the reader sees EOF
//     naturally and the normal exit path handles the rest. Idempotent.
//
// Not movable/copyable — holds an fd, a pid, and a joinable thread.
class SubprocessJob {
public:
    using EnvOverrides = std::vector<std::pair<std::string, std::string>>;

    SubprocessJob(std::vector<std::string> argv,
                  EnvOverrides             env_overrides = {},
                  std::filesystem::path    log_file_path = {});
    ~SubprocessJob();

    SubprocessJob(const SubprocessJob&)            = delete;
    SubprocessJob& operator=(const SubprocessJob&) = delete;

    // Must be set before start() — set on the calling thread, read on the
    // reader thread after exit, with no synchronisation in between. Fires
    // before the final cv_ notify so SSE subscribers see a consistent
    // terminal state when they wake.
    void set_on_exit(std::function<void(SubprocessState, int /*exit_code*/)> cb);

    // Fork + execvp. Throws std::runtime_error on pipe/fork failure.
    // Restarting an already-started job throws.
    void start();

    // SIGTERM the child's process group. Thread-safe, idempotent, no-op if
    // already finished. Returns immediately; the reader thread completes
    // the lifecycle.
    void cancel();

    SubprocessState state()         const { return state_.load(std::memory_order_acquire); }
    int             exit_code()     const { return exit_code_.load(std::memory_order_acquire); }
    uint64_t        started_at_ms() const { return started_at_ms_.load(std::memory_order_acquire); }
    uint64_t        ended_at_ms()   const { return ended_at_ms_.load(std::memory_order_acquire); }

    // Monotonic.
    size_t log_bytes() const;

    // Full copy; O(log_bytes), use sparingly.
    std::string log_snapshot() const;

    // Returns `length` bytes from `offset`, clamped to what's available.
    std::string log_slice(size_t offset, size_t length) const;

    // Wait until log_bytes > offset OR state is terminal OR timeout fires.
    // Returns the post-wait log_bytes; if it equals offset and state is
    // still Running, no new data arrived in the window.
    size_t wait_for_log(size_t offset, std::chrono::milliseconds timeout) const;

private:
    void reader_loop();
    void append_log(const char* data, size_t n);

    std::vector<std::string> argv_;
    EnvOverrides             env_;
    std::filesystem::path    log_path_;
    std::ofstream            log_file_;

    mutable std::mutex              mu_;
    mutable std::condition_variable cv_;
    std::string                     log_buffer_;       // guarded by mu_

    std::atomic<SubprocessState>  state_{SubprocessState::Pending};
    std::atomic<int>              exit_code_{0};
    std::atomic<uint64_t>         started_at_ms_{0};
    std::atomic<uint64_t>         ended_at_ms_{0};
    std::atomic<bool>             cancel_requested_{false};

    pid_t                         pid_           = -1;
    int                           pipe_read_fd_  = -1;
    std::thread                   reader_;

    std::function<void(SubprocessState, int)> on_exit_;
};

}  // namespace gw::server
