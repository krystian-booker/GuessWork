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

// Lifecycle of a child process spawned by SubprocessJob.
//   Pending    — constructed but not yet start()ed
//   Running    — child is alive, reader thread draining its stdout+stderr
//   Succeeded  — exited with code 0
//   Failed     — exited with a non-zero code (or was killed by a signal we
//                didn't initiate)
//   Cancelled  — cancel() was called; child was SIGTERMed
enum class SubprocessState { Pending, Running, Succeeded, Failed, Cancelled };

const char* to_string(SubprocessState s);

// Runs an external command, captures combined stdout+stderr into both a
// shared in-memory buffer and a file on disk, and exposes a wait-for-new-log
// API consumers can use for SSE pumps or test polling.
//
// Thread model:
//   - Reader thread drains the pipe in a tight read() loop, holds mu_ to
//     append to log_buffer_, writes the same bytes to log_file_, notifies cv_.
//   - On pipe EOF, reader reaps the child via waitpid(), transitions state_,
//     fires on_exit_ (under no lock), then notifies cv_ a final time.
//   - cancel() sends SIGTERM to the child's process group; the reader sees
//     EOF naturally and the normal exit path handles the rest. Idempotent.
//
// Not movable/copyable — holds an fd, a pid, and a joinable thread.
class SubprocessJob {
public:
    // Each pair is name=value; appended to (or overrides in) the inherited
    // environment of the child. Intentionally narrow: we only need PATH and
    // GW_KALIBR_FOCAL_HINT today.
    using EnvOverrides = std::vector<std::pair<std::string, std::string>>;

    SubprocessJob(std::vector<std::string> argv,
                  EnvOverrides             env_overrides = {},
                  std::filesystem::path    log_file_path = {});
    ~SubprocessJob();

    SubprocessJob(const SubprocessJob&)            = delete;
    SubprocessJob& operator=(const SubprocessJob&) = delete;

    // Fires the on_exit callback once the child has been reaped. The callback
    // runs on the reader thread, before the final cv_ notification, so SSE
    // subscribers see a consistent terminal state when they wake.
    void set_on_exit(std::function<void(SubprocessState, int /*exit_code*/)> cb);

    // Fork + execvp. Throws std::runtime_error on pipe/fork failure.
    // Idempotent: re-starting an already-started job throws.
    void start();

    // Send SIGTERM to the child's process group. Safe to call from any thread,
    // any number of times. Returns immediately; the reader thread completes
    // the lifecycle. No-op if the job has already finished.
    void cancel();

    SubprocessState state()         const { return state_.load(std::memory_order_acquire); }
    int             exit_code()     const { return exit_code_.load(std::memory_order_acquire); }
    uint64_t        started_at_ms() const { return started_at_ms_.load(std::memory_order_acquire); }
    uint64_t        ended_at_ms()   const { return ended_at_ms_.load(std::memory_order_acquire); }

    // Snapshot of the log buffer's size in bytes. Monotonic.
    size_t log_bytes() const;

    // Full copy of the log buffer. O(log_bytes); use sparingly.
    std::string log_snapshot() const;

    // Returns `length` bytes from `offset`. If offset >= log_bytes_, returns
    // empty; if offset + length > log_bytes_, returns what's available.
    std::string log_slice(size_t offset, size_t length) const;

    // Wait until either log_bytes_ exceeds `offset`, the job reaches a
    // terminal state, or `timeout` elapses. Returns the post-wait log_bytes_.
    // If the return value == offset and the state is still Running, no new
    // data arrived during the timeout window (the caller can loop).
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

    // Set before start(); read on the reader thread after exit.
    std::function<void(SubprocessState, int)> on_exit_;
};

}  // namespace gw::server
