#include "server/subprocess_job.hpp"

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

namespace gw::server {

const char* to_string(SubprocessState s) {
    switch (s) {
        case SubprocessState::Pending:   return "pending";
        case SubprocessState::Running:   return "running";
        case SubprocessState::Succeeded: return "succeeded";
        case SubprocessState::Failed:    return "failed";
        case SubprocessState::Cancelled: return "cancelled";
    }
    return "unknown";
}

namespace {

uint64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// Build a NUL-terminated argv array for execvp(). The returned vector owns
// the underlying char* by virtue of the input strings staying alive.
std::vector<char*> build_argv(const std::vector<std::string>& argv) {
    std::vector<char*> out;
    out.reserve(argv.size() + 1);
    for (const auto& a : argv) out.push_back(const_cast<char*>(a.c_str()));
    out.push_back(nullptr);
    return out;
}

// Merge env_overrides into the inherited environment. Each override
// replaces an existing entry with the same key, or appends a new entry.
// Returns owned strings + a parallel NUL-terminated envp[] for execve.
struct EnvBlock {
    std::vector<std::string> owned;
    std::vector<char*>       envp;
};
EnvBlock make_env(const SubprocessJob::EnvOverrides& overrides) {
    EnvBlock out;
    // Copy parent env, replacing any keys we override along the way.
    for (char** e = environ; e && *e; ++e) {
        std::string_view entry(*e);
        const auto eq = entry.find('=');
        const std::string_view key = (eq == std::string_view::npos)
                                         ? entry
                                         : entry.substr(0, eq);
        bool replaced = false;
        for (const auto& [k, v] : overrides) {
            if (key == k) {
                out.owned.push_back(k + "=" + v);
                replaced = true;
                break;
            }
        }
        if (!replaced) out.owned.emplace_back(entry);
    }
    // Append overrides not present in the parent env.
    for (const auto& [k, v] : overrides) {
        bool present = false;
        for (const auto& s : out.owned) {
            if (s.size() > k.size() && s[k.size()] == '=' &&
                std::memcmp(s.data(), k.data(), k.size()) == 0) {
                present = true;
                break;
            }
        }
        if (!present) out.owned.push_back(k + "=" + v);
    }
    out.envp.reserve(out.owned.size() + 1);
    for (auto& s : out.owned) out.envp.push_back(s.data());
    out.envp.push_back(nullptr);
    return out;
}

}  // namespace

SubprocessJob::SubprocessJob(std::vector<std::string> argv,
                             EnvOverrides             env_overrides,
                             std::filesystem::path    log_file_path)
    : argv_(std::move(argv)),
      env_(std::move(env_overrides)),
      log_path_(std::move(log_file_path)) {
    if (argv_.empty()) {
        throw std::invalid_argument("SubprocessJob: argv must not be empty");
    }
}

SubprocessJob::~SubprocessJob() {
    cancel();
    if (reader_.joinable()) reader_.join();
    if (log_file_.is_open()) log_file_.close();
}

void SubprocessJob::set_on_exit(std::function<void(SubprocessState, int)> cb) {
    on_exit_ = std::move(cb);
}

void SubprocessJob::start() {
    auto expected = SubprocessState::Pending;
    if (!state_.compare_exchange_strong(expected, SubprocessState::Running)) {
        throw std::logic_error("SubprocessJob::start: already started");
    }

    if (!log_path_.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(log_path_.parent_path(), ec);
        log_file_.open(log_path_, std::ios::out | std::ios::binary | std::ios::trunc);
        // Don't fail the whole job on a missing log file; the in-memory
        // buffer is the source of truth for the UI. Log to stderr if open
        // fails so a misconfigured path is visible.
        if (!log_file_.is_open()) {
            std::fprintf(stderr,
                         "SubprocessJob: cannot open log file %s (errno=%d)\n",
                         log_path_.string().c_str(), errno);
        }
    }

    int pipefd[2];
    if (::pipe(pipefd) != 0) {
        state_.store(SubprocessState::Failed, std::memory_order_release);
        throw std::runtime_error(std::string("SubprocessJob: pipe(): ") +
                                 std::strerror(errno));
    }
    // Reads on the parent side don't block forever after the child exits —
    // the read() returns 0 on EOF, which is what we want. No O_NONBLOCK.
    ::fcntl(pipefd[0], F_SETFD, FD_CLOEXEC);

    started_at_ms_.store(now_ms(), std::memory_order_release);

    const pid_t pid = ::fork();
    if (pid < 0) {
        ::close(pipefd[0]);
        ::close(pipefd[1]);
        state_.store(SubprocessState::Failed, std::memory_order_release);
        throw std::runtime_error(std::string("SubprocessJob: fork(): ") +
                                 std::strerror(errno));
    }

    if (pid == 0) {
        // --- child ---
        // New process group so cancel() can kill the whole tree (e.g. the
        // docker client) without taking down the parent guesswork.
        ::setpgid(0, 0);

        // Redirect stdout + stderr to the pipe's write end. Close the read
        // end in the child to avoid keeping the parent's pipe open forever.
        ::close(pipefd[0]);
        ::dup2(pipefd[1], STDOUT_FILENO);
        ::dup2(pipefd[1], STDERR_FILENO);
        if (pipefd[1] != STDOUT_FILENO && pipefd[1] != STDERR_FILENO) {
            ::close(pipefd[1]);
        }
        // Detach from any controlling terminal stdin to avoid odd interactive
        // behavior in things like bash subscripts. Closing fd 0 is fine; if
        // anything actually reads it, it'll get EOF.
        ::close(STDIN_FILENO);

        // Build env + argv. macOS lacks execvpe(), so we swap `environ` to
        // our merged block and then exec with PATH resolution. Mutating
        // `environ` is safe in the post-fork child (single-threaded). The
        // envblk's storage outlives the call because exec replaces this
        // address space wholesale.
        auto envblk = make_env(env_);
        auto argv_v = build_argv(this->argv_);
        environ     = envblk.envp.data();
        ::execvp(argv_v[0], argv_v.data());

        // exec failed; surface a one-line error to the captured stderr
        // before exiting so the parent can see what went wrong.
        std::fprintf(stderr, "SubprocessJob: execvp(%s) failed: %s\n",
                     argv_v[0], std::strerror(errno));
        _exit(127);
    }

    // --- parent ---
    ::close(pipefd[1]);
    pid_          = pid;
    pipe_read_fd_ = pipefd[0];
    reader_       = std::thread([this] { reader_loop(); });
}

void SubprocessJob::cancel() {
    if (cancel_requested_.exchange(true)) return;
    if (pid_ > 0 && state_.load() == SubprocessState::Running) {
        // Kill the whole process group — `docker run` typically forks helpers
        // and we want them all to exit promptly.
        ::kill(-pid_, SIGTERM);
    }
}

void SubprocessJob::reader_loop() {
    char buf[4096];
    for (;;) {
        const ssize_t n = ::read(pipe_read_fd_, buf, sizeof(buf));
        if (n > 0) {
            append_log(buf, static_cast<size_t>(n));
        } else if (n == 0) {
            break;  // EOF: child closed its pipe ends.
        } else if (errno == EINTR) {
            continue;
        } else {
            // Unexpected pipe error. Append a diagnostic and move on to
            // waitpid so we still reap the child.
            const std::string err = std::string("\n[subprocess: read error: ") +
                                    std::strerror(errno) + "]\n";
            append_log(err.data(), err.size());
            break;
        }
    }
    ::close(pipe_read_fd_);
    pipe_read_fd_ = -1;

    int status = 0;
    if (::waitpid(pid_, &status, 0) < 0 && errno != ECHILD) {
        const std::string err = std::string("\n[subprocess: waitpid: ") +
                                std::strerror(errno) + "]\n";
        append_log(err.data(), err.size());
    }
    pid_ = -1;

    SubprocessState final_state = SubprocessState::Failed;
    int             code        = 0;
    if (WIFEXITED(status)) {
        code = WEXITSTATUS(status);
        if (cancel_requested_.load()) {
            final_state = SubprocessState::Cancelled;
        } else if (code == 0) {
            final_state = SubprocessState::Succeeded;
        } else {
            final_state = SubprocessState::Failed;
        }
    } else if (WIFSIGNALED(status)) {
        code = -WTERMSIG(status);  // negative signal number signals "killed"
        final_state = cancel_requested_.load() ? SubprocessState::Cancelled
                                               : SubprocessState::Failed;
    }

    exit_code_.store(code, std::memory_order_release);
    ended_at_ms_.store(now_ms(), std::memory_order_release);
    state_.store(final_state, std::memory_order_release);

    if (on_exit_) {
        try { on_exit_(final_state, code); }
        catch (const std::exception& e) {
            const std::string err = std::string("\n[subprocess: on_exit threw: ") +
                                    e.what() + "]\n";
            append_log(err.data(), err.size());
        }
        catch (...) {
            const std::string err = "\n[subprocess: on_exit threw unknown exception]\n";
            append_log(err.data(), err.size());
        }
    }

    // Wake any SSE subscribers blocked in wait_for_log so they observe the
    // terminal state and stop streaming.
    {
        std::lock_guard lk(mu_);
        cv_.notify_all();
    }

    if (log_file_.is_open()) log_file_.close();
}

void SubprocessJob::append_log(const char* data, size_t n) {
    {
        std::lock_guard lk(mu_);
        log_buffer_.append(data, n);
        if (log_file_.is_open()) {
            log_file_.write(data, static_cast<std::streamsize>(n));
            log_file_.flush();
        }
    }
    cv_.notify_all();
}

size_t SubprocessJob::log_bytes() const {
    std::lock_guard lk(mu_);
    return log_buffer_.size();
}

std::string SubprocessJob::log_snapshot() const {
    std::lock_guard lk(mu_);
    return log_buffer_;
}

std::string SubprocessJob::log_slice(size_t offset, size_t length) const {
    std::lock_guard lk(mu_);
    if (offset >= log_buffer_.size()) return {};
    return log_buffer_.substr(offset, length);
}

size_t SubprocessJob::wait_for_log(size_t                    offset,
                                   std::chrono::milliseconds timeout) const {
    std::unique_lock lk(mu_);
    cv_.wait_for(lk, timeout, [&] {
        return log_buffer_.size() > offset ||
               state_.load(std::memory_order_acquire) != SubprocessState::Running;
    });
    return log_buffer_.size();
}

}  // namespace gw::server
