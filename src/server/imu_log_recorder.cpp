#include "server/imu_log_recorder.hpp"

#include <chrono>
#include <cstring>
#include <fstream>
#include <mutex>
#include <thread>

#include "core/imu_types.hpp"
#include "core/measurement_bus.hpp"
#include "server/sync_controller_manager.hpp"

namespace gw::server {

namespace {

constexpr size_t kRecordBytes = 32;  // u64 t_ns + 3×f32 accel + 3×f32 gyro

void encode_record(const gw::ImuSample& s, uint8_t out[kRecordBytes]) {
    std::memcpy(out, &s.t_ns, 8);
    std::memcpy(out + 8, s.accel, 12);
    std::memcpy(out + 20, s.gyro, 12);
}

}  // namespace

struct ImuLogRecorder::Impl {
    mutable std::mutex mu;
    std::thread        thread;
    bool               recording = false;

    gw::MeasurementBus<gw::ImuSample>::SubscriberHandle sub;

    std::string current_file;  // basename
    uint64_t    samples = 0;
    uint64_t    bytes   = 0;
    uint64_t    first_t_ns = 0;
    uint64_t    last_t_ns  = 0;
    int64_t     duration_s = 0;

    // ~1 s rate window.
    std::chrono::steady_clock::time_point window_start{};
    uint64_t                              window_count = 0;
    double                                rate_hz      = 0.0;
};

ImuLogRecorder::ImuLogRecorder(SyncControllerManager& controller, std::filesystem::path dir)
    : impl_(std::make_unique<Impl>()), controller_(controller), dir_(std::move(dir)) {}

ImuLogRecorder::~ImuLogRecorder() { stop(); }

bool ImuLogRecorder::start(int64_t duration_s, std::string& err) {
    if (duration_s < 1 || duration_s > kMaxDurationS) {
        err = "duration_s must be 1.." + std::to_string(kMaxDurationS);
        return false;
    }
    std::error_code ec;
    std::filesystem::create_directories(dir_, ec);

    // Reap a finished previous drain thread OUTSIDE the mutex — its exit
    // path takes the mutex (joining under it would deadlock).
    std::thread prev;
    {
        std::lock_guard lk(impl_->mu);
        if (impl_->recording) {
            err = "recording already in progress";
            return false;
        }
        prev = std::move(impl_->thread);
    }
    if (prev.joinable()) prev.join();

    std::lock_guard lk(impl_->mu);
    if (impl_->recording) {  // lost a start/start race while unlocked
        err = "recording already in progress";
        return false;
    }
    const auto ts = std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
    const std::string basename = std::to_string(ts) + ".bin";
    const auto        path     = dir_ / basename;

    auto out = std::make_shared<std::ofstream>(path, std::ios::binary);
    if (!out->is_open()) {
        err = "cannot open " + path.string();
        return false;
    }

    impl_->recording    = true;
    impl_->current_file = basename;
    impl_->samples      = 0;
    impl_->bytes        = 0;
    impl_->first_t_ns   = 0;
    impl_->last_t_ns    = 0;
    impl_->duration_s   = duration_s;
    impl_->window_count = 0;
    impl_->rate_hz      = 0.0;
    impl_->sub          = controller_.imu_bus().subscribe(4096);

    // The thread gets value copies of the handle and span — it never reads
    // impl_->sub (stop() mutates that shared_ptr; concurrent access to the
    // same shared_ptr object is a data race).
    const uint64_t span_ns =
        static_cast<uint64_t>(duration_s) * 1'000'000'000ull;
    auto sub = impl_->sub;
    impl_->thread = std::thread([this, out, sub, span_ns] {
        gw::ImuSample s;
        while (controller_.imu_bus().wait_pop(sub, s)) {
            uint8_t rec[kRecordBytes];
            encode_record(s, rec);
            out->write(reinterpret_cast<const char*>(rec), kRecordBytes);

            bool done = false;
            {
                const auto now = std::chrono::steady_clock::now();
                std::lock_guard lk(impl_->mu);
                if (impl_->first_t_ns == 0) impl_->first_t_ns = s.t_ns;
                impl_->last_t_ns = s.t_ns;
                ++impl_->samples;
                impl_->bytes += kRecordBytes;
                if (impl_->window_count == 0) impl_->window_start = now;
                ++impl_->window_count;
                const auto elapsed = now - impl_->window_start;
                if (elapsed >= std::chrono::seconds(1)) {
                    impl_->rate_hz =
                        static_cast<double>(impl_->window_count) /
                        std::chrono::duration<double>(elapsed).count();
                    impl_->window_count = 0;
                }
                done = s.t_ns - impl_->first_t_ns >= span_ns;
            }
            if (done) break;
        }
        out->flush();
        std::lock_guard lk(impl_->mu);
        if (impl_->sub) {
            controller_.imu_bus().unsubscribe(impl_->sub);
            impl_->sub.reset();
        }
        impl_->recording = false;
        impl_->rate_hz   = 0.0;
    });
    return true;
}

bool ImuLogRecorder::stop() {
    bool        was_recording = false;
    std::thread t;
    {
        std::lock_guard lk(impl_->mu);
        was_recording = impl_->recording;
        if (!was_recording && !impl_->thread.joinable()) return false;
        if (impl_->sub) {
            // Wakes the blocked wait_pop; the drain thread finishes the file.
            controller_.imu_bus().unsubscribe(impl_->sub);
            impl_->sub.reset();
        }
        // Move the handle out so the join happens unlocked (the drain
        // thread's exit path takes this mutex).
        t = std::move(impl_->thread);
    }
    if (t.joinable()) t.join();
    return was_recording;
}

ImuLogRecorder::Status ImuLogRecorder::status() const {
    std::lock_guard lk(impl_->mu);
    Status st;
    st.recording = impl_->recording;
    if (!impl_->current_file.empty()) st.file = impl_->current_file;
    st.samples = impl_->samples;
    st.bytes   = impl_->bytes;
    st.rate_hz = impl_->recording ? impl_->rate_hz : 0.0;
    if (impl_->recording && impl_->first_t_ns != 0) {
        const int64_t elapsed_s = static_cast<int64_t>(
            (impl_->last_t_ns - impl_->first_t_ns) / 1'000'000'000ull);
        st.remaining_s = std::max<int64_t>(impl_->duration_s - elapsed_s, 0);
    } else if (impl_->recording) {
        st.remaining_s = impl_->duration_s;
    }
    return st;
}

std::optional<std::filesystem::path> ImuLogRecorder::newest_log() const {
    std::error_code ec;
    std::optional<std::filesystem::path> best;
    for (const auto& e : std::filesystem::directory_iterator(dir_, ec)) {
        if (!e.is_regular_file() || e.path().extension() != ".bin") continue;
        if (!best || e.path().filename() > best->filename()) best = e.path();
    }
    return best;
}

}  // namespace gw::server
