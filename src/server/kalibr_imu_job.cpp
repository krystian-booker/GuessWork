#include "server/kalibr_imu_job.hpp"

#include <fstream>
#include <iomanip>
#include <iterator>
#include <map>
#include <regex>
#include <sstream>
#include <utility>

#include "calibration/calibration_store.hpp"
#include "server/camera_repository.hpp"

namespace gw::server {

namespace {

// Kalibr's results-imucam txt prints one line per camera of the form:
//   Reprojection error (cam0):     mean 0.123, median 0.101, std: 0.067
struct ReprojStats {
    double mean, median, std_dev;
};
std::map<size_t, ReprojStats>
extract_reproj_stats(const std::filesystem::path& results_txt) {
    std::map<size_t, ReprojStats> out;
    std::ifstream in(results_txt);
    if (!in.is_open()) return out;
    const std::string body((std::istreambuf_iterator<char>(in)),
                            std::istreambuf_iterator<char>());

    static const std::regex re(
        R"(Reprojection error \(cam(\d+)\):\s*mean ([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),\s*)"
        R"(median ([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),\s*std:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?))");
    for (auto it = std::sregex_iterator(body.begin(), body.end(), re);
         it != std::sregex_iterator(); ++it) {
        try {
            out[std::stoul((*it)[1].str())] = ReprojStats{
                std::stod((*it)[2].str()),
                std::stod((*it)[3].str()),
                std::stod((*it)[4].str()),
            };
        } catch (...) {
            // best-effort — skip malformed lines
        }
    }
    return out;
}

}  // namespace

std::vector<std::string> build_imu_extrinsics_payloads(
    const std::string& camchain_imucam_yaml, size_t expected_cams) {
    const auto chain = gw::calib::parse_camchain(camchain_imucam_yaml);
    if (chain.cameras.size() != expected_cams) {
        throw gw::calib::CalibrationParseError(
            "camchain-imucam has " + std::to_string(chain.cameras.size()) +
            " cameras, expected " + std::to_string(expected_cams));
    }
    std::vector<std::string> out;
    out.reserve(chain.cameras.size());
    for (const auto& [name, entry] : chain.cameras) {
        if (!entry.imu) {
            throw gw::calib::CalibrationParseError(
                "camchain-imucam entry " + name + " is missing T_cam_imu");
        }
        out.push_back(gw::calib::serialize_single_camera(entry));
    }
    return out;
}

std::string KalibrImuJob::join_hints(const std::vector<uint32_t>& hints) {
    std::ostringstream out;
    for (size_t i = 0; i < hints.size(); ++i) {
        if (i) out << ' ';
        out << hints[i];
    }
    return out.str();
}

KalibrImuJob::KalibrImuJob(CameraRepository&     repo,
                           std::vector<int64_t>  camera_ids,
                           std::filesystem::path session_root,
                           std::filesystem::path calibrate_imu_script_path,
                           std::string           model,
                           std::vector<uint32_t> focal_hints_px,
                           std::string           session_id)
    : repo_(repo),
      camera_ids_(std::move(camera_ids)),
      session_root_(std::move(session_root)),
      model_(std::move(model)),
      session_id_(std::move(session_id)),
      sub_({calibrate_imu_script_path.string(),
            session_root_.string(),
            std::to_string(camera_ids_.size()),
            model_},
           {{"PATH",                 kalibr_augmented_path()},
            {"GW_KALIBR_FOCAL_HINT", join_hints(focal_hints_px)}},
           session_root_ / "kalibr-imu.log") {
    sub_.set_on_exit([this](SubprocessState s, int code) { handle_exit(s, code); });
}

KalibrImuJob::~KalibrImuJob() = default;

void KalibrImuJob::start() {
    sub_.start();
}

CalibrationJobStatus KalibrImuJob::status() const {
    CalibrationJobStatus st;
    st.state              = sub_.state();
    st.model              = model_;
    st.exit_code          = sub_.exit_code();
    st.started_at_ms      = sub_.started_at_ms();
    st.ended_at_ms        = sub_.ended_at_ms();
    st.log_bytes          = sub_.log_bytes();
    st.calibration_stored = calibration_stored_.load(std::memory_order_acquire);
    {
        std::lock_guard lk(err_mu_);
        if (upload_error_) st.upload_error = *upload_error_;
    }
    return st;
}

void KalibrImuJob::handle_exit(SubprocessState s, int /*exit_code*/) {
    if (s != SubprocessState::Succeeded) return;

    // kalibr_calibrate_imu_camera names its output
    // camchain-imucam-<bag-basename>.yaml; the bag is always calibration.bag.
    const auto camchain_path = session_root_ / "camchain-imucam-calibration.yaml";
    std::ifstream in(camchain_path, std::ios::binary);
    if (!in.is_open()) {
        std::lock_guard lk(err_mu_);
        upload_error_ =
            "Kalibr exited 0 but " + camchain_path.string() + " was not produced";
        return;
    }
    const std::string yaml((std::istreambuf_iterator<char>(in)),
                            std::istreambuf_iterator<char>());

    std::vector<std::string>   payloads;
    gw::calib::Camchain        chain;
    try {
        payloads = build_imu_extrinsics_payloads(yaml, camera_ids_.size());
        chain    = gw::calib::parse_camchain(yaml);  // for per-camera timeshift
    } catch (const std::exception& e) {
        std::lock_guard lk(err_mu_);
        upload_error_ = std::string("camchain-imucam parse failed: ") + e.what();
        return;
    }

    // Best-effort quality stats — tolerate a missing/reshaped results file.
    const auto stats =
        extract_reproj_stats(session_root_ / "results-imucam-calibration.txt");

    std::string errors;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
        std::string payload = std::move(payloads[i]);
        if (!payload.empty() && payload.back() != '\n') payload += '\n';

        std::ostringstream meta;
        meta << std::setprecision(9);
        meta << "guesswork_meta:\n"
             << "  session_id: " << session_id_ << "\n"
             << "  source_cam_index: " << i << "\n"
             << "  timeshift_cam_imu_s: "
             << chain.cameras[i].second.imu->timeshift_cam_imu << "\n";
        if (const auto it = stats.find(i); it != stats.end()) {
            meta << "  reprojection_error_mean_px: "   << it->second.mean    << "\n"
                 << "  reprojection_error_median_px: " << it->second.median  << "\n"
                 << "  reprojection_error_std_px: "    << it->second.std_dev << "\n";
        }
        payload += meta.str();

        try {
            const auto stored = repo_.set_imu_extrinsics(camera_ids_[i], payload);
            if (!stored) {
                errors += "camera row " + std::to_string(camera_ids_[i]) +
                          " missing during auto-upload; ";
            }
        } catch (const std::exception& e) {
            errors += "camera " + std::to_string(camera_ids_[i]) + ": " + e.what() + "; ";
        }
    }

    if (!errors.empty()) {
        std::lock_guard lk(err_mu_);
        upload_error_ = errors;
        return;
    }
    calibration_stored_.store(true, std::memory_order_release);
}

}  // namespace gw::server
