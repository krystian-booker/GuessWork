#include "server/kalibr_job.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <utility>

#include "server/camera_repository.hpp"

namespace gw::server {

KalibrLensConfig derive_kalibr_lens(double   focal_length_mm,
                                    double   pixel_pitch_mm,
                                    uint32_t sensor_width_px) {
    const double focal_px    = focal_length_mm / pixel_pitch_mm;
    const double sensor_w_mm = pixel_pitch_mm * static_cast<double>(sensor_width_px);
    const double hfov_rad    = 2.0 * std::atan((sensor_w_mm / 2.0) / focal_length_mm);
    const double hfov_deg    = hfov_rad * 180.0 / M_PI;
    return {
        static_cast<uint32_t>(std::lround(focal_px)),
        (hfov_deg >= 95.0) ? "pinhole-equi" : "pinhole-radtan",
    };
}

// Prepend Homebrew + /usr/local so the subprocess can find `docker` and
// `colima` when guesswork was launched from a minimal-env shell (e.g. macOS
// launchctl). Inherited PATH is appended by SubprocessJob's env merge.
std::string kalibr_augmented_path() {
    const char* parent_path = std::getenv("PATH");
    std::string p           = "/opt/homebrew/bin:/usr/local/bin";
    if (parent_path && *parent_path) {
        p += ":";
        p += parent_path;
    }
    return p;
}

namespace {

// Kalibr's results-cam.txt contains a line of the form:
//   reprojection error: [<mean_u>, <mean_v>] +- [<sigma_u>, <sigma_v>]
// The means are ~zero by construction; the sigmas (px) are the per-axis
// reprojection-error standard deviations we surface as the calibration's
// quality score.
struct ReprojSigmas {
    double u_px;
    double v_px;
};
std::optional<ReprojSigmas>
extract_reproj_sigmas(const std::filesystem::path& results_txt) {
    std::ifstream in(results_txt);
    if (!in.is_open()) return std::nullopt;
    const std::string body((std::istreambuf_iterator<char>(in)),
                            std::istreambuf_iterator<char>());

    static const std::regex re(
        R"(reprojection error:\s*\[[^\]]*\]\s*\+\-\s*\[\s*)"
        R"(([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*,\s*)"
        R"(([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*\])");
    std::smatch m;
    if (!std::regex_search(body, m, re)) return std::nullopt;
    try {
        return ReprojSigmas{std::stod(m[1].str()), std::stod(m[2].str())};
    } catch (...) {
        return std::nullopt;
    }
}

// Append a guesswork_meta block to the camchain so downstream code (cameras
// list, calibrate page) can render the quality score without re-reading
// results-cam.txt. Kalibr's loader ignores unknown top-level keys, but we
// never feed this YAML back into kalibr anyway.
std::string augment_with_quality(std::string yaml, const ReprojSigmas& sigmas) {
    const double rms = std::sqrt((sigmas.u_px * sigmas.u_px +
                                  sigmas.v_px * sigmas.v_px) / 2.0);
    if (!yaml.empty() && yaml.back() != '\n') yaml += '\n';
    std::ostringstream meta;
    meta << std::fixed << std::setprecision(6);
    meta << "guesswork_meta:\n"
         << "  reprojection_error_px: "   << rms          << "\n"
         << "  reprojection_error_u_px: " << sigmas.u_px  << "\n"
         << "  reprojection_error_v_px: " << sigmas.v_px  << "\n";
    yaml += meta.str();
    return yaml;
}

}  // namespace

KalibrJob::KalibrJob(CameraRepository&            repo,
                     int64_t                      camera_id,
                     std::filesystem::path        session_root,
                     const std::filesystem::path& calibrate_script_path,
                     const KalibrLensConfig&      lens)
    : repo_(repo),
      camera_id_(camera_id),
      session_root_(std::move(session_root)),
      model_(lens.model),
      sub_({calibrate_script_path.string(),
            session_root_.string(),
            lens.model},
           {{"PATH",                 kalibr_augmented_path()},
            {"GW_KALIBR_FOCAL_HINT", std::to_string(lens.focal_hint_px)}},
           session_root_ / "kalibr.log") {
    sub_.set_on_exit([this](SubprocessState s, int code) { handle_exit(s, code); });
}

KalibrJob::KalibrJob(CameraRepository&     repo,
                     int64_t               camera_id,
                     std::filesystem::path session_root,
                     double                focal_length_mm,
                     std::filesystem::path calibrate_script_path,
                     double                pixel_pitch_mm,
                     uint32_t              sensor_width_px)
    : KalibrJob(repo, camera_id, std::move(session_root),
                calibrate_script_path,
                derive_kalibr_lens(focal_length_mm, pixel_pitch_mm, sensor_width_px)) {}

KalibrJob::~KalibrJob() = default;

void KalibrJob::start() {
    sub_.start();
}

CalibrationJobStatus KalibrJob::status() const {
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

void KalibrJob::handle_exit(SubprocessState s, int /*exit_code*/) {
    if (s != SubprocessState::Succeeded) return;

    // Kalibr names its camchain `<bag-basename>-camchain.yaml`; we always
    // record into `<session>/calibration.bag`, so the name is fixed.
    const auto camchain = session_root_ / "calibration-camchain.yaml";

    std::ifstream in(camchain, std::ios::binary);
    if (!in.is_open()) {
        std::lock_guard lk(err_mu_);
        upload_error_ = "Kalibr exited 0 but " + camchain.string() + " was not produced";
        return;
    }
    std::string yaml((std::istreambuf_iterator<char>(in)),
                      std::istreambuf_iterator<char>());

    // Best-effort: enrich with quality metadata. If results-cam.txt is
    // missing or its format drifts we still store the bare camchain — the
    // rest of the pipeline tolerates a missing reprojection_error_px.
    const auto results_txt = session_root_ / "calibration-results-cam.txt";
    if (const auto sigmas = extract_reproj_sigmas(results_txt)) {
        yaml = augment_with_quality(std::move(yaml), *sigmas);
    }

    try {
        const auto stored = repo_.set_calibration(camera_id_, yaml);
        if (!stored) {
            std::lock_guard lk(err_mu_);
            upload_error_ = "camera row " + std::to_string(camera_id_) +
                            " missing during auto-upload";
            return;
        }
    } catch (const std::exception& e) {
        std::lock_guard lk(err_mu_);
        upload_error_ = std::string("auto-upload failed: ") + e.what();
        return;
    }

    calibration_stored_.store(true, std::memory_order_release);
    if (on_stored_) on_stored_(camera_id_);
}

}  // namespace gw::server
