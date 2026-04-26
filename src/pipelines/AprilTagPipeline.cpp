#include "posest/pipelines/AprilTagPipeline.h"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <utility>

#include <apriltag/apriltag.h>
#include <apriltag/common/image_types.h>
#include <apriltag/common/zarray.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <array>
#include <string_view>
#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>

#include "posest/pipelines/AprilTagPoseSolver.h"

namespace posest::pipelines {

namespace {

struct FamilyBinding {
    std::string_view name;
    apriltag_family_t* (*create)();
    void (*destroy)(apriltag_family_t*);
};

constexpr std::array<FamilyBinding, 5> kFamilyBindings{{
    {"tag36h11", &tag36h11_create, &tag36h11_destroy},
    {"tag25h9", &tag25h9_create, &tag25h9_destroy},
    {"tagStandard41h12", &tagStandard41h12_create, &tagStandard41h12_destroy},
    {"tagCircle21h7", &tagCircle21h7_create, &tagCircle21h7_destroy},
    {"tagCustom48h12", &tagCustom48h12_create, &tagCustom48h12_destroy},
}};

const FamilyBinding* findFamily(std::string_view name) {
    for (const auto& binding : kFamilyBindings) {
        if (binding.name == name) {
            return &binding;
        }
    }
    return nullptr;
}

cv::Mat toGray8(const cv::Mat& image) {
    if (image.empty()) {
        return {};
    }
    if (image.type() == CV_8UC1) {
        return image;
    }
    cv::Mat gray;
    if (image.type() == CV_8UC3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        return gray;
    }
    if (image.type() == CV_8UC4) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
        return gray;
    }
    throw std::runtime_error("AprilTagPipeline only supports 8-bit grayscale/BGR/BGRA frames");
}

template <typename T>
T jsonValueOr(const nlohmann::json& json, const char* key, T fallback) {
    if (!json.contains(key)) {
        return fallback;
    }
    return json.at(key).get<T>();
}

}  // namespace

AprilTagPipelineConfig parseAprilTagPipelineConfig(
    const runtime::PipelineConfig& pipeline_config) {
    AprilTagPipelineConfig config;
    const nlohmann::json params = pipeline_config.parameters_json.empty()
        ? nlohmann::json::object()
        : nlohmann::json::parse(pipeline_config.parameters_json);

    if (!params.is_object()) {
        throw std::invalid_argument("AprilTag pipeline parameters_json must be a JSON object");
    }

    config.family = jsonValueOr<std::string>(params, "family", config.family);
    config.nthreads = jsonValueOr<int>(params, "nthreads", config.nthreads);
    config.quad_decimate = jsonValueOr<double>(params, "quad_decimate", config.quad_decimate);
    config.quad_sigma = jsonValueOr<double>(params, "quad_sigma", config.quad_sigma);
    config.refine_edges = jsonValueOr<bool>(params, "refine_edges", config.refine_edges);
    config.decode_sharpening =
        jsonValueOr<double>(params, "decode_sharpening", config.decode_sharpening);
    config.debug = jsonValueOr<bool>(params, "debug", config.debug);
    config.calibration_version =
        jsonValueOr<std::string>(params, "calibration_version", config.calibration_version);
    config.field_layout_id =
        jsonValueOr<std::string>(params, "field_layout_id", config.field_layout_id);
    config.tag_size_m = jsonValueOr<double>(params, "tag_size_m", config.tag_size_m);

    if (params.contains("covariance")) {
        const auto& cov = params.at("covariance");
        if (!cov.is_object()) {
            throw std::invalid_argument(
                "AprilTag pipeline parameters_json.covariance must be a JSON object");
        }
        config.covariance.base_sigma_translation_m = jsonValueOr<double>(
            cov, "base_sigma_translation_m", config.covariance.base_sigma_translation_m);
        config.covariance.base_sigma_rotation_rad = jsonValueOr<double>(
            cov, "base_sigma_rotation_rad", config.covariance.base_sigma_rotation_rad);
        config.covariance.reference_distance_m = jsonValueOr<double>(
            cov, "reference_distance_m", config.covariance.reference_distance_m);
        config.covariance.reference_rms_px = jsonValueOr<double>(
            cov, "reference_rms_px", config.covariance.reference_rms_px);
        config.covariance.single_tag_translation_mult = jsonValueOr<double>(
            cov, "single_tag_translation_mult", config.covariance.single_tag_translation_mult);
        config.covariance.single_tag_rotation_mult = jsonValueOr<double>(
            cov, "single_tag_rotation_mult", config.covariance.single_tag_rotation_mult);
        config.covariance.ambiguity_drop_threshold = jsonValueOr<double>(
            cov, "ambiguity_drop_threshold", config.covariance.ambiguity_drop_threshold);
        config.covariance.multi_tag_decay_k = jsonValueOr<double>(
            cov, "multi_tag_decay_k", config.covariance.multi_tag_decay_k);
        config.covariance.well_spread_floor_mult = jsonValueOr<double>(
            cov, "well_spread_floor_mult", config.covariance.well_spread_floor_mult);
    }

    if (!findFamily(config.family)) {
        throw std::invalid_argument("Unsupported AprilTag family: " + config.family);
    }
    if (config.nthreads <= 0) {
        throw std::invalid_argument("AprilTag nthreads must be > 0");
    }
    if (config.quad_decimate <= 0.0) {
        throw std::invalid_argument("AprilTag quad_decimate must be > 0");
    }
    if (config.quad_sigma < 0.0) {
        throw std::invalid_argument("AprilTag quad_sigma must be >= 0");
    }
    if (config.decode_sharpening < 0.0) {
        throw std::invalid_argument("AprilTag decode_sharpening must be >= 0");
    }
    if (config.tag_size_m <= 0.0) {
        throw std::invalid_argument("AprilTag tag_size_m must be > 0");
    }
    if (config.covariance.base_sigma_translation_m <= 0.0) {
        throw std::invalid_argument(
            "AprilTag covariance.base_sigma_translation_m must be > 0");
    }
    if (config.covariance.base_sigma_rotation_rad <= 0.0) {
        throw std::invalid_argument(
            "AprilTag covariance.base_sigma_rotation_rad must be > 0");
    }
    if (config.covariance.reference_distance_m <= 0.0) {
        throw std::invalid_argument(
            "AprilTag covariance.reference_distance_m must be > 0");
    }
    if (config.covariance.reference_rms_px <= 0.0) {
        throw std::invalid_argument(
            "AprilTag covariance.reference_rms_px must be > 0");
    }
    if (config.covariance.single_tag_translation_mult <= 0.0) {
        throw std::invalid_argument(
            "AprilTag covariance.single_tag_translation_mult must be > 0");
    }
    if (config.covariance.single_tag_rotation_mult <= 0.0) {
        throw std::invalid_argument(
            "AprilTag covariance.single_tag_rotation_mult must be > 0");
    }
    if (config.covariance.ambiguity_drop_threshold <= 0.0 ||
        config.covariance.ambiguity_drop_threshold > 1.0) {
        throw std::invalid_argument(
            "AprilTag covariance.ambiguity_drop_threshold must be in (0, 1]");
    }
    if (config.covariance.multi_tag_decay_k <= 0.0) {
        throw std::invalid_argument(
            "AprilTag covariance.multi_tag_decay_k must be > 0");
    }
    if (config.covariance.well_spread_floor_mult <= 0.0 ||
        config.covariance.well_spread_floor_mult > 1.0) {
        throw std::invalid_argument(
            "AprilTag covariance.well_spread_floor_mult must be in (0, 1]");
    }

    return config;
}

AprilTagPipeline::AprilTagPipeline(
    std::string id,
    IMeasurementSink& sink,
    AprilTagPipelineConfig config)
    : VisionPipelineBase(std::move(id), "apriltag", sink), config_(std::move(config)) {
    const FamilyBinding* binding = findFamily(config_.family);
    if (!binding) {
        throw std::invalid_argument("Unsupported AprilTag family: " + config_.family);
    }
    family_ = binding->create();
    family_destroy_ = binding->destroy;
    detector_ = apriltag_detector_create();
    if (!family_ || !detector_) {
        throw std::runtime_error("Failed to create AprilTag detector");
    }

    detector_->nthreads = config_.nthreads;
    detector_->quad_decimate = static_cast<float>(config_.quad_decimate);
    detector_->quad_sigma = static_cast<float>(config_.quad_sigma);
    detector_->refine_edges = config_.refine_edges;
    detector_->decode_sharpening = config_.decode_sharpening;
    detector_->debug = config_.debug;
    apriltag_detector_add_family(detector_, family_);

    stats_.pipeline_id = this->id();
}

AprilTagPipelineStats AprilTagPipeline::stats() const {
    AprilTagPipelineStats out;
    {
        std::lock_guard<std::mutex> g(stats_mu_);
        out = stats_;
    }
    out.mailbox_drops = droppedByMailbox();
    return out;
}

void AprilTagPipeline::recordOutcome(
    Outcome outcome,
    std::chrono::steady_clock::time_point t0,
    double rms_px) {
    const auto t1 = std::chrono::steady_clock::now();
    const std::int64_t lat_us =
        std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    std::lock_guard<std::mutex> g(stats_mu_);
    ++stats_.frames_processed;
    switch (outcome) {
        case Outcome::NoDetection:          ++stats_.frames_no_detection; break;
        case Outcome::DroppedNoCalibration: ++stats_.frames_dropped_no_calibration; break;
        case Outcome::DroppedByAmbiguity:   ++stats_.frames_dropped_by_ambiguity; break;
        case Outcome::SolvedSingle:         ++stats_.frames_solved_single; break;
        case Outcome::SolvedMulti:          ++stats_.frames_solved_multi; break;
    }
    stats_.last_solve_latency_us = lat_us;
    stats_.max_solve_latency_us = std::max(stats_.max_solve_latency_us, lat_us);
    if (outcome == Outcome::SolvedSingle || outcome == Outcome::SolvedMulti) {
        stats_.last_reprojection_rms_px = rms_px;
    }
}

AprilTagPipeline::~AprilTagPipeline() {
    stop();
    if (detector_) {
        apriltag_detector_destroy(detector_);
        detector_ = nullptr;
    }
    if (family_) {
        if (family_destroy_) {
            family_destroy_(family_);
        }
        family_ = nullptr;
        family_destroy_ = nullptr;
    }
}

void AprilTagPipeline::processFrame(const Frame& frame) {
    const auto t0 = std::chrono::steady_clock::now();

    AprilTagObservation observation;
    observation.camera_id = frame.camera_id;
    observation.frame_sequence = frame.sequence;
    observation.capture_time = frame.capture_time;
    observation.teensy_time_us = frame.teensy_time_us;
    observation.trigger_sequence = frame.trigger_sequence;

    cv::Mat gray = toGray8(frame.image);
    if (gray.empty()) {
        measurementSink().publish(std::move(observation));
        recordOutcome(Outcome::NoDetection, t0, 0.0);
        return;
    }
    if (!gray.isContinuous()) {
        gray = gray.clone();
    }

    image_u8_t image{
        gray.cols,
        gray.rows,
        static_cast<int32_t>(gray.step[0]),
        gray.data,
    };

    zarray_t* detections = apriltag_detector_detect(detector_, &image);
    if (!detections) {
        measurementSink().publish(std::move(observation));
        recordOutcome(Outcome::NoDetection, t0, 0.0);
        return;
    }

    const int count = zarray_size(detections);
    observation.detections.reserve(static_cast<std::size_t>(count));

    AprilTagPoseSolveInput solver_input;
    solver_input.tag_size_m = config_.tag_size_m;
    solver_input.covariance = config_.covariance;

    const auto calibration_it = config_.camera_calibrations.find(frame.camera_id);
    const auto extrinsics_it = config_.camera_to_robot.find(frame.camera_id);
    const bool has_calibration = calibration_it != config_.camera_calibrations.end();
    const bool has_extrinsics = extrinsics_it != config_.camera_to_robot.end();
    if (has_calibration) {
        solver_input.calibration = calibration_it->second;
    }
    if (has_extrinsics) {
        solver_input.camera_to_robot = extrinsics_it->second;
    }

    std::vector<int> solver_index_to_detection_index;
    solver_index_to_detection_index.reserve(static_cast<std::size_t>(count));

    for (int i = 0; i < count; ++i) {
        apriltag_detection_t* detection = nullptr;
        zarray_get(detections, i, &detection);
        if (!detection) {
            continue;
        }

        AprilTagDetection out;
        out.tag_id = detection->id;
        out.ambiguity = 0.0;
        out.reprojection_error_px = 0.0;
        for (std::size_t corner = 0; corner < out.image_corners_px.size(); ++corner) {
            out.image_corners_px[corner].x = detection->p[corner][0];
            out.image_corners_px[corner].y = detection->p[corner][1];
        }
        const std::size_t detection_index = observation.detections.size();
        observation.detections.push_back(out);

        if (!has_calibration || !has_extrinsics) {
            continue;
        }
        const auto field_it = config_.field_to_tags.find(detection->id);
        if (field_it == config_.field_to_tags.end()) {
            continue;
        }
        AprilTagPoseSolveInput::TagInput tag_input;
        tag_input.tag_id = detection->id;
        tag_input.image_corners_px = out.image_corners_px;
        tag_input.field_to_tag = field_it->second;
        tag_input.raw = detection;
        solver_input.tags.push_back(tag_input);
        solver_index_to_detection_index.push_back(static_cast<int>(detection_index));
    }

    // Outcome classification, defaults walk in this order (overridden once
    // the solve runs and produces a definitive answer):
    //   - count == 0        → NoDetection
    //   - solver_input empty after detection loop → DroppedNoCalibration
    //     (covers no calibration, no extrinsics, or no field-layout match)
    //   - solver kept pose, single-tag path     → SolvedSingle
    //   - solver kept pose, multi-tag path      → SolvedMulti
    //   - solver dropped pose with ambiguity set → DroppedByAmbiguity
    Outcome outcome = (count == 0) ? Outcome::NoDetection : Outcome::DroppedNoCalibration;
    double rms_for_stats = 0.0;

    if (!solver_input.tags.empty()) {
        const AprilTagPoseSolveOutput solver_output = solveAprilTagPose(solver_input);
        observation.field_to_robot = solver_output.field_to_robot;
        observation.covariance = solver_output.covariance;
        observation.reprojection_rms_px = solver_output.reprojection_rms_px;
        observation.solved_tag_count = solver_output.solved_tag_count;
        if (solver_output.single_tag_ambiguity.has_value() &&
            !solver_index_to_detection_index.empty()) {
            const int detection_index = solver_index_to_detection_index.front();
            observation.detections[static_cast<std::size_t>(detection_index)].ambiguity =
                *solver_output.single_tag_ambiguity;
        }
        // Scatter per-tag pose + reprojection error back onto each detection
        // using the solver→detection index map built above.
        for (std::size_t k = 0; k < solver_output.per_tag.size() &&
                                k < solver_index_to_detection_index.size(); ++k) {
            const int detection_index = solver_index_to_detection_index[k];
            auto& detection = observation.detections[
                static_cast<std::size_t>(detection_index)];
            detection.camera_to_tag = solver_output.per_tag[k].camera_to_tag;
            detection.reprojection_error_px =
                solver_output.per_tag[k].reprojection_error_px;
        }

        if (solver_output.field_to_robot.has_value()) {
            outcome = (solver_output.solved_tag_count >= 2)
                ? Outcome::SolvedMulti : Outcome::SolvedSingle;
            rms_for_stats = solver_output.reprojection_rms_px;
        } else if (solver_output.single_tag_ambiguity.has_value()) {
            outcome = Outcome::DroppedByAmbiguity;
        }
    }

    apriltag_detections_destroy(detections);
    measurementSink().publish(std::move(observation));
    recordOutcome(outcome, t0, rms_for_stats);
}

}  // namespace posest::pipelines
