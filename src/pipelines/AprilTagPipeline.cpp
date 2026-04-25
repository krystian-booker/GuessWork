#include "posest/pipelines/AprilTagPipeline.h"

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
    AprilTagObservation observation;
    observation.camera_id = frame.camera_id;
    observation.frame_sequence = frame.sequence;
    observation.capture_time = frame.capture_time;
    observation.teensy_time_us = frame.teensy_time_us;
    observation.trigger_sequence = frame.trigger_sequence;

    cv::Mat gray = toGray8(frame.image);
    if (gray.empty()) {
        measurementSink().publish(std::move(observation));
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
    }

    apriltag_detections_destroy(detections);
    measurementSink().publish(std::move(observation));
}

}  // namespace posest::pipelines
