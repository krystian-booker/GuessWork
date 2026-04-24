#include "posest/pipelines/AprilTagPipeline.h"

#include <stdexcept>
#include <utility>

#include <apriltag/apriltag.h>
#include <apriltag/common/image_types.h>
#include <apriltag/common/zarray.h>
#include <apriltag/tag36h11.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>

namespace posest::pipelines {

namespace {

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

    if (config.family != "tag36h11") {
        throw std::invalid_argument("Only AprilTag family tag36h11 is supported");
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

    return config;
}

AprilTagPipeline::AprilTagPipeline(
    std::string id,
    IMeasurementSink& sink,
    AprilTagPipelineConfig config)
    : VisionPipelineBase(std::move(id), "apriltag", sink), config_(std::move(config)) {
    family_ = tag36h11_create();
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
        tag36h11_destroy(family_);
        family_ = nullptr;
    }
}

void AprilTagPipeline::processFrame(const Frame& frame) {
    AprilTagObservation observation;
    observation.camera_id = frame.camera_id;
    observation.frame_sequence = frame.sequence;
    observation.capture_time = frame.capture_time;

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
    for (int i = 0; i < count; ++i) {
        apriltag_detection_t* detection = nullptr;
        zarray_get(detections, i, &detection);
        if (!detection) {
            continue;
        }

        AprilTagDetection out;
        out.tag_id = detection->id;
        out.ambiguity = static_cast<double>(detection->hamming);
        out.reprojection_error_px = 0.0;
        for (std::size_t corner = 0; corner < out.image_corners_px.size(); ++corner) {
            out.image_corners_px[corner].x = detection->p[corner][0];
            out.image_corners_px[corner].y = detection->p[corner][1];
        }
        observation.detections.push_back(out);
    }

    apriltag_detections_destroy(detections);
    measurementSink().publish(std::move(observation));
}

}  // namespace posest::pipelines
