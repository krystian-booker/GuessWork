#include "server/apriltag_supervisor.hpp"

#include <iostream>
#include <utility>

#include "apriltag/field_layout.hpp"
#include "calibration/calibration_store.hpp"
#include "server/camera_repository.hpp"
#include "server/field_layout_repository.hpp"
#include "server/imu_config_repository.hpp"

namespace gw::server {

namespace {

// Builds the consumer's camera model from a parsed camchain entry.
gw::apriltag::PinholeCamera to_pinhole(const gw::calib::CameraIntrinsics& in) {
    gw::apriltag::PinholeCamera cam;
    cam.fxfycxcy = in.intrinsics;
    cam.model    = (in.distortion_model == "equidistant")
                       ? gw::apriltag::PinholeCamera::Dist::kEquidistant
                       : gw::apriltag::PinholeCamera::Dist::kRadTan;
    cam.d      = in.distortion_coeffs;
    cam.width  = in.resolution[0];
    cam.height = in.resolution[1];
    return cam;
}

}  // namespace

ApriltagSupervisor::ApriltagSupervisor(CameraRepository&      cameras,
                                       FieldLayoutRepository& field_layouts,
                                       ImuConfigRepository&   imu_config)
    : cameras_(cameras),
      field_layouts_(field_layouts),
      imu_config_(imu_config),
      bus_(std::make_shared<gw::apriltag::TagPoseBus>()) {
    std::lock_guard lk(mu_);
    shared_ = build_shared_locked();
}

std::shared_ptr<const gw::apriltag::SharedTagConfig>
ApriltagSupervisor::build_shared_locked() {
    auto shared = std::make_shared<gw::apriltag::SharedTagConfig>();

    active_layout_id_.reset();
    active_layout_name_.reset();
    try {
        if (const auto row = field_layouts_.get_active()) {
            shared->layout = gw::apriltag::prepare_layout(
                gw::apriltag::parse_field_layout_json(row->json));
            active_layout_id_   = row->id;
            active_layout_name_ = row->name;
        }
    } catch (const std::exception& e) {
        std::cerr << "ApriltagSupervisor: active field layout unusable: "
                  << e.what() << "\n";
    }

    try {
        const auto cfg = imu_config_.get();
        if (cfg.t_imu_robot_json) {
            shared->T_robot_imu =
                gw::calib::parse_t_robot_imu(*cfg.t_imu_robot_json);
        }
    } catch (const std::exception& e) {
        std::cerr << "ApriltagSupervisor: t_imu_robot unusable: " << e.what()
                  << "\n";
    }

    return shared;
}

std::shared_ptr<gw::IConsumer> ApriltagSupervisor::make_consumer(const Camera& row) {
    std::lock_guard lk(mu_);

    if (!row.role || *row.role != "apriltag") {
        consumers_.erase(row.id);
        factory_reasons_.erase(row.id);
        return nullptr;
    }

    // Prefer the camera-IMU extrinsics result: Kalibr refined the intrinsics
    // there, and it carries T_cam_imu for the field-frame chain.
    std::optional<gw::calib::CamchainEntry> entry;
    const std::string* source =
        row.imu_extrinsics_json ? &*row.imu_extrinsics_json
        : row.calibration_json  ? &*row.calibration_json
                                : nullptr;
    if (!source) {
        factory_reasons_[row.id] = "uncalibrated";
        consumers_.erase(row.id);
        return nullptr;
    }
    try {
        auto chain = gw::calib::parse_camchain(*source);
        entry      = chain.cameras.front().second;
    } catch (const std::exception& e) {
        factory_reasons_[row.id] =
            std::string("calibration unparseable: ") + e.what();
        consumers_.erase(row.id);
        return nullptr;
    }

    std::optional<gw::apriltag::Mat4> T_cam_imu;
    if (entry->imu) T_cam_imu = entry->imu->T_cam_imu;

    auto consumer = std::make_shared<gw::apriltag::AprilTagConsumer>(
        row.id, row.name, to_pinhole(entry->intrinsics), T_cam_imu, bus_,
        shared_);
    consumers_[row.id] = consumer;
    factory_reasons_.erase(row.id);
    return consumer;
}

void ApriltagSupervisor::reload_shared() {
    std::lock_guard lk(mu_);
    shared_ = build_shared_locked();
    for (auto it = consumers_.begin(); it != consumers_.end();) {
        if (auto c = it->second.lock()) {
            c->update_shared(shared_);
            ++it;
        } else {
            it = consumers_.erase(it);
        }
    }
}

ApriltagStatus ApriltagSupervisor::status() {
    ApriltagStatus out;

    // Snapshot registry state under our lock; repository reads happen
    // without it (Database has its own mutex).
    std::map<int64_t, std::shared_ptr<gw::apriltag::AprilTagConsumer>> live;
    std::map<int64_t, std::string> reasons;
    {
        std::lock_guard lk(mu_);
        out.active_layout_id   = active_layout_id_;
        out.active_layout_name = active_layout_name_;
        out.t_robot_imu_set    = shared_ && shared_->T_robot_imu.has_value();
        for (auto it = consumers_.begin(); it != consumers_.end();) {
            if (auto c = it->second.lock()) {
                live[it->first] = std::move(c);
                ++it;
            } else {
                it = consumers_.erase(it);
            }
        }
        reasons = factory_reasons_;
    }

    for (const auto& row : cameras_.list_all()) {
        if (!row.role || *row.role != "apriltag") continue;
        ApriltagStatus::CameraEntry e;
        e.camera_id = row.id;
        e.name      = row.name;
        if (const auto it = live.find(row.id); it != live.end()) {
            e.running = true;
            e.stats   = it->second->snapshot();
            e.reason  = e.stats.reason;
        } else if (const auto rit = reasons.find(row.id); rit != reasons.end()) {
            e.reason = rit->second;
        } else {
            // Role set but no consumer and no recorded decline — the slot
            // never started (camera offline). The route layer confirms via
            // CameraSupervisor::snapshot_all.
            e.reason = "offline";
        }
        out.cameras.push_back(std::move(e));
    }
    return out;
}

}  // namespace gw::server
