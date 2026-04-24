#include "posest/runtime/RuntimeGraph.h"

#include <stdexcept>
#include <utility>

namespace posest::runtime {

RuntimeGraph::RuntimeGraph(
    RuntimeConfig config,
    ICameraBackendFactory& camera_factory,
    IPipelineFactory& pipeline_factory,
    IMeasurementSink& measurement_sink)
    : config_(std::move(config)),
      camera_factory_(camera_factory),
      pipeline_factory_(pipeline_factory),
      measurement_sink_(measurement_sink) {}

RuntimeGraph::~RuntimeGraph() {
    stop();
}

void RuntimeGraph::build() {
    if (built_) {
        return;
    }

    for (const auto& camera_cfg : config_.cameras) {
        if (!camera_cfg.enabled) {
            continue;
        }
        if (camera_cfg.id.empty()) {
            throw std::runtime_error("Camera config has empty id");
        }
        auto camera = camera_factory_.createCamera(camera_cfg);
        if (!camera) {
            throw std::runtime_error("Camera factory returned null for " + camera_cfg.id);
        }
        auto [it, inserted] = cameras_.emplace(camera_cfg.id, camera);
        if (!inserted) {
            throw std::runtime_error("Duplicate camera id: " + camera_cfg.id);
        }
        camera_start_order_.push_back(it->second);
    }

    for (const auto& pipeline_cfg : config_.pipelines) {
        if (!pipeline_cfg.enabled) {
            continue;
        }
        if (pipeline_cfg.id.empty()) {
            throw std::runtime_error("Pipeline config has empty id");
        }
        auto pipeline = pipeline_factory_.createPipeline(pipeline_cfg, measurement_sink_);
        if (!pipeline) {
            throw std::runtime_error("Pipeline factory returned null for " + pipeline_cfg.id);
        }
        auto [it, inserted] = pipelines_.emplace(pipeline_cfg.id, pipeline);
        if (!inserted) {
            throw std::runtime_error("Duplicate pipeline id: " + pipeline_cfg.id);
        }
        pipeline_start_order_.push_back(it->second);
    }

    for (const auto& binding : config_.bindings) {
        auto camera_it = cameras_.find(binding.camera_id);
        if (camera_it == cameras_.end()) {
            throw std::runtime_error("Binding references unknown camera: " + binding.camera_id);
        }
        auto pipeline_it = pipelines_.find(binding.pipeline_id);
        if (pipeline_it == pipelines_.end()) {
            throw std::runtime_error("Binding references unknown or disabled pipeline: " +
                                     binding.pipeline_id);
        }
        camera_it->second->addConsumer(pipeline_it->second);
    }

    built_ = true;
}

void RuntimeGraph::start() {
    if (!built_) {
        build();
    }
    if (running_) {
        return;
    }
    for (auto& pipeline : pipeline_start_order_) {
        pipeline->start();
    }
    for (auto& camera : camera_start_order_) {
        camera->start();
    }
    running_ = true;
}

void RuntimeGraph::stop() {
    if (!running_) {
        return;
    }
    for (auto it = camera_start_order_.rbegin(); it != camera_start_order_.rend(); ++it) {
        (*it)->stop();
    }
    for (auto it = pipeline_start_order_.rbegin(); it != pipeline_start_order_.rend(); ++it) {
        (*it)->stop();
    }
    running_ = false;
}

}  // namespace posest::runtime
