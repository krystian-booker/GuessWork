#pragma once

#include <string>

namespace posest::runtime {

struct PipelineConfig {
    std::string id;
    std::string type;
    bool enabled{true};
    std::string parameters_json;
};

struct CameraPipelineBinding {
    std::string camera_id;
    std::string pipeline_id;
};

}  // namespace posest::runtime
