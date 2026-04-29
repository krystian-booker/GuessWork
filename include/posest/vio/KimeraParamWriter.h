#pragma once

#include <filesystem>

#include "posest/runtime/RuntimeConfig.h"

namespace posest::vio {

// Emit the seven Kimera-VIO YAMLs into `param_dir` so a downstream
// VIO::VioParams(param_dir) constructor can read them. Files written:
//   - PipelineParams.yaml          (compile-time-embedded copy; selects
//                                    MonoImu frontend + plain VioBackend)
//   - FrontendParams.yaml          (compile-time-embedded copy)
//   - BackendParams.yaml           (compile-time-embedded copy)
//   - LcdParams.yaml               (compile-time-embedded copy; required
//                                    by parseYAML even when LCD is off)
//   - DisplayParams.yaml           (compile-time-embedded copy; ditto)
//   - ImuParams.yaml               (template + IMUtoBodyT_BS appended
//                                    from FusionConfig::imu_extrinsic_body_to_imu)
//   - LeftCameraParams.yaml        (intrinsics, distortion, T_BS) built
//                                    from the active CameraCalibrationConfig
//                                    and CameraImuCalibrationConfig matching
//                                    runtime_config.vio.vio_camera_id.
//
// No-op when runtime_config.vio.enabled is false. Throws
// std::runtime_error when:
//   - vio.vio_camera_id has no active CameraCalibrationConfig row,
//   - vio.vio_camera_id has no active CameraImuCalibrationConfig row,
//   - filesystem creation / write fails,
//   - a distortion model from Kalibr is not in the supported map.
//
// All four files are written atomically via write-to-tmp + rename so a
// partial write never leaves Kimera reading half a file.
void emitKimeraParamYamls(
    const runtime::RuntimeConfig& runtime_config,
    const std::filesystem::path& param_dir);

}  // namespace posest::vio
