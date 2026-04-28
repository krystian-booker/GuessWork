#include "posest/calibration/CalibrationTargetWriter.h"

#include <fstream>
#include <stdexcept>

namespace posest::calibration {

void writeKalibrTargetYaml(
    const runtime::CalibrationTargetConfig& target,
    const std::filesystem::path& path) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error(
            "failed to open Kalibr target YAML for writing: " + path.string());
    }

    if (target.type == "aprilgrid") {
        out << "target_type: 'aprilgrid'\n"
            << "tagCols: " << target.cols << "\n"
            << "tagRows: " << target.rows << "\n"
            << "tagSize: " << target.tag_size_m << "\n"
            << "tagSpacing: " << target.tag_spacing_ratio << "\n"
            << "tagFamily: '" << target.tag_family << "'\n";
    } else if (target.type == "checkerboard") {
        out << "target_type: 'checkerboard'\n"
            << "targetCols: " << target.cols << "\n"
            << "targetRows: " << target.rows << "\n"
            << "rowSpacingMeters: " << target.square_size_m << "\n"
            << "colSpacingMeters: " << target.square_size_m << "\n";
    } else if (target.type == "circlegrid") {
        out << "target_type: 'circlegrid'\n"
            << "targetCols: " << target.cols << "\n"
            << "targetRows: " << target.rows << "\n"
            << "spacingMeters: " << target.square_size_m << "\n"
            << "asymmetricGrid: false\n";
    } else {
        throw std::invalid_argument(
            "unsupported calibration target type: " + target.type);
    }
}

}  // namespace posest::calibration
