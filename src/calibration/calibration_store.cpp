#include "calibration/calibration_store.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace gw::calib {

namespace {

// "cam12" → 12; nullopt for anything that isn't cam<digits>.
std::optional<int> cam_index(const std::string& key) {
    if (key.size() < 4 || key.compare(0, 3, "cam") != 0) return std::nullopt;
    int idx = 0;
    for (size_t i = 3; i < key.size(); ++i) {
        if (!std::isdigit(static_cast<unsigned char>(key[i]))) return std::nullopt;
        idx = idx * 10 + (key[i] - '0');
    }
    return idx;
}

[[noreturn]] void fail(const std::string& cam, const std::string& what) {
    throw CalibrationParseError("camchain " + cam + ": " + what);
}

template <size_t N>
std::array<double, N> parse_double_seq(const YAML::Node& n, const std::string& cam,
                                       const char* field) {
    if (!n || !n.IsSequence() || n.size() != N) {
        fail(cam, std::string(field) + " must be a sequence of " + std::to_string(N) +
                  " numbers");
    }
    std::array<double, N> out{};
    for (size_t i = 0; i < N; ++i) {
        try {
            out[i] = n[i].as<double>();
        } catch (const YAML::Exception&) {
            fail(cam, std::string(field) + "[" + std::to_string(i) + "] is not a number");
        }
    }
    return out;
}

Mat4 parse_mat4(const YAML::Node& n, const std::string& cam, const char* field) {
    if (!n || !n.IsSequence() || n.size() != 4) {
        fail(cam, std::string(field) + " must be a 4x4 matrix (4 rows)");
    }
    Mat4 m{};
    for (size_t r = 0; r < 4; ++r) {
        m[r] = parse_double_seq<4>(n[r], cam, field);
    }
    return m;
}

CamchainEntry parse_entry(const YAML::Node& node, const std::string& cam) {
    if (!node.IsMap()) fail(cam, "entry is not a mapping");

    CamchainEntry e;
    auto& in = e.intrinsics;

    const auto model_node = node["camera_model"];
    if (!model_node) fail(cam, "missing camera_model");
    in.camera_model = model_node.as<std::string>();

    in.intrinsics = parse_double_seq<4>(node["intrinsics"], cam, "intrinsics");

    const auto dist_model_node = node["distortion_model"];
    if (!dist_model_node) fail(cam, "missing distortion_model");
    in.distortion_model = dist_model_node.as<std::string>();

    in.distortion_coeffs =
        parse_double_seq<4>(node["distortion_coeffs"], cam, "distortion_coeffs");

    const auto res = node["resolution"];
    if (!res || !res.IsSequence() || res.size() != 2) {
        fail(cam, "resolution must be a sequence of 2 integers");
    }
    try {
        in.resolution = {res[0].as<uint32_t>(), res[1].as<uint32_t>()};
    } catch (const YAML::Exception&) {
        fail(cam, "resolution entries must be integers");
    }

    if (const auto topic = node["rostopic"]) {
        in.rostopic = topic.as<std::string>();
    }

    // camchain-imucam additions — all-or-nothing on T_cam_imu.
    if (const auto t_ci = node["T_cam_imu"]) {
        CameraImuExtrinsics imu;
        imu.T_cam_imu = parse_mat4(t_ci, cam, "T_cam_imu");
        if (const auto shift = node["timeshift_cam_imu"]) {
            try {
                imu.timeshift_cam_imu = shift.as<double>();
            } catch (const YAML::Exception&) {
                fail(cam, "timeshift_cam_imu is not a number");
            }
        }
        if (const auto t_rel = node["T_cn_cnm1"]) {
            imu.T_cn_cnm1 = parse_mat4(t_rel, cam, "T_cn_cnm1");
        }
        e.imu = std::move(imu);
    }
    return e;
}

YAML::Node emit_double_seq(const double* v, size_t n) {
    YAML::Node seq(YAML::NodeType::Sequence);
    seq.SetStyle(YAML::EmitterStyle::Flow);
    for (size_t i = 0; i < n; ++i) seq.push_back(v[i]);
    return seq;
}

YAML::Node emit_mat4(const Mat4& m) {
    YAML::Node rows(YAML::NodeType::Sequence);
    for (const auto& r : m) rows.push_back(emit_double_seq(r.data(), 4));
    return rows;
}

YAML::Node emit_entry(const CamchainEntry& e) {
    YAML::Node n(YAML::NodeType::Map);
    const auto& in = e.intrinsics;
    n["camera_model"]      = in.camera_model;
    n["intrinsics"]        = emit_double_seq(in.intrinsics.data(), 4);
    n["distortion_model"]  = in.distortion_model;
    n["distortion_coeffs"] = emit_double_seq(in.distortion_coeffs.data(), 4);
    {
        YAML::Node res(YAML::NodeType::Sequence);
        res.SetStyle(YAML::EmitterStyle::Flow);
        res.push_back(in.resolution[0]);
        res.push_back(in.resolution[1]);
        n["resolution"] = res;
    }
    if (!in.rostopic.empty()) n["rostopic"] = in.rostopic;
    if (e.imu) {
        n["T_cam_imu"]         = emit_mat4(e.imu->T_cam_imu);
        n["timeshift_cam_imu"] = e.imu->timeshift_cam_imu;
        if (e.imu->T_cn_cnm1) n["T_cn_cnm1"] = emit_mat4(*e.imu->T_cn_cnm1);
    }
    return n;
}

}  // namespace

Camchain parse_camchain(const std::string& yaml_text) {
    YAML::Node root;
    try {
        root = YAML::Load(yaml_text);
    } catch (const YAML::Exception& e) {
        throw CalibrationParseError(std::string("camchain: invalid YAML: ") + e.what());
    }
    if (!root.IsMap()) {
        throw CalibrationParseError("camchain: document is not a mapping");
    }

    std::vector<std::tuple<int, std::string, CamchainEntry>> found;
    for (const auto& kv : root) {
        const std::string key = kv.first.as<std::string>();
        const auto idx        = cam_index(key);
        if (!idx) continue;  // guesswork_meta and friends pass through silently
        found.emplace_back(*idx, key, parse_entry(kv.second, key));
    }
    if (found.empty()) {
        throw CalibrationParseError("camchain: no cam<N> entries found");
    }
    std::sort(found.begin(), found.end(),
              [](const auto& a, const auto& b) { return std::get<0>(a) < std::get<0>(b); });

    Camchain chain;
    chain.cameras.reserve(found.size());
    for (auto& [idx, key, entry] : found) {
        chain.cameras.emplace_back(std::move(key), std::move(entry));
    }
    return chain;
}

std::string serialize_camchain(const Camchain& chain) {
    YAML::Node root(YAML::NodeType::Map);
    for (const auto& [name, entry] : chain.cameras) {
        root[name] = emit_entry(entry);
    }
    std::ostringstream out;
    out << root << "\n";
    return out.str();
}

std::string serialize_single_camera(const CamchainEntry& entry) {
    YAML::Node root(YAML::NodeType::Map);
    root["cam0"] = emit_entry(entry);
    std::ostringstream out;
    out << root << "\n";
    return out.str();
}

std::optional<GuessworkMeta> parse_guesswork_meta(const std::string& yaml_text) {
    YAML::Node root;
    try {
        root = YAML::Load(yaml_text);
    } catch (const YAML::Exception& e) {
        throw CalibrationParseError(std::string("guesswork_meta: invalid YAML: ") +
                                    e.what());
    }
    if (!root.IsMap()) return std::nullopt;
    const auto meta = root["guesswork_meta"];
    if (!meta || !meta.IsMap()) return std::nullopt;

    GuessworkMeta out;
    const auto get_d = [&](const char* key) -> std::optional<double> {
        const auto n = meta[key];
        if (!n) return std::nullopt;
        try {
            return n.as<double>();
        } catch (const YAML::Exception&) {
            return std::nullopt;  // tolerate malformed individual fields
        }
    };
    if (const auto n = meta["session_id"]) {
        try { out.session_id = n.as<std::string>(); } catch (const YAML::Exception&) {}
    }
    out.reprojection_error_px        = get_d("reprojection_error_px");
    out.reprojection_error_mean_px   = get_d("reprojection_error_mean_px");
    out.reprojection_error_median_px = get_d("reprojection_error_median_px");
    out.reprojection_error_std_px    = get_d("reprojection_error_std_px");
    out.timeshift_cam_imu_s          = get_d("timeshift_cam_imu_s");
    if (const auto n = meta["source_cam_index"]) {
        try { out.source_cam_index = n.as<int>(); } catch (const YAML::Exception&) {}
    }
    return out;
}

Mat4 parse_t_robot_imu(const std::string& json_text) {
    YAML::Node root;
    try {
        root = YAML::Load(json_text);
    } catch (const YAML::Exception& e) {
        throw CalibrationParseError(std::string("t_robot_imu: invalid JSON: ") + e.what());
    }
    if (!root.IsMap() || !root["T_robot_imu"]) {
        throw CalibrationParseError("t_robot_imu: missing top-level key T_robot_imu");
    }
    const Mat4 T = parse_mat4(root["T_robot_imu"], "t_robot_imu", "T_robot_imu");

    // Bottom row must be (0,0,0,1).
    const auto& b = T[3];
    if (std::abs(b[0]) > 1e-9 || std::abs(b[1]) > 1e-9 || std::abs(b[2]) > 1e-9 ||
        std::abs(b[3] - 1.0) > 1e-9) {
        throw CalibrationParseError("t_robot_imu: bottom row must be (0,0,0,1)");
    }

    // Rotation block: orthonormal within 1e-3, det ≈ +1 (a reflection or a
    // scaled matrix here silently corrupts every downstream pose).
    constexpr double kTol = 1e-3;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double dot = 0;
            for (int k = 0; k < 3; ++k) dot += T[k][i] * T[k][j];
            const double expect = (i == j) ? 1.0 : 0.0;
            if (std::abs(dot - expect) > kTol) {
                throw CalibrationParseError(
                    "t_robot_imu: rotation block is not orthonormal");
            }
        }
    }
    const double det =
        T[0][0] * (T[1][1] * T[2][2] - T[1][2] * T[2][1]) -
        T[0][1] * (T[1][0] * T[2][2] - T[1][2] * T[2][0]) +
        T[0][2] * (T[1][0] * T[2][1] - T[1][1] * T[2][0]);
    if (std::abs(det - 1.0) > kTol) {
        throw CalibrationParseError("t_robot_imu: rotation determinant is not +1");
    }
    return T;
}

std::string serialize_t_robot_imu(const Mat4& T) {
    std::ostringstream out;
    out << std::setprecision(17);
    out << "{\"T_robot_imu\": [";
    for (int r = 0; r < 4; ++r) {
        out << (r ? ", [" : "[");
        for (int c = 0; c < 4; ++c) out << (c ? ", " : "") << T[r][c];
        out << "]";
    }
    out << "]}";
    return out.str();
}

}  // namespace gw::calib
