#pragma once

#include <array>

#include "calibration/calibration_store.hpp"  // gw::calib::Mat4

// Small SE(3) helpers on gw::calib::Mat4 (4×4 row-major homogeneous
// transforms). No OpenCV, no Eigen — used by the tag-pose estimator, the
// NEES covariance tests, and (later) the fusion layer.
//
// Tangent vectors are 6-dim [ωx ωy ωz, tx ty tz] — rotation first, then
// translation — matching GTSAM's Pose3 convention so Phase 6 consumes our
// covariances unchanged. Perturbations are RIGHT (body-frame):
// retract(T, ξ) = T · exp_se3(ξ).

namespace gw::apriltag {

using Mat4 = gw::calib::Mat4;
using Vec6 = std::array<double, 6>;
using Mat3 = std::array<std::array<double, 3>, 3>;

Mat4 mat4_identity();
Mat4 mat4_mul(const Mat4& a, const Mat4& b);

// Inverse of a rigid transform (R | t): (Rᵀ | −Rᵀt). Assumes the input is a
// valid SE(3) member — no general 4×4 inversion.
Mat4 mat4_inverse_se3(const Mat4& T);

// Exponential map se(3) → SE(3), ξ = [ω, v].
Mat4 exp_se3(const Vec6& xi);

// Logarithm map SE(3) → se(3). Valid for rotation angles < π.
Vec6 log_se3(const Mat4& T);

// retract(T, ξ) = T · exp_se3(ξ) — right/body perturbation.
Mat4 retract(const Mat4& T, const Vec6& xi);

// Rotation matrix from a WPILib-style quaternion (W, X, Y, Z order).
Mat3 quat_wxyz_to_mat3(double w, double x, double y, double z);

// Builds a Mat4 from rotation + translation.
Mat4 mat4_from_rt(const Mat3& R, const std::array<double, 3>& t);

// Row-major 6×6 (tangent-space operator / covariance), same [ω, v] ordering
// as Vec6.
using Mat6 = std::array<double, 36>;

// SE(3) Adjoint of T in the [ω, v] tangent ordering:
//   Ad_T = [[ R,      0 ],
//           [ [t]×R,  R ]]
// satisfying log(T · Exp(ξ) · T⁻¹) = Ad_T · ξ. Rotates body-tangent deltas
// and covariances between frames: Σ' = Ad_T · Σ · Ad_Tᵀ.
Mat6 adjoint_se3(const Mat4& T);

Mat6 mat6_mul(const Mat6& a, const Mat6& b);

// Covariance congruence A · S · Aᵀ, result explicitly symmetrized.
Mat6 congruence(const Mat6& A, const Mat6& S);

}  // namespace gw::apriltag
