#include "apriltag/pose_math.hpp"

#include <cmath>

namespace gw::apriltag {

namespace {

constexpr double kSmallAngle = 1e-10;

Mat3 mat3_mul(const Mat3& a, const Mat3& b) {
    Mat3 out{};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            for (int k = 0; k < 3; ++k) out[r][c] += a[r][k] * b[k][c];
    return out;
}

Mat3 mat3_identity() {
    return {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
}

Mat3 skew(double x, double y, double z) {
    return {{{0, -z, y}, {z, 0, -x}, {-y, x, 0}}};
}

Mat3 mat3_add_scaled(const Mat3& a, const Mat3& b, double sb, const Mat3& c, double sc) {
    Mat3 out{};
    for (int r = 0; r < 3; ++r)
        for (int k = 0; k < 3; ++k) out[r][k] = a[r][k] + sb * b[r][k] + sc * c[r][k];
    return out;
}

std::array<double, 3> mat3_apply(const Mat3& m, const std::array<double, 3>& v) {
    std::array<double, 3> out{};
    for (int r = 0; r < 3; ++r)
        for (int k = 0; k < 3; ++k) out[r] += m[r][k] * v[k];
    return out;
}

}  // namespace

Mat4 mat4_identity() {
    Mat4 T{};
    for (int i = 0; i < 4; ++i) T[i][i] = 1.0;
    return T;
}

Mat4 mat4_mul(const Mat4& a, const Mat4& b) {
    Mat4 out{};
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            for (int k = 0; k < 4; ++k) out[r][c] += a[r][k] * b[k][c];
    return out;
}

Mat4 mat4_inverse_se3(const Mat4& T) {
    Mat4 out = mat4_identity();
    // Rᵀ
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) out[r][c] = T[c][r];
    // −Rᵀ t
    for (int r = 0; r < 3; ++r) {
        out[r][3] = -(out[r][0] * T[0][3] + out[r][1] * T[1][3] + out[r][2] * T[2][3]);
    }
    return out;
}

Mat4 exp_se3(const Vec6& xi) {
    const double wx = xi[0], wy = xi[1], wz = xi[2];
    const std::array<double, 3> v{xi[3], xi[4], xi[5]};
    const double theta2 = wx * wx + wy * wy + wz * wz;
    const double theta  = std::sqrt(theta2);

    const Mat3 W  = skew(wx, wy, wz);
    const Mat3 W2 = mat3_mul(W, W);

    double a, b, c;  // R = I + aW + bW², V = I + bW + cW²
    if (theta < kSmallAngle) {
        a = 1.0;
        b = 0.5;
        c = 1.0 / 6.0;
    } else {
        a = std::sin(theta) / theta;
        b = (1.0 - std::cos(theta)) / theta2;
        c = (theta - std::sin(theta)) / (theta2 * theta);
    }
    const Mat3 R = mat3_add_scaled(mat3_identity(), W, a, W2, b);
    const Mat3 V = mat3_add_scaled(mat3_identity(), W, b, W2, c);
    return mat4_from_rt(R, mat3_apply(V, v));
}

Vec6 log_se3(const Mat4& T) {
    // Rotation log.
    const double trace = T[0][0] + T[1][1] + T[2][2];
    double cos_theta   = (trace - 1.0) / 2.0;
    if (cos_theta > 1.0)  cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;
    const double theta = std::acos(cos_theta);

    std::array<double, 3> w{};
    if (theta < kSmallAngle) {
        // ω ≈ vee(R − Rᵀ)/2
        w = {(T[2][1] - T[1][2]) / 2.0, (T[0][2] - T[2][0]) / 2.0,
             (T[1][0] - T[0][1]) / 2.0};
    } else {
        const double s = theta / (2.0 * std::sin(theta));
        w = {s * (T[2][1] - T[1][2]), s * (T[0][2] - T[2][0]),
             s * (T[1][0] - T[0][1])};
    }

    // V⁻¹ t.
    const double theta2 = theta * theta;
    const Mat3   W      = skew(w[0], w[1], w[2]);
    const Mat3   W2     = mat3_mul(W, W);
    double       coeff;
    if (theta < kSmallAngle) {
        coeff = 1.0 / 12.0;
    } else {
        coeff = (1.0 - (theta * std::cos(theta / 2.0)) /
                           (2.0 * std::sin(theta / 2.0))) /
                theta2;
    }
    const Mat3 Vinv = mat3_add_scaled(mat3_identity(), W, -0.5, W2, coeff);
    const auto v    = mat3_apply(Vinv, {T[0][3], T[1][3], T[2][3]});
    return {w[0], w[1], w[2], v[0], v[1], v[2]};
}

Mat4 retract(const Mat4& T, const Vec6& xi) {
    return mat4_mul(T, exp_se3(xi));
}

Mat3 quat_wxyz_to_mat3(double w, double x, double y, double z) {
    // Normalize defensively — layout files carry finite precision.
    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    w /= n; x /= n; y /= n; z /= n;
    return {{{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
             {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
             {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}}};
}

Mat4 mat4_from_rt(const Mat3& R, const std::array<double, 3>& t) {
    Mat4 T = mat4_identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) T[r][c] = R[r][c];
        T[r][3] = t[r];
    }
    return T;
}

}  // namespace gw::apriltag
