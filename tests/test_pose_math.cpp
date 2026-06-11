#include <gtest/gtest.h>

#include <cmath>

#include "apriltag/pose_math.hpp"

namespace gw::apriltag {

namespace {

void expect_mat4_near(const Mat4& a, const Mat4& b, double tol = 1e-9) {
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) EXPECT_NEAR(a[r][c], b[r][c], tol)
            << "at (" << r << "," << c << ")";
}

}  // namespace

TEST(PoseMathTest, IdentityRoundTrips) {
    const Mat4 I = mat4_identity();
    expect_mat4_near(mat4_mul(I, I), I);
    expect_mat4_near(mat4_inverse_se3(I), I);
    const Vec6 xi = log_se3(I);
    for (double v : xi) EXPECT_NEAR(v, 0.0, 1e-12);
}

TEST(PoseMathTest, ExpLogRoundTrip) {
    const Vec6 xi{0.3, -0.2, 0.5, 1.0, -2.0, 0.7};
    const auto back = log_se3(exp_se3(xi));
    for (int i = 0; i < 6; ++i) EXPECT_NEAR(back[i], xi[i], 1e-9);
}

TEST(PoseMathTest, ExpLogSmallAngle) {
    const Vec6 xi{1e-12, -2e-12, 3e-12, 0.1, 0.2, 0.3};
    const auto back = log_se3(exp_se3(xi));
    for (int i = 0; i < 6; ++i) EXPECT_NEAR(back[i], xi[i], 1e-9);
}

TEST(PoseMathTest, KnownRotation90DegZ) {
    // ω = (0,0,π/2): rotation by 90° about Z. exp([0,0,π/2,0,0,0]) maps
    // x̂ → ŷ.
    const Mat4 T = exp_se3({0, 0, M_PI / 2, 0, 0, 0});
    EXPECT_NEAR(T[0][0], 0.0, 1e-12);
    EXPECT_NEAR(T[1][0], 1.0, 1e-12);
    EXPECT_NEAR(T[0][1], -1.0, 1e-12);
    EXPECT_NEAR(T[2][2], 1.0, 1e-12);
}

TEST(PoseMathTest, InverseComposesToIdentity) {
    const Mat4 T = exp_se3({0.4, 0.1, -0.6, 2.0, 0.5, -1.5});
    expect_mat4_near(mat4_mul(T, mat4_inverse_se3(T)), mat4_identity(), 1e-12);
    expect_mat4_near(mat4_mul(mat4_inverse_se3(T), T), mat4_identity(), 1e-12);
}

TEST(PoseMathTest, RetractIsRightPerturbation) {
    const Mat4 T  = exp_se3({0.2, 0.3, -0.1, 1.0, 0.0, 0.5});
    const Vec6 xi{0.01, -0.02, 0.005, 0.1, -0.05, 0.02};
    expect_mat4_near(retract(T, xi), mat4_mul(T, exp_se3(xi)), 1e-12);
    // log(inverse(T) · retract(T, ξ)) == ξ — the relationship the NEES test
    // and Phase 6's GTSAM retract rely on.
    const auto back = log_se3(mat4_mul(mat4_inverse_se3(T), retract(T, xi)));
    for (int i = 0; i < 6; ++i) EXPECT_NEAR(back[i], xi[i], 1e-9);
}

TEST(PoseMathTest, QuaternionGoldens) {
    // Identity quaternion.
    const Mat3 I = quat_wxyz_to_mat3(1, 0, 0, 0);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) EXPECT_NEAR(I[r][c], r == c ? 1.0 : 0.0, 1e-12);

    // 180° about Z: (w=0, z=1) — flips X and Y.
    const Mat3 R = quat_wxyz_to_mat3(0, 0, 0, 1);
    EXPECT_NEAR(R[0][0], -1.0, 1e-12);
    EXPECT_NEAR(R[1][1], -1.0, 1e-12);
    EXPECT_NEAR(R[2][2], 1.0, 1e-12);

    // 90° about Z: (w=cos45, z=sin45) — maps x̂ → ŷ.
    const double s = std::sqrt(0.5);
    const Mat3 Q = quat_wxyz_to_mat3(s, 0, 0, s);
    EXPECT_NEAR(Q[1][0], 1.0, 1e-12);
    EXPECT_NEAR(Q[0][1], -1.0, 1e-12);

    // Unnormalized input is normalized defensively.
    const Mat3 N = quat_wxyz_to_mat3(2, 0, 0, 0);
    EXPECT_NEAR(N[0][0], 1.0, 1e-12);
}

TEST(PoseMathTest, AdjointMatchesConjugation) {
    // The defining identity: log(T · Exp(ξ) · T⁻¹) = Ad_T · ξ (exact for
    // small ξ to first order; use a small perturbation + tight tolerance).
    const Mat4 T = exp_se3({0.4, -0.3, 0.7, 1.2, -0.8, 2.0});
    const Vec6 xi{1e-4, -2e-4, 1.5e-4, 3e-4, -1e-4, 2e-4};

    const Mat4 conj = mat4_mul(mat4_mul(T, exp_se3(xi)), mat4_inverse_se3(T));
    const Vec6 lhs  = log_se3(conj);

    const Mat6 ad = adjoint_se3(T);
    Vec6 rhs{};
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) rhs[r] += ad[r * 6 + c] * xi[c];

    for (int i = 0; i < 6; ++i) EXPECT_NEAR(lhs[i], rhs[i], 1e-9);
}

TEST(PoseMathTest, AdjointOfIdentityIsIdentity) {
    const Mat6 ad = adjoint_se3(mat4_identity());
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            EXPECT_NEAR(ad[r * 6 + c], r == c ? 1.0 : 0.0, 1e-12);
}

TEST(PoseMathTest, AdjointOfInverseIsInverse) {
    const Mat4 T = exp_se3({-0.2, 0.5, 0.1, 0.7, 1.1, -0.4});
    const Mat6 prod = mat6_mul(adjoint_se3(T), adjoint_se3(mat4_inverse_se3(T)));
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            EXPECT_NEAR(prod[r * 6 + c], r == c ? 1.0 : 0.0, 1e-9);
}

TEST(PoseMathTest, CongruencePreservesSymmetryAndPsd) {
    // Σ = Lᵀ·L + εI is symmetric PSD by construction.
    Mat6 S{};
    const double L[6][6] = {{1, 0, 0, 0, 0, 0},    {0.5, 2, 0, 0, 0, 0},
                            {0.1, -0.3, 1.5, 0, 0, 0}, {0, 0.2, 0, 1, 0, 0},
                            {-0.4, 0, 0.6, 0.1, 2.5, 0}, {0, 0, 0, -0.2, 0.3, 0.8}};
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            for (int k = 0; k < 6; ++k) S[r * 6 + c] += L[r][k] * L[c][k];

    const Mat4 T   = exp_se3({0.3, 0.2, -0.5, 0.4, -1.0, 0.6});
    const Mat6 out = congruence(adjoint_se3(T), S);

    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) EXPECT_NEAR(out[r * 6 + c], out[c * 6 + r], 1e-12);
        EXPECT_GE(out[r * 6 + r], 0.0);
    }
    // Congruence by an invertible map preserves PSD; spot-check with a few
    // quadratic forms xᵀΣ'x ≥ 0.
    const double probes[3][6] = {
        {1, -1, 0.5, 2, 0, -0.3}, {0, 1, 0, -1, 1, 0}, {0.2, 0.2, 0.2, 1, 1, 1}};
    for (const auto& x : probes) {
        double q = 0;
        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 6; ++c) q += x[r] * out[r * 6 + c] * x[c];
        EXPECT_GE(q, -1e-12);
    }
}

}  // namespace gw::apriltag
