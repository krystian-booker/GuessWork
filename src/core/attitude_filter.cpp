#include "core/attitude_filter.hpp"

#include <cmath>

namespace gw {

namespace {

using Quat = std::array<double, 4>;  // w, x, y, z

Quat quat_mul(const Quat& a, const Quat& b) {
    return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

void quat_normalize(Quat& q) {
    const double n =
        std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n <= 0) {
        q = {1, 0, 0, 0};
        return;
    }
    for (auto& v : q) v /= n;
    if (q[0] < 0) {  // canonical hemisphere, keeps UI slerp short-path
        for (auto& v : q) v = -v;
    }
}

// exp of a rotation vector (axis·angle, rad) as a quaternion.
Quat quat_exp(double rx, double ry, double rz) {
    const double angle = std::sqrt(rx * rx + ry * ry + rz * rz);
    if (angle < 1e-12) return {1, rx / 2, ry / 2, rz / 2};
    const double s = std::sin(angle / 2) / angle;
    return {std::cos(angle / 2), rx * s, ry * s, rz * s};
}

// Rotate body-frame vector v into the world frame by q.
std::array<double, 3> rotate(const Quat& q, const std::array<double, 3>& v) {
    // v' = q ⊗ (0,v) ⊗ q*
    const Quat p{0, v[0], v[1], v[2]};
    const Quat qc{q[0], -q[1], -q[2], -q[3]};
    const Quat r = quat_mul(quat_mul(q, p), qc);
    return {r[1], r[2], r[3]};
}

}  // namespace

void AttitudeFilter::feed(const ImuSample& s) {
    ++samples_;
    const double ax = s.accel[0], ay = s.accel[1], az = s.accel[2];
    const double amag = std::sqrt(ax * ax + ay * ay + az * az);

    if (!initialized_) {
        // Bootstrap roll/pitch from gravity (yaw = 0 by construction):
        // find the rotation taking body-accel to world +Z.
        if (amag < kAccelGateLo || amag > kAccelGateHi) return;
        const std::array<double, 3> a{ax / amag, ay / amag, az / amag};
        // axis = a × z, angle = acos(a·z)
        const double cx = a[1], cy = -a[0];  // a × (0,0,1)
        const double cn = std::sqrt(cx * cx + cy * cy);
        const double angle = std::atan2(cn, a[2]);
        if (cn < 1e-9) {
            q_ = (a[2] >= 0) ? Quat{1, 0, 0, 0} : Quat{0, 1, 0, 0};
        } else {
            q_ = quat_exp(angle * cx / cn, angle * cy / cn, 0.0);
        }
        quat_normalize(q_);
        initialized_ = true;
        last_t_ns_   = s.t_ns;
        return;
    }

    double dt = 0.0025;  // fall back to the nominal 400 Hz step
    if (s.t_ns > last_t_ns_) {
        dt = static_cast<double>(s.t_ns - last_t_ns_) * 1e-9;
        if (dt > kMaxStepSec) dt = kMaxStepSec;
    }
    last_t_ns_ = s.t_ns;

    // Gyro integration (body rates, right-multiply).
    q_ = quat_mul(q_, quat_exp(s.gyro[0] * dt, s.gyro[1] * dt, s.gyro[2] * dt));

    // Accel tilt correction: pull the world-frame accel direction toward
    // world +Z with a first-order blend, only when near 1 g.
    if (amag > kAccelGateLo && amag < kAccelGateHi) {
        const auto aw = rotate(q_, {ax / amag, ay / amag, az / amag});
        // error rotation vector (world frame) = â_world × ẑ
        const double ex = aw[1], ey = -aw[0];
        const double alpha = kAccelGain * dt;
        q_ = quat_mul(quat_exp(ex * alpha, ey * alpha, 0.0), q_);
    }
    quat_normalize(q_);
}

void AttitudeFilter::zero_yaw() {
    if (!initialized_) return;
    // Current yaw (ZYX convention) — undo it with a world-Z rotation.
    const double yaw = std::atan2(
        2 * (q_[0] * q_[3] + q_[1] * q_[2]),
        1 - 2 * (q_[2] * q_[2] + q_[3] * q_[3]));
    q_ = quat_mul(quat_exp(0, 0, -yaw), q_);
    quat_normalize(q_);
}

void AttitudeFilter::reset() {
    q_           = {1, 0, 0, 0};
    initialized_ = false;
    last_t_ns_   = 0;
    samples_     = 0;
}

AttitudeFilter::Snapshot AttitudeFilter::snapshot() const {
    Snapshot s;
    s.initialized = initialized_;
    s.q           = q_;
    s.last_t_ns   = last_t_ns_;
    s.samples     = samples_;
    // ZYX euler for the readout.
    const auto& q = q_;
    s.roll_deg = std::atan2(2 * (q[0] * q[1] + q[2] * q[3]),
                            1 - 2 * (q[1] * q[1] + q[2] * q[2])) *
                 180.0 / M_PI;
    double sp = 2 * (q[0] * q[2] - q[3] * q[1]);
    if (sp > 1) sp = 1;
    if (sp < -1) sp = -1;
    s.pitch_deg = std::asin(sp) * 180.0 / M_PI;
    s.yaw_deg   = std::atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                             1 - 2 * (q[2] * q[2] + q[3] * q[3])) *
                180.0 / M_PI;
    return s;
}

}  // namespace gw
