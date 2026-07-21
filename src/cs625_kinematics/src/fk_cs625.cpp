#include "cs625_kinematics/fk_cs625.hpp"

#include <algorithm>
#include <cmath>

namespace cs625_kinematics {

// -----------------------------------------------------------------------------
// Internal: Modified DH transform
// A_i = Rotx(alpha) * Transx(a) * Rotz(theta) * Transz(d)
// -----------------------------------------------------------------------------
static Eigen::Matrix4d mdh_transform(double a, double alpha, double d, double theta)
{
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);
    const double ct = std::cos(theta);
    const double st = std::sin(theta);

    Eigen::Matrix4d A;
    A << ct,      -st,       0.0,   a,
         st*ca,   ct*ca,    -sa,   -d*sa,
         st*sa,   ct*sa,     ca,    d*ca,
         0.0,     0.0,       0.0,   1.0;
    return A;
}

Eigen::Matrix4d fk_cs625_flange(const std::array<double, 6>& q)
{
    // MDH parameters (meters), copied from fk_cs625_opw_v6.m
    const double a[6] = {
         0.0000000,
         0.0000000,
        -0.7505170,
        -0.6230980,
         0.0000000,
         0.0000000
    };

    const double d[6] = {
        0.2310070,
        0.0000000,
        0.0000000,
        0.1711440,
        0.1279060,
        0.1274780
    };

    const double alpha[6] = {
         0.0000000,
         1.5707999,
         0.0000000,
         0.0000000,
         1.5707999,
        -1.5707999
    };

    const double theta_offset[6] = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    };

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 6; ++i) {
        const double theta = q[i] + theta_offset[i];
        const Eigen::Matrix4d A = mdh_transform(a[i], alpha[i], d[i], theta);
        T = T * A;
    }

    return T;
}

Eigen::Matrix4d flange_to_my_end_effector()
{
    // Confirmed by user:
    // my_end_effector_link == tool offset
    // translation unit: meters
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = -0.046;
    T(1, 3) =  0.0;
    T(2, 3) =  0.353;
    return T;
}

Eigen::Matrix4d fk_cs625_ee(const std::array<double, 6>& q)
{
    return fk_cs625_flange(q) * flange_to_my_end_effector();
}

Eigen::Matrix4d fk_cs625(const std::array<double, 6>& q, bool include_tool)
{
    return include_tool ? fk_cs625_ee(q) : fk_cs625_flange(q);
}

double positionError(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b)
{
    return (a.block<3,1>(0,3) - b.block<3,1>(0,3)).norm();
}

double orientationError(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b)
{
    const Eigen::Matrix3d Ra = a.block<3,3>(0,0);
    const Eigen::Matrix3d Rb = b.block<3,3>(0,0);
    const Eigen::Matrix3d Rerr = Ra.transpose() * Rb;

    double c = 0.5 * (Rerr.trace() - 1.0);
    c = std::clamp(c, -1.0, 1.0);
    return std::acos(c);
}

}  // namespace cs625_kinematics
