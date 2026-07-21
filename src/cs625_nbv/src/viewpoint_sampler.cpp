#include "cs625_nbv/viewpoint_sampler.hpp"
#include <cmath>

namespace cs625_nbv {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kGoldenRatio = 1.61803398874989484820;  // (1+√5)/2
}  // namespace

ViewpointSampler::ViewpointSampler(const CameraModel& camera)
    : camera_(camera) {}

std::vector<geometry_msgs::msg::PoseStamped> ViewpointSampler::generate_candidates(
    const Eigen::Vector3d& target_center,
    double view_distance,
    int sample_count,
    const Eigen::Vector3d& hemisphere_axis) const
{
    std::vector<geometry_msgs::msg::PoseStamped> candidates;
    candidates.reserve(sample_count);

    // Fibonacci sphere sampling on a hemisphere
    for (int i = 0; i < sample_count; ++i) {
        // Map i to [0, 1) for hemisphere
        double y = 1.0 - static_cast<double>(i) / (sample_count - 1);  // cos(theta) from 1 to 0
        double theta = 2.0 * kPi * static_cast<double>(i) / kGoldenRatio;

        double sin_theta = std::sin(theta);
        double cos_theta = std::cos(theta);
        double sin_phi = std::sqrt(1.0 - y * y);  // sqrt(1 - cos^2)

        // Point on unit hemisphere (Y-up for standard robot coords)
        double nx = sin_phi * cos_theta;
        double ny = y;                     // cos(theta) maps to Y
        double nz = sin_phi * sin_theta;

        // Camera position: move view_distance away from target along direction
        Eigen::Vector3d camera_pos = target_center + Eigen::Vector3d(nx, ny, nz) * view_distance;

        // Compute camera orientation looking at target
        Eigen::Matrix3d R = look_at(camera_pos, target_center);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "base_link";
        pose.pose.position.x = camera_pos.x();
        pose.pose.position.y = camera_pos.y();
        pose.pose.position.z = camera_pos.z();

        // Convert rotation matrix to quaternion
        Eigen::Quaterniond q(R);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        // Filter: only keep if target is within FOV
        Eigen::Isometry3d cam_pose = Eigen::Isometry3d::Identity();
        cam_pose.translation() = camera_pos;
        cam_pose.linear() = R;

        if (is_target_in_fov(target_center, cam_pose)) {
            candidates.push_back(pose);
        }
    }

    return candidates;
}

Eigen::Matrix3d ViewpointSampler::look_at(
    const Eigen::Vector3d& camera_pos,
    const Eigen::Vector3d& target_pos,
    const Eigen::Vector3d& up) const
{
    // Camera Z-axis points from camera to target (forward direction for eye-in-hand)
    Eigen::Vector3d z_axis = (target_pos - camera_pos).normalized();

    // X-axis = up × Z (perpendicular to both)
    Eigen::Vector3d x_axis = up.cross(z_axis).normalized();
    if (x_axis.norm() < 1e-6) {
        // Handle degenerate case: up is parallel to z_axis
        x_axis = Eigen::Vector3d::UnitX().cross(z_axis).normalized();
    }

    // Y-axis = Z × X (completes the right-handed frame)
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);

    // Build rotation matrix: columns are X, Y, Z axes
    Eigen::Matrix3d R;
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = z_axis;
    return R;
}

bool ViewpointSampler::is_target_in_fov(
    const Eigen::Vector3d& target_center,
    const Eigen::Isometry3d& camera_pose) const
{
    // Transform target to camera frame
    Eigen::Vector3d target_in_cam = camera_pose.inverse() * target_center;

    // Target should be in front of the camera (Z > 0)
    if (target_in_cam.z() <= camera_.min_range || target_in_cam.z() > camera_.max_range) {
        return false;
    }

    // Check horizontal FOV
    const double tan_hfov = std::tan(camera_.h_fov / 2.0);
    if (std::abs(target_in_cam.x() / target_in_cam.z()) > tan_hfov) {
        return false;
    }

    // Check vertical FOV
    const double tan_vfov = std::tan(camera_.v_fov / 2.0);
    if (std::abs(target_in_cam.y() / target_in_cam.z()) > tan_vfov) {
        return false;
    }

    return true;
}

}  // namespace cs625_nbv
