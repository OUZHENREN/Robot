#pragma once

#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace cs625_nbv {

/**
 * @brief Camera model parameters for viewpoint FOV constraint.
 */
struct CameraModel {
    double h_fov{1.047};   // Horizontal FOV in radians (default: 60 degrees)
    double v_fov{0.785};   // Vertical FOV in radians (default: 45 degrees)
    double min_range{0.05}; // Minimum sensing range (m)
    double max_range{5.0};  // Maximum sensing range (m)
    int width{640};
    int height{480};
};

/**
 * @brief Generates candidate viewpoints around a target using Fibonacci sphere
 *        sampling on a hemisphere.
 *
 * Candidate viewpoints are camera poses (position + orientation) such that:
 *  - The camera looks at the target centroid from a given viewing distance.
 *  - The target is within the camera's FOV.
 *  - Viewpoints are spread evenly over a hemisphere above the table plane.
 */
class ViewpointSampler {
public:
    /**
     * @param camera Camera model parameters for FOV constraint.
     */
    explicit ViewpointSampler(const CameraModel& camera = CameraModel{});

    /**
     * @brief Generate N candidate viewpoints on a hemisphere around the target.
     *
     * @param target_center   Target centroid in base_link frame (m).
     * @param view_distance   Viewing distance from camera to target (m).
     * @param sample_count    Number of Fibonacci sphere samples.
     * @param hemisphere_axis Unit vector pointing "up" (default: +Z for tabletop).
     * @return                List of candidate camera poses in base_link frame.
     */
    std::vector<geometry_msgs::msg::PoseStamped> generate_candidates(
        const Eigen::Vector3d& target_center,
        double view_distance,
        int sample_count = 42,
        const Eigen::Vector3d& hemisphere_axis = Eigen::Vector3d::UnitZ()
    ) const;

    /**
     * @brief Check whether the target is within the camera's FOV from a given pose.
     */
    bool is_target_in_fov(const Eigen::Vector3d& target_center,
                          const Eigen::Isometry3d& camera_pose) const;

private:
    CameraModel camera_;

    /// Compute camera orientation matrix that looks from position to target.
    Eigen::Matrix3d look_at(const Eigen::Vector3d& camera_pos,
                            const Eigen::Vector3d& target_pos,
                            const Eigen::Vector3d& up = Eigen::Vector3d::UnitZ()) const;
};

}  // namespace cs625_nbv
