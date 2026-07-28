#pragma once

#include <memory>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace cs625_nbv {

/**
 * @brief 6-DoF pose estimation using ICP registration.
 *
 * Given a model point cloud (pre-loaded from known object geometry)
 * and a scene point cloud (captured by the depth camera), estimates
 * the rigid transformation that aligns the model to the scene.
 */
class PoseEstimator {
public:
    PoseEstimator();
    ~PoseEstimator();

    /**
     * @brief Set the model (target object) point cloud.
     */
    void set_model_cloud(const sensor_msgs::msg::PointCloud2& cloud);

    /**
     * @brief Set the model cloud from an Eigen matrix of points.
     */
    void set_model_cloud(const Eigen::Matrix<double, 3, Eigen::Dynamic>& points);

    /**
     * @brief Estimate the pose of the model in the scene cloud.
     *
     * @param scene_cloud   Point cloud from the depth camera.
     * @param initial_guess Initial transform guess (optional, helps ICP convergence).
     * @param max_iterations Maximum ICP iterations.
     * @param max_correspondence_distance Max distance for point correspondences (m).
     * @return              Estimated transform T_base_model (model in base_link frame).
     */
    Eigen::Isometry3d estimate_pose(
        const sensor_msgs::msg::PointCloud2& scene_cloud,
        const Eigen::Isometry3d& initial_guess = Eigen::Isometry3d::Identity(),
        int max_iterations = 50,
        double max_correspondence_distance = 0.05
    );

    /// ICP point-to-point RMS residual from the most recent estimate (metres).
    /// This is an observation-conditioned uncertainty proxy, not ground truth.
    double last_registration_rmse() const;

    /**
     * @brief Convert Eigen transform to ROS PoseStamped.
     */
    static geometry_msgs::msg::PoseStamped to_pose_stamped(
        const Eigen::Isometry3d& transform,
        const std::string& frame_id = "base_link"
    );

private:
    /// Model point cloud (PCL format, stored internally)
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace cs625_nbv
