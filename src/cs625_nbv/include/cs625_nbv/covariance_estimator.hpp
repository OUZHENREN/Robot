#pragma once

#include <Eigen/Dense>
#include <random>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "cs625_nbv/pose_estimator.hpp"

namespace cs625_nbv {

/**
 * @brief Bootstrap covariance estimator for 6D pose uncertainty.
 *
 * Runs K repeated ICP registrations with small perturbations
 * (sensor noise, initial transform jitter) and computes the
 * empirical 6×6 covariance matrix in SE(3) tangent space.
 *
 * This is the core research component — the quality of Sigma
 * directly determines the quality of the information gain metric.
 */
class CovarianceEstimator {
public:
    /**
     * @param K     Number of bootstrap iterations (default: 30).
     * @param trans_noise  Translation noise std for perturbation (m, default: 0.005).
     * @param rot_noise    Rotation noise std for perturbation (rad, default: 0.02).
     */
    CovarianceEstimator(int K = 30,
                        double trans_noise = 0.005,
                        double rot_noise = 0.02);

    /**
     * @brief Estimate pose covariance via bootstrap ICP.
     *
     * @param estimator         Pose estimator with model cloud already set.
     * @param scene_cloud       Point cloud from the depth camera.
     * @param initial_guess     Initial transform guess.
     * @return                  6×6 covariance matrix in SE(3) tangent space.
     *                          First 3 elements = translation, last 3 = rotation.
     */
    Eigen::Matrix<double, 6, 6> estimate_covariance(
        PoseEstimator& estimator,
        const sensor_msgs::msg::PointCloud2& scene_cloud,
        const Eigen::Isometry3d& initial_guess
    );

    /**
     * @brief Perturb a pose with Gaussian noise.
     */
    Eigen::Isometry3d perturb_pose(const Eigen::Isometry3d& pose);

    /**
     * @brief Convert a pose to a 6D tangent-space vector.
     */
    static Eigen::Matrix<double, 6, 1> pose_to_tangent(const Eigen::Isometry3d& pose);

    /**
     * @brief Convert a 6D tangent-space vector back to a pose.
     */
    static Eigen::Isometry3d tangent_to_pose(const Eigen::Matrix<double, 6, 1>& xi);

    int get_K() const { return K_; }
    void set_K(int K) { K_ = K > 1 ? K : 2; }
    void set_seed(uint32_t seed) { generator_.seed(seed); }

private:
    int K_;
    double trans_noise_;
    double rot_noise_;
    std::mt19937 generator_;
};

}  // namespace cs625_nbv
