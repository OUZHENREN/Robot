#include "cs625_nbv/covariance_estimator.hpp"
#include <random>
#include <cmath>
#include <thread>
#include <Eigen/Geometry>

namespace cs625_nbv {

CovarianceEstimator::CovarianceEstimator(int K, double trans_noise, double rot_noise)
    : K_(K), trans_noise_(trans_noise), rot_noise_(rot_noise), generator_(6252024U) {}

Eigen::Matrix<double, 6, 6> CovarianceEstimator::estimate_covariance(
    PoseEstimator& estimator,
    const sensor_msgs::msg::PointCloud2& scene_cloud,
    const Eigen::Isometry3d& initial_guess)
{
    // Collect K pose estimates
    std::vector<Eigen::Matrix<double, 6, 1>> pose_vectors;
    pose_vectors.reserve(K_);

    // Run ICP K times with perturbations (reuse same estimator instance)
    for (int k = 0; k < K_; ++k) {
        // Perturb the initial guess
        Eigen::Isometry3d perturbed_initial = perturb_pose(initial_guess);

        // Run ICP
        Eigen::Isometry3d result = estimator.estimate_pose(
            scene_cloud, perturbed_initial
        );

        // Convert to tangent space
        pose_vectors.push_back(pose_to_tangent(result));
    }

    // Compute mean
    Eigen::Matrix<double, 6, 1> mean = Eigen::Matrix<double, 6, 1>::Zero();
    for (const auto& v : pose_vectors) {
        mean += v;
    }
    mean /= static_cast<double>(K_);

    // Compute empirical covariance
    Eigen::Matrix<double, 6, 6> Sigma = Eigen::Matrix<double, 6, 6>::Zero();
    for (const auto& v : pose_vectors) {
        Eigen::Matrix<double, 6, 1> diff = v - mean;
        Sigma += diff * diff.transpose();
    }
    Sigma /= static_cast<double>(K_ - 1);  // Unbiased estimator

    // Ensure positive semi-definite (add small diagonal if needed)
    for (int i = 0; i < 6; ++i) {
        if (Sigma(i, i) < 1e-12) {
            Sigma(i, i) = 1e-12;
        }
    }

    return Sigma;
}

Eigen::Isometry3d CovarianceEstimator::perturb_pose(const Eigen::Isometry3d& pose)
{
    std::normal_distribution<double> trans_dist(0.0, trans_noise_);
    std::normal_distribution<double> rot_dist(0.0, rot_noise_);

    // Perturb translation
    Eigen::Vector3d trans_perturb(
        trans_dist(generator_), trans_dist(generator_), trans_dist(generator_)
    );

    // Perturb rotation (small rotation vector)
    Eigen::Vector3d rot_perturb(
        rot_dist(generator_), rot_dist(generator_), rot_dist(generator_)
    );
    double angle = rot_perturb.norm();
    Eigen::AngleAxisd rot_aa;
    if (angle > 1e-9) {
        rot_aa = Eigen::AngleAxisd(angle, rot_perturb.normalized());
    } else {
        rot_aa = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    }

    Eigen::Isometry3d perturbed = pose;
    perturbed.translation() += trans_perturb;
    perturbed.linear() = pose.rotation() * rot_aa.toRotationMatrix();

    return perturbed;
}

Eigen::Matrix<double, 6, 1> CovarianceEstimator::pose_to_tangent(const Eigen::Isometry3d& pose)
{
    Eigen::Matrix<double, 6, 1> xi;
    // Translation part
    xi.segment<3>(0) = pose.translation();
    // Rotation part: rotation vector
    Eigen::AngleAxisd aa(pose.rotation());
    xi.segment<3>(3) = aa.angle() * aa.axis();
    return xi;
}

Eigen::Isometry3d CovarianceEstimator::tangent_to_pose(const Eigen::Matrix<double, 6, 1>& xi)
{
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = xi.segment<3>(0);
    double angle = xi.segment<3>(3).norm();
    if (angle > 1e-9) {
        pose.linear() = Eigen::AngleAxisd(angle, xi.segment<3>(3).normalized()).toRotationMatrix();
    } else {
        pose.linear() = Eigen::Matrix3d::Identity();
    }
    return pose;
}

}  // namespace cs625_nbv
