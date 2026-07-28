#include "cs625_nbv/information_gain.hpp"
#include <algorithm>
#include <cmath>

namespace cs625_nbv {

InformationGain::InformationGain()
    : weights_{}, utility_{} {}

InformationGain::InformationGain(const WeightConfig& w, const UtilityConfig& u)
    : weights_(w), utility_(u) {}

Eigen::Matrix<double, 6, 6> InformationGain::build_weight_matrix() const
{
    Eigen::Matrix<double, 6, 6> W = Eigen::Matrix<double, 6, 6>::Zero();
    W(0, 0) = weights_.w_tx;
    W(1, 1) = weights_.w_ty;
    W(2, 2) = weights_.w_tz;
    W(3, 3) = weights_.w_rx;
    W(4, 4) = weights_.w_ry;
    W(5, 5) = weights_.w_rz;
    return W;
}

double InformationGain::compute_ig(
    const Eigen::Matrix<double, 6, 6>& Sigma_t,
    const Eigen::Matrix<double, 6, 6>& expected_Sigma) const
{
    Eigen::Matrix<double, 6, 6> W = build_weight_matrix();

    // IG = max(0, tr(W * Sigma_t) - tr(W * E[Sigma_{t+1}]))
    double trace_current = (W * Sigma_t).trace();
    double trace_expected = (W * expected_Sigma).trace();

    return std::max(0.0, trace_current - trace_expected);
}

Eigen::Matrix<double, 6, 6> InformationGain::expected_covariance(
    const Eigen::Matrix<double, 6, 6>& Sigma_t,
    const Eigen::Isometry3d& camera_pose,
    const Eigen::Vector3d& target_center,
    double visibility_fraction) const
{
    // Compute viewing geometry factors
    Eigen::Vector3d cam_to_target = target_center - camera_pose.translation();
    double distance = cam_to_target.norm();

    // Cosine of angle between camera viewing direction (Z axis) and target direction
    Eigen::Vector3d camera_z = camera_pose.rotation().col(2);  // Camera forward axis
    Eigen::Vector3d to_target_dir = cam_to_target.normalized();
    double cos_angle = std::abs(camera_z.dot(to_target_dir));

    // Information matrix update: Lambda_{t+1} = Lambda_t + H^T R^{-1} H
    // Simplified model: measurement quality ∝ cos_angle * visibility / distance²
    double quality_factor = cos_angle * visibility_fraction / (distance * distance + 0.01);

    // Clamp quality factor
    quality_factor = std::max(0.0, std::min(quality_factor, 100.0));

    // Build information matrix increment (simplified isotropic model)
    // In practice, this should be derived from the measurement Jacobian of ICP
    double info_t = quality_factor * 1e4;   // Translation information
    double info_r = quality_factor * 1e2;   // Rotation information

    Eigen::Matrix<double, 6, 6> H_info = Eigen::Matrix<double, 6, 6>::Zero();
    H_info(0, 0) = info_t;
    H_info(1, 1) = info_t;
    H_info(2, 2) = info_t;  // Depth direction gets more info
    H_info(3, 3) = info_r;
    H_info(4, 4) = info_r;
    H_info(5, 5) = info_r;

    // Current information matrix (inverse of covariance)
    Eigen::Matrix<double, 6, 6> Lambda_t = Sigma_t.inverse();

    // Updated information matrix
    Eigen::Matrix<double, 6, 6> Lambda_next = Lambda_t + H_info;

    // Expected covariance after observation
    return Lambda_next.inverse();
}

double InformationGain::compute_utility(
    double information_gain,
    double path_length,
    double planning_time) const
{
    double denominator = 1.0 + path_length / utility_.L0
                         + utility_.lambda_tau * planning_time / utility_.tau_0;
    return information_gain / denominator;
}

void InformationGain::set_weights(const WeightConfig& w) { weights_ = w; }

void InformationGain::set_utility_config(const UtilityConfig& u) { utility_ = u; }

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

void score_candidates(
    std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates,
    const Eigen::Matrix<double, 6, 6>& current_covariance,
    InformationGain& ig)
{
    for (auto& c : candidates) {
        if (!c.reachable) {
            c.information_gain = 0.0;
            c.utility_score = 0.0;
            continue;
        }

        // Build camera pose from candidate
        Eigen::Isometry3d cam_pose = Eigen::Isometry3d::Identity();
        cam_pose.translation() = Eigen::Vector3d(
            c.pose.pose.position.x,
            c.pose.pose.position.y,
            c.pose.pose.position.z
        );
        Eigen::Quaterniond q(
            c.pose.pose.orientation.w,
            c.pose.pose.orientation.x,
            c.pose.pose.orientation.y,
            c.pose.pose.orientation.z
        );
        cam_pose.linear() = q.toRotationMatrix();

        // Default target: camera looks at origin. In practice, the orchestrator
        // will set the target center before calling this function.
        Eigen::Vector3d target(0.5, 0.3, 0.845);

        // Analytic virtual-cuboid visibility predictor. It only uses the
        // candidate pose and known model dimensions, not the captured cloud.
        // This gives the uncertainty-only/PoseGain ablation non-identical
        // scores while preserving a causal, pre-observation selection rule.
        const Eigen::Vector3d view_dir = (cam_pose.translation() - target).normalized();
        constexpr double area_yz = 0.06 * 0.10;
        constexpr double area_xz = 0.08 * 0.10;
        constexpr double area_xy = 0.08 * 0.06;
        const double projected_area = area_yz * std::abs(view_dir.x())
            + area_xz * std::abs(view_dir.y())
            + area_xy * std::abs(view_dir.z());
        const double visibility = std::max(0.05, std::min(1.0,
            0.8 * projected_area / (area_yz + area_xz + area_xy)));

        Eigen::Matrix<double, 6, 6> expected = ig.expected_covariance(
            current_covariance, cam_pose, target, visibility
        );

        c.information_gain = ig.compute_ig(current_covariance, expected);
        c.utility_score = ig.compute_utility(
            c.information_gain, c.path_length, c.planning_time
        );
    }
}

}  // namespace cs625_nbv
