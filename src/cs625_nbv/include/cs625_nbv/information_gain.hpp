#pragma once

#include <Eigen/Dense>
#include <vector>
#include "cs625_nbv/msg/viewpoint_candidate.hpp"

namespace cs625_nbv {

/**
 * @brief Pose information gain and utility scoring for NBV.
 *
 * IG_pose(v) = max(0, tr(W * Sigma_t) - tr(W * E[Sigma_{t+1} | v]))
 * U(v)       = IG_pose(v) / (1 + l(v)/L0 + lambda_tau * tau(v)/tau_0)
 */
class InformationGain {
public:
    /// Weight matrix configuration
    struct WeightConfig {
        double w_tx{1.0}, w_ty{1.0}, w_tz{1.0};  // Translation weights
        double w_rx{0.5}, w_ry{0.5}, w_rz{0.5};  // Rotation weights
    };

    /// Utility scoring parameters
    struct UtilityConfig {
        double L0{1.0};            // Path length normalization (m)
        double tau_0{5.0};         // Planning time normalization (s)
        double lambda_tau{0.3};    // Time cost weight
    };

    InformationGain();  // Default weights and utility config
    InformationGain(const WeightConfig& w,
                    const UtilityConfig& u);

    /**
     * @brief Compute information gain for a candidate viewpoint.
     *
     * @param Sigma_t         Current pose covariance (6×6).
     * @param expected_Sigma  Expected covariance after observing from viewpoint v.
     * @return                IG_pose(v), non-negative.
     */
    double compute_ig(const Eigen::Matrix<double, 6, 6>& Sigma_t,
                      const Eigen::Matrix<double, 6, 6>& expected_Sigma) const;

    /**
     * @brief Compute approximate expected covariance after viewpoint v.
     *
     * Simplified model: uncertainty reduction scales with:
     *   - Viewing angle (cosine of angle between camera axis and surface normal)
     *   - Distance (inverse square)
     *   - Visibility fraction (how much of the target is visible)
     *
     * @param Sigma_t         Current covariance.
     * @param camera_pose     Camera pose for viewpoint v.
     * @param target_center   Target object centroid.
     * @param visibility_fraction Fraction of target visible from v (0.0–1.0).
     * @return                Expected Sigma_{t+1} after observation.
     */
    Eigen::Matrix<double, 6, 6> expected_covariance(
        const Eigen::Matrix<double, 6, 6>& Sigma_t,
        const Eigen::Isometry3d& camera_pose,
        const Eigen::Vector3d& target_center,
        double visibility_fraction
    ) const;

    /**
     * @brief Compute utility U(v) = IG / (1 + l/L0 + lambda_tau * tau/tau_0).
     */
    double compute_utility(double information_gain,
                           double path_length,
                           double planning_time) const;

    /**
     * @brief Update weight matrix.
     */
    void set_weights(const WeightConfig& w);

    /**
     * @brief Update utility config.
     */
    void set_utility_config(const UtilityConfig& u);

    /// Build diagonal weight matrix W (6×6).
    Eigen::Matrix<double, 6, 6> build_weight_matrix() const;

private:
    WeightConfig weights_;
    UtilityConfig utility_;
};

/**
 * @brief Score and rank candidate viewpoints.
 *
 * @param candidates         List of candidate viewpoints (modified in-place).
 * @param current_covariance Current 6×6 pose covariance.
 * @param ig                 Information gain computer.
 */
void score_candidates(
    std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates,
    const Eigen::Matrix<double, 6, 6>& current_covariance,
    InformationGain& ig
);

}  // namespace cs625_nbv
