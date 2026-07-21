#include "cs625_nbv/nbv_orchestrator.hpp"
#include "cs625_nbv/baseline_strategies.hpp"
#include <algorithm>
#include <iostream>
#include <limits>

namespace cs625_nbv {

NbvOrchestrator::NbvOrchestrator()
    : covariance_estimator_(30, 0.005, 0.02)  // K=30, 5mm, 0.02rad noise
{
    current_covariance_ = Eigen::Matrix<double, 6, 6>::Identity();
    // Default: high initial uncertainty (~10cm trans, ~10deg rot)
    current_covariance_(0, 0) = 0.01;   // 10cm² in X
    current_covariance_(1, 1) = 0.01;
    current_covariance_(2, 2) = 0.01;
    current_covariance_(3, 3) = 0.03;   // ~10deg²
    current_covariance_(4, 4) = 0.03;
    current_covariance_(5, 5) = 0.03;
}

NbvOrchestrator::~NbvOrchestrator() = default;

void NbvOrchestrator::configure(
    const CameraModel& camera,
    const StopCriteria& stop,
    const InformationGain::WeightConfig& weights,
    const InformationGain::UtilityConfig& utility)
{
    viewpoint_sampler_ = ViewpointSampler(camera);
    stop_ = stop;
    information_gain_.set_weights(weights);
    information_gain_.set_utility_config(utility);
}

void NbvOrchestrator::set_model_cloud(const sensor_msgs::msg::PointCloud2& cloud)
{
    pose_estimator_.set_model_cloud(cloud);
}

EpisodeResult NbvOrchestrator::run_episode(
    const Eigen::Vector3d& target_center,
    int strategy,
    const geometry_msgs::msg::PoseStamped& initial_viewpoint)
{
    current_episode_ = EpisodeResult();
    auto& result = current_episode_;
    view_count_ = 0;
    total_path_length_ = 0.0;
    total_planning_time_ = 0.0;
    ig_history_.clear();
    error_history_.clear();
    last_ig_ = std::numeric_limits<double>::max();

    BaselineStrategies baselines;

    // ====================================================================
    // Step 0: Move to initial viewpoint, capture first point cloud
    // ====================================================================
    transition(SELECT);
    if (plan_cb_ && move_cb_ && capture_cb_) {
        if (!plan_cb_(initial_viewpoint.pose) || !move_cb_()) {
            transition(ERROR_STATE);
            result.stop_reason = ERROR;
            return result;
        }
        auto cloud = capture_cb_();
        if (fuse_cb_) fuse_cb_(cloud);
    }

    // ====================================================================
    // BOOTSTRAP: run K=30 ICP to get initial covariance
    // ====================================================================
    transition(BOOTSTRAP);
    if (capture_cb_) {
        auto cloud = capture_cb_();
        Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
        current_covariance_ = covariance_estimator_.estimate_covariance(
            pose_estimator_, cloud, init_guess
        );
    }
    view_count_ = 1;

    // ====================================================================
    // Main NBV loop
    // ====================================================================
    while (view_count_ < stop_.max_views) {
        // ---------------------------------------------------------------
        // SELECT: generate candidates, score, select best
        // ---------------------------------------------------------------
        transition(SELECT);

        const int N_CANDIDATES = 42;
        const double VIEW_DISTANCE = 0.5;  // 50cm viewing distance

        auto candidates_raw = viewpoint_sampler_.generate_candidates(
            target_center, VIEW_DISTANCE, N_CANDIDATES
        );

        // Convert to ViewpointCandidate messages
        std::vector<cs625_nbv::msg::ViewpointCandidate> candidates;
        for (auto& pose : candidates_raw) {
            cs625_nbv::msg::ViewpointCandidate c;
            c.pose = pose;
            c.reachable = true;  // Will be set by IK check in real implementation
            c.path_length = 0.3; // Default estimate
            c.planning_time = 2.0;
            candidates.push_back(c);
        }

        result.candidates_generated += static_cast<int>(candidates.size());

        // Score them with information gain
        score_candidates(candidates, current_covariance_, information_gain_);

        // Count reachable
        for (const auto& c : candidates) {
            if (c.reachable) result.reachable_candidates++;
        }
        result.unreachable_candidates = result.candidates_generated - result.reachable_candidates;
        result.unreachable_ratio = static_cast<double>(result.unreachable_candidates)
                                 / std::max(1, result.candidates_generated);

        // Select next viewpoint
        int selected_idx = baselines.select_next(
            static_cast<BaselineStrategies::Type>(strategy), candidates, view_count_
        );

        if (selected_idx < 0) {
            result.stop_reason = ERROR;
            break;
        }

        // ---------------------------------------------------------------
        // PLAN + MOVE: execute trajectory to selected viewpoint
        // ---------------------------------------------------------------
        transition(PLAN);
        if (plan_cb_ && !plan_cb_(candidates[selected_idx].pose.pose)) {
            result.stop_reason = ERROR;
            break;
        }

        transition(MOVE);
        if (move_cb_ && !move_cb_()) {
            result.stop_reason = ERROR;
            break;
        }

        // ---------------------------------------------------------------
        // CAPTURE + FUSE: get and integrate new point cloud
        // ---------------------------------------------------------------
        transition(CAPTURE);
        auto new_cloud = capture_cb_ ? capture_cb_() : sensor_msgs::msg::PointCloud2();

        transition(FUSE);
        if (fuse_cb_) fuse_cb_(new_cloud);

        // ---------------------------------------------------------------
        // SCORE: re-estimate pose and covariance
        // ---------------------------------------------------------------
        transition(SCORE);
        Eigen::Isometry3d new_estimate = Eigen::Isometry3d::Identity();
        if (!new_cloud.data.empty()) {
            new_estimate = pose_estimator_.estimate_pose(new_cloud, current_pose_);
        }
        current_covariance_ = covariance_estimator_.estimate_covariance(
            pose_estimator_, new_cloud, new_estimate
        );
        current_pose_ = new_estimate;

        // Compute achieved IG
        double ig_achieved = last_ig_ > 1e-10
            ? (last_ig_ - (information_gain_.build_weight_matrix() * current_covariance_).trace())
              / last_ig_
            : 0.0;
        ig_history_.push_back(ig_achieved);

        view_count_++;
        total_path_length_ += candidates[selected_idx].path_length;
        total_planning_time_ += candidates[selected_idx].planning_time;

        // ---------------------------------------------------------------
        // CHECK_STOP
        // ---------------------------------------------------------------
        transition(CHECK_STOP);

        // Check convergence from current covariance
        double trans_error = std::sqrt(
            current_covariance_(0, 0) + current_covariance_(1, 1) + current_covariance_(2, 2)
        );
        double rot_error = std::sqrt(
            current_covariance_(3, 3) + current_covariance_(4, 4) + current_covariance_(5, 5)
        );

        if (trans_error < stop_.translation_threshold &&
            rot_error < (stop_.rotation_threshold * M_PI / 180.0)) {
            result.converged = true;
            result.stop_reason = CONVERGED;
            break;
        }

        last_ig_ = (information_gain_.build_weight_matrix() * current_covariance_).trace();
    }  // end main loop

    if (!result.converged && result.stop_reason != ERROR) {
        result.stop_reason = MAX_VIEWS;
    }

    // Populate final result
    transition(DONE);
    result.total_views = view_count_;
    result.total_path_length = total_path_length_;
    result.total_planning_time = total_planning_time_;

    // Compute final errors from covariance
    result.final_translation_error = std::sqrt(
        current_covariance_(0, 0) + current_covariance_(1, 1) + current_covariance_(2, 2)
    );
    result.final_rotation_error = std::sqrt(
        current_covariance_(3, 3) + current_covariance_(4, 4) + current_covariance_(5, 5)
    );
    result.final_add_score = result.final_translation_error;  // Simplified ADD

    return result;
}

void NbvOrchestrator::transition(State next)
{
    state_ = next;
}

std::string NbvOrchestrator::get_state_name() const
{
    switch (state_) {
        case IDLE:       return "IDLE";
        case BOOTSTRAP:  return "BOOTSTRAP";
        case SELECT:     return "SELECT";
        case PLAN:       return "PLAN";
        case MOVE:       return "MOVE";
        case CAPTURE:    return "CAPTURE";
        case FUSE:       return "FUSE";
        case SCORE:      return "SCORE";
        case CHECK_STOP: return "CHECK_STOP";
        case DONE:       return "DONE";
        case ERROR_STATE: return "ERROR_STATE";
        default:         return "UNKNOWN";
    }
}

}  // namespace cs625_nbv
