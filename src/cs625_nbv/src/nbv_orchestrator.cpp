#include "cs625_nbv/nbv_orchestrator.hpp"
#include "cs625_nbv/baseline_strategies.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace cs625_nbv {
namespace {

double translationErrorToVirtualTruth(const Eigen::Isometry3d& estimate)
{
    // The virtual model cloud is authored directly in base_link, so its known
    // ground-truth transform is identity. This must never be used for hardware.
    return estimate.translation().norm();
}

double rotationErrorToVirtualTruth(const Eigen::Isometry3d& estimate)
{
    return Eigen::AngleAxisd(estimate.rotation()).angle();
}

struct TranslationPrediction {
    double reduction{0.0};
    double prior_std{0.0};
    double predicted_posterior_std{0.0};
    double view_novelty{0.0};
    double observability_score{0.0};
};

TranslationPrediction expectedTranslationStdReduction(
    const InformationGain& information_gain,
    const Eigen::Matrix<double, 6, 6>& covariance,
    const geometry_msgs::msg::PoseStamped& candidate,
    const Eigen::Vector3d& target,
    const std::vector<Eigen::Vector3d>& prior_view_directions)
{
    Eigen::Isometry3d camera_pose = Eigen::Isometry3d::Identity();
    camera_pose.translation() = Eigen::Vector3d(
        candidate.pose.position.x, candidate.pose.position.y, candidate.pose.position.z
    );
    const Eigen::Quaterniond orientation(
        candidate.pose.orientation.w, candidate.pose.orientation.x,
        candidate.pose.orientation.y, candidate.pose.orientation.z
    );
    camera_pose.linear() = orientation.toRotationMatrix();

    const auto prediction = information_gain.predict_observation(
        covariance, camera_pose, target, prior_view_directions
    );
    const auto translation_std = [](const auto& sigma) {
        return std::sqrt(std::max(0.0, sigma(0, 0) + sigma(1, 1) + sigma(2, 2)));
    };
    TranslationPrediction result;
    result.prior_std = translation_std(covariance);
    result.predicted_posterior_std = translation_std(prediction.expected_covariance);
    result.reduction = std::max(0.0, result.prior_std - result.predicted_posterior_std);
    result.view_novelty = prediction.view_novelty;
    result.observability_score = prediction.observability_score;
    return result;
}

}  // namespace

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
    model_point_count_ = cloud.width * std::max(1U, cloud.height);
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
    step_records_.clear();
    executed_view_directions_.clear();
    current_pose_ = Eigen::Isometry3d::Identity();
    current_camera_position_ = Eigen::Vector3d(
        initial_viewpoint.pose.position.x,
        initial_viewpoint.pose.position.y,
        initial_viewpoint.pose.position.z
    );
    const Eigen::Vector3d initial_view_offset = current_camera_position_ - target_center;
    if (initial_view_offset.squaredNorm() > 1e-12) {
        executed_view_directions_.push_back(initial_view_offset.normalized());
    }
    current_covariance_ = Eigen::Matrix<double, 6, 6>::Identity();
    current_covariance_(0, 0) = 0.01;
    current_covariance_(1, 1) = 0.01;
    current_covariance_(2, 2) = 0.01;
    current_covariance_(3, 3) = 0.03;
    current_covariance_(4, 4) = 0.03;
    current_covariance_(5, 5) = 0.03;
    last_ig_ = 0.0;

    BaselineStrategies baselines(random_seed_);

    // ====================================================================
    // Step 0: Move to initial viewpoint, capture first point cloud
    // ====================================================================
    transition(SELECT);
    if (plan_cb_ && move_cb_ && capture_cb_) {
        if (!plan_cb_(initial_viewpoint.pose) || !move_cb_()) {
            transition(ERROR_STATE);
            result.stop_reason = ERROR;
            result.failure_reason = "initial_plan_or_move_failed";
            return result;
        }
        auto cloud = capture_cb_();
        if (fuse_cb_) fuse_cb_(cloud);
    }

    // ====================================================================
    // BOOTSTRAP: run K=30 ICP to get initial covariance
    // ====================================================================
    transition(BOOTSTRAP);
    sensor_msgs::msg::PointCloud2 bootstrap_cloud;
    double bootstrap_rmse = 0.0;
    if (capture_cb_) {
        for (int attempt = 0; attempt < 20 && bootstrap_cloud.data.empty(); ++attempt) {
            bootstrap_cloud = capture_cb_();
            if (bootstrap_cloud.data.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
        if (bootstrap_cloud.data.empty()) {
            // A service becoming available does not guarantee that the
            // synthetic camera has published its first cloud. Continuing
            // would emit a regularisation-floor covariance and a false
            // convergence record, so reject this episode as invalid.
            transition(ERROR_STATE);
            result.failure_reason = "bootstrap_cloud_missing";
            result.stop_reason = ERROR;
            return result;
        }
        Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
        if (observation_model_cb_) {
            const auto visible_model = observation_model_cb_();
            if (!visible_model.data.empty()) {
                pose_estimator_.set_model_cloud(visible_model);
            }
        }
        current_pose_ = pose_estimator_.estimate_pose(bootstrap_cloud, init_guess);
        bootstrap_rmse = pose_estimator_.last_registration_rmse();
        current_covariance_ = covariance_estimator_.estimate_covariance(
            pose_estimator_, bootstrap_cloud, init_guess
        );
    }
    view_count_ = 1;
    const double initial_trans_error = translationErrorToVirtualTruth(current_pose_);
    const double initial_rot_error = rotationErrorToVirtualTruth(current_pose_);
    const double initial_covariance_std = std::sqrt(std::max(0.0,
        current_covariance_(0, 0) + current_covariance_(1, 1) + current_covariance_(2, 2)
    ));
    last_ig_ = (information_gain_.build_weight_matrix() * current_covariance_).trace();
    ig_history_.push_back(0.0);
    error_history_.push_back(initial_trans_error);
    result.ig_per_view.push_back(0.0);
    result.path_length_per_view.push_back(0.0);
    result.error_per_view.push_back(initial_trans_error);
    step_records_.push_back(StepRecord{
        0, initial_viewpoint.pose, initial_trans_error, initial_rot_error,
        initial_trans_error, 0.0, 0.0, 0.0,
        bootstrap_cloud.width * std::max(1U, bootstrap_cloud.height),
        model_point_count_ > 0
            ? static_cast<double>(bootstrap_cloud.width * std::max(1U, bootstrap_cloud.height)) /
                model_point_count_
            : 0.0,
        bootstrap_rmse,
        0.0,
        initial_covariance_std,
        initial_covariance_std,
        initial_covariance_std,
        1.0,
        0.0,
        0, 0
    });

    // ====================================================================
    // Main NBV loop
    // ====================================================================
    const int episode_max_views =
        strategy == BaselineStrategies::SINGLE_VIEW ? 1 : stop_.max_views;
    while (view_count_ < episode_max_views) {
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
            const Eigen::Vector3d candidate_position(
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
            );
            c.path_length = (candidate_position - current_camera_position_).norm();
            c.planning_time = 0.25 + 1.5 * c.path_length;
            candidates.push_back(c);
        }

        result.candidates_generated += static_cast<int>(candidates.size());

        // Score them with information gain
        score_candidates(
            candidates, current_covariance_, information_gain_,
            target_center, executed_view_directions_
        );

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
            result.failure_reason = "no_reachable_candidate";
            break;
        }
        const auto prediction = expectedTranslationStdReduction(
            information_gain_, current_covariance_, candidates[selected_idx].pose,
            target_center, executed_view_directions_
        );

        // ---------------------------------------------------------------
        // PLAN + MOVE: execute trajectory to selected viewpoint
        // ---------------------------------------------------------------
        transition(PLAN);
        if (plan_cb_ && !plan_cb_(candidates[selected_idx].pose.pose)) {
            result.stop_reason = ERROR;
            result.failure_reason = "planning_failed";
            break;
        }

        transition(MOVE);
        if (move_cb_ && !move_cb_()) {
            result.stop_reason = ERROR;
            result.failure_reason = "move_failed";
            break;
        }
        current_camera_position_ = Eigen::Vector3d(
            candidates[selected_idx].pose.pose.position.x,
            candidates[selected_idx].pose.pose.position.y,
            candidates[selected_idx].pose.pose.position.z
        );

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
        double registration_rmse = 0.0;
        const Eigen::Matrix<double, 6, 6> prior_covariance = current_covariance_;
        const Eigen::Isometry3d prior_pose = current_pose_;
        if (!new_cloud.data.empty()) {
            if (observation_model_cb_) {
                const auto visible_model = observation_model_cb_();
                if (!visible_model.data.empty()) {
                    pose_estimator_.set_model_cloud(visible_model);
                }
            }
            new_estimate = pose_estimator_.estimate_pose(new_cloud, current_pose_);
            registration_rmse = pose_estimator_.last_registration_rmse();
            const auto measurement_covariance = covariance_estimator_.estimate_covariance(
                pose_estimator_, new_cloud, new_estimate
            );
            current_covariance_ = covariance_estimator_.fuse_covariances(
                prior_covariance, measurement_covariance
            );
            current_pose_ = covariance_estimator_.fuse_pose_estimates(
                prior_pose, prior_covariance, new_estimate, measurement_covariance
            );
        }

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

        // Virtual-only truth error: model and observation share base_link and
        // the authored model transform is identity. Real camera runs must
        // provide an independently measured T_base_object before using this.
        double trans_error = translationErrorToVirtualTruth(current_pose_);
        double rot_error = rotationErrorToVirtualTruth(current_pose_);
        const double covariance_std = std::sqrt(std::max(0.0,
            current_covariance_(0, 0) + current_covariance_(1, 1) + current_covariance_(2, 2)
        ));
        error_history_.push_back(trans_error);
        result.ig_per_view.push_back(ig_achieved);
        result.path_length_per_view.push_back(candidates[selected_idx].path_length);
        result.error_per_view.push_back(trans_error);
        step_records_.push_back(StepRecord{
            view_count_ - 1,
            candidates[selected_idx].pose.pose,
            trans_error,
            rot_error,
            trans_error,
            ig_achieved,
            candidates[selected_idx].path_length,
            candidates[selected_idx].planning_time,
            new_cloud.width * std::max(1U, new_cloud.height),
            model_point_count_ > 0
                ? static_cast<double>(new_cloud.width * std::max(1U, new_cloud.height)) /
                    model_point_count_
                : 0.0,
            registration_rmse,
            prediction.reduction,
            prediction.prior_std,
            prediction.predicted_posterior_std,
            covariance_std,
            prediction.view_novelty,
            prediction.observability_score,
            static_cast<int>(candidates.size()),
            static_cast<int>(std::count_if(
                candidates.begin(), candidates.end(),
                [](const auto& candidate) { return candidate.reachable; }
            ))
        });

        const Eigen::Vector3d selected_view_offset =
            current_camera_position_ - target_center;
        if (selected_view_offset.squaredNorm() > 1e-12) {
            executed_view_directions_.push_back(selected_view_offset.normalized());
        }

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

    result.final_translation_error = translationErrorToVirtualTruth(current_pose_);
    result.final_rotation_error = rotationErrorToVirtualTruth(current_pose_);
    // This remains a translation-only proxy; do not call it CAD ADD/ADD-S.
    result.final_add_score = result.final_translation_error;

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
