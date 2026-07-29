#pragma once

#include <memory>
#include <string>
#include <functional>
#include <algorithm>
#include <cstdint>
#include <vector>
#include "cs625_nbv/viewpoint_sampler.hpp"
#include "cs625_nbv/pose_estimator.hpp"
#include "cs625_nbv/covariance_estimator.hpp"
#include "cs625_nbv/information_gain.hpp"
#include "cs625_nbv/msg/viewpoint_candidate.hpp"
#include "cs625_nbv/msg/episode_result.hpp"

namespace cs625_nbv {
using msg::EpisodeResult;

/**
 * @brief NBV closed-loop orchestrator state machine.
 *
 * States:
 *   IDLE → BOOTSTRAP → SELECT → PLAN → MOVE → CAPTURE → FUSE → SCORE → CHECK_STOP
 *                                                                         ├─ NO  → SELECT
 *                                                                         └─ YES → DONE
 */
class NbvOrchestrator {
public:
    struct StepRecord {
        int step{0};
        geometry_msgs::msg::Pose viewpoint;
        double translation_error{0.0};
        double rotation_error{0.0};
        double add_score{0.0};
        double information_gain{0.0};
        double path_length{0.0};
        double planning_time{0.0};
        uint32_t observation_points{0};
        double visible_ratio{0.0};
        double registration_rmse{0.0};
        double uncertainty_proxy{0.0};
        double prior_covariance_translation_std{0.0};
        double predicted_posterior_covariance_translation_std{0.0};
        double covariance_translation_std{0.0};
        double observed_covariance_translation_std_reduction{0.0};
        double virtual_initial_translation_bias_m{0.0};
        double view_novelty{0.0};
        double observability_score{0.0};
        int candidates_generated{0};
        int reachable_candidates{0};
    };

    /// Stop reason codes (matching EpisodeResult.msg)
    enum StopReason : int32_t {
        CONVERGED = 0,
        MAX_VIEWS = 1,
        TIMEOUT   = 2,
        ERROR     = 3,
    };

    /// Stop criteria thresholds
    struct StopCriteria {
        double translation_threshold{0.003};    // 3 mm
        double rotation_threshold{2.0};          // degrees (≈0.035 rad)
        double marginal_gain_threshold{0.05};    // 5% relative
        int max_views{6};
    };

    /// Callback types for simulation/real-hardware integration
    using PlanCallback = std::function<bool(const geometry_msgs::msg::Pose& target)>;
    using MoveCallback  = std::function<bool()>;
    using CaptureCallback = std::function<sensor_msgs::msg::PointCloud2()>;
    using FuseCallback  = std::function<void(const sensor_msgs::msg::PointCloud2&)>;
    using ObservationModelCallback = std::function<sensor_msgs::msg::PointCloud2()>;

    NbvOrchestrator();
    ~NbvOrchestrator();

    /**
     * @brief Configure the orchestrator.
     */
    void configure(const CameraModel& camera,
                   const StopCriteria& stop,
                   const InformationGain::WeightConfig& weights,
                   const InformationGain::UtilityConfig& utility);

    /**
     * @brief Register callbacks for hardware interaction.
     *
     * In simulation: callbacks talk to Gazebo/MoveIt.
     * On real robot: callbacks talk to actual hardware.
     */
    void set_plan_callback(PlanCallback cb) { plan_cb_ = std::move(cb); }
    void set_move_callback(MoveCallback cb) { move_cb_ = std::move(cb); }
    void set_capture_callback(CaptureCallback cb) { capture_cb_ = std::move(cb); }
    void set_fuse_callback(FuseCallback cb) { fuse_cb_ = std::move(cb); }
    void set_observation_model_callback(ObservationModelCallback cb) {
        observation_model_cb_ = std::move(cb);
    }
    void set_random_seed(uint32_t random_seed) {
        random_seed_ = random_seed;
        covariance_estimator_.set_seed(random_seed ^ 0x6252024U);
    }
    void set_max_views(int max_views) { stop_.max_views = std::max(1, max_views); }
    void set_covariance_bootstrap_samples(int samples) { covariance_estimator_.set_K(samples); }
    void set_virtual_observation_config(const VirtualObservationConfig& config) {
        information_gain_.set_virtual_observation_config(config);
    }
    // Virtual-only evaluation aid.  It creates a known, deterministic initial
    // translation error and declares the corresponding prior covariance.  It
    // is disabled by default and must never be used to label a hardware run.
    void set_virtual_initial_pose_bias(double translation_bias_m,
                                       double covariance_std_m) {
        virtual_initial_translation_bias_m_ = std::max(0.0, translation_bias_m);
        virtual_initial_covariance_std_m_ = std::max(0.0, covariance_std_m);
    }

    /**
     * @brief Set the model point cloud for ICP registration.
     */
    void set_model_cloud(const sensor_msgs::msg::PointCloud2& cloud);

    /**
     * @brief Run a full NBV episode.
     *
     * @param target_center       Target centroid in base_link frame.
     * @param strategy            Strategy selector (0=single_view, 1=fixed_order,
     *                            2=random_reachable, 3=coverage_greedy,
     *                            4=uncertainty_only, 5=pose_gain,
     *                            6=path_cost_only).
     * @param initial_viewpoint   Starting camera pose (for bootstrap).
     * @return                    Episode result with all metrics.
     */
    EpisodeResult run_episode(
        const Eigen::Vector3d& target_center,
        int strategy,
        const geometry_msgs::msg::PoseStamped& initial_viewpoint
    );

    /// Get current state for monitoring
    std::string get_state_name() const;
    const std::vector<StepRecord>& get_step_records() const { return step_records_; }

private:
    // State machine
    enum State { IDLE, BOOTSTRAP, SELECT, PLAN, MOVE, CAPTURE, FUSE, SCORE, CHECK_STOP, DONE, ERROR_STATE };
    State state_{IDLE};
    void transition(State next);

    // Components
    ViewpointSampler viewpoint_sampler_;
    PoseEstimator pose_estimator_;
    CovarianceEstimator covariance_estimator_;
    InformationGain information_gain_;
    mutable StopCriteria stop_;

    // State
    Eigen::Matrix<double, 6, 6> current_covariance_;
    Eigen::Isometry3d current_pose_;
    int view_count_{0};
    double last_ig_{0.0};
    double total_path_length_{0.0};
    double total_planning_time_{0.0};
    std::vector<double> ig_history_;
    std::vector<double> error_history_;
    std::vector<StepRecord> step_records_;
    uint32_t random_seed_{625U};
    uint32_t model_point_count_{0};
    Eigen::Vector3d current_camera_position_{Eigen::Vector3d::Zero()};
    std::vector<Eigen::Vector3d> executed_view_directions_;
    double virtual_initial_translation_bias_m_{0.0};
    double virtual_initial_covariance_std_m_{0.0};

    // Callbacks
    PlanCallback plan_cb_;
    MoveCallback move_cb_;
    CaptureCallback capture_cb_;
    FuseCallback fuse_cb_;
    ObservationModelCallback observation_model_cb_;

    // Episode data
    EpisodeResult current_episode_;
};

}  // namespace cs625_nbv
