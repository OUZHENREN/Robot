#pragma once

#include <memory>
#include <string>
#include <functional>
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
     *                            4=uncertainty_only, 5=pose_gain).
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

    // Callbacks
    PlanCallback plan_cb_;
    MoveCallback move_cb_;
    CaptureCallback capture_cb_;
    FuseCallback fuse_cb_;

    // Episode data
    EpisodeResult current_episode_;
};

}  // namespace cs625_nbv
