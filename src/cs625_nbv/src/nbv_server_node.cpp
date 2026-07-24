/**
 * @file nbv_server_node.cpp
 * @brief ROS2 node that wraps the NbvOrchestrator as a service server.
 *
 * Subscribes to /camera/points (depth camera), /joint_states (robot state),
 * and provides the /cs625_nbv/run_episode service.
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "cs625_nbv/nbv_orchestrator.hpp"
#include "cs625_nbv/baseline_strategies.hpp"
#include "cs625_nbv/experiment_logger.hpp"
#include "cs625_nbv/srv/run_nbv_episode.hpp"
#include "cs625_nbv/srv/get_experiment_results.hpp"

using namespace std::chrono_literals;

class NbvServerNode : public rclcpp::Node {
public:
    NbvServerNode()
        : Node("cs625_nbv_server")
    {
        // Declare parameters
        this->declare_parameter("view_distance", 0.5);
        this->declare_parameter("sample_count", 42);
        this->declare_parameter("max_views", 6);
        this->declare_parameter("translation_threshold", 0.003);
        this->declare_parameter("rotation_threshold_deg", 2.0);
        this->declare_parameter("log_base_dir", "~/nbv_experiments");

        // Configure orchestrator
        cs625_nbv::CameraModel camera;
        camera.h_fov = 1.047;   // 60 deg
        camera.v_fov = 0.785;   // 45 deg
        camera.min_range = 0.05;
        camera.max_range = 5.0;

        cs625_nbv::NbvOrchestrator::StopCriteria stop;
        stop.max_views = this->get_parameter("max_views").as_int();
        stop.translation_threshold = this->get_parameter("translation_threshold").as_double();
        stop.rotation_threshold = this->get_parameter("rotation_threshold_deg").as_double();

        orchestrator_.configure(camera, stop,
            cs625_nbv::InformationGain::WeightConfig{},
            cs625_nbv::InformationGain::UtilityConfig{});
        logger_ = std::make_unique<cs625_nbv::ExperimentLogger>(
            this->get_parameter("log_base_dir").as_string()
        );

        // Subscriptions
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/points", 10,
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                latest_cloud_ = *msg;
                has_cloud_ = true;
            }
        );

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                latest_joints_ = *msg;
            }
        );

        // Service servers
        run_episode_srv_ = this->create_service<cs625_nbv::srv::RunNbvEpisode>(
            "/cs625_nbv/run_episode",
            [this](
                const std::shared_ptr<cs625_nbv::srv::RunNbvEpisode::Request> req,
                std::shared_ptr<cs625_nbv::srv::RunNbvEpisode::Response> resp
            ) {
                handle_run_episode(req, resp);
            }
        );

        // Status topic
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/cs625_nbv/status", 10);

        RCLCPP_INFO(this->get_logger(), "NBV server node initialized");
    }

private:
    void handle_run_episode(
        const std::shared_ptr<cs625_nbv::srv::RunNbvEpisode::Request> req,
        std::shared_ptr<cs625_nbv::srv::RunNbvEpisode::Response> resp)
    {
        RCLCPP_INFO(this->get_logger(), "Starting NBV episode: strategy=%s",
                    req->strategy_name.c_str());

        // Never report convergence from an empty camera message. PCL accepts
        // the default-constructed PointCloud2 far enough to produce a tiny
        // covariance, which looks like a successful episode but contains no
        // measurement evidence.
        if (!has_cloud_ || latest_cloud_.data.empty()) {
            resp->success = false;
            resp->message =
                "No point cloud received on /camera/points; episode not started";
            RCLCPP_WARN(
                this->get_logger(),
                "Rejecting NBV episode because /camera/points is empty"
            );
            return;
        }

        // Set up callbacks for simulation (capture from latest cloud)
        orchestrator_.set_capture_callback([this]() -> sensor_msgs::msg::PointCloud2 {
            return latest_cloud_;
        });

        orchestrator_.set_fuse_callback([this](const sensor_msgs::msg::PointCloud2& /*cloud*/) {
            RCLCPP_DEBUG(this->get_logger(), "Point cloud fused");
        });

        // Set move/plan callbacks (stub — in full implementation, call MoveIt services)
        orchestrator_.set_plan_callback([this](const geometry_msgs::msg::Pose& target) -> bool {
            RCLCPP_INFO(this->get_logger(), "Plan to pose: [%.2f, %.2f, %.2f]",
                        target.position.x, target.position.y, target.position.z);
            return true;
        });
        orchestrator_.set_move_callback([this]() -> bool {
            RCLCPP_INFO(this->get_logger(), "Execute move");
            return true;
        });

        // Parse strategy
        cs625_nbv::BaselineStrategies bs;
        int strategy_id = 0;
        if (req->strategy_name == "fixed_order") strategy_id = 1;
        else if (req->strategy_name == "random_reachable") strategy_id = 2;
        else if (req->strategy_name == "coverage_greedy") strategy_id = 3;
        else if (req->strategy_name == "uncertainty_only") strategy_id = 4;
        else if (req->strategy_name == "pose_gain") strategy_id = 5;

        // Target position (for now, hardcoded to table center)
        Eigen::Vector3d target(0.5, 0.3, 0.845);

        // Initial viewpoint
        geometry_msgs::msg::PoseStamped init_view;
        init_view.header.frame_id = "base_link";
        init_view.pose.position.x = 0.3;
        init_view.pose.position.y = 0.3;
        init_view.pose.position.z = 0.9;
        init_view.pose.orientation.w = 1.0;

        // Run episode
        auto result = orchestrator_.run_episode(target, strategy_id, init_view);

        // Log
        logger_->start_episode(req->strategy_name, "default_scene", 0);
        logger_->end_episode(result);

        // Fill response
        resp->success = (result.stop_reason != cs625_nbv::NbvOrchestrator::ERROR);
        resp->result = result;
        resp->message = resp->success ? "Episode completed" : "Episode failed";

        RCLCPP_INFO(this->get_logger(),
                    "Episode done: %d views, final error=%.4f m, converged=%s",
                    result.total_views, result.final_translation_error,
                    result.converged ? "yes" : "no");
    }

    // Components
    cs625_nbv::NbvOrchestrator orchestrator_;
    std::unique_ptr<cs625_nbv::ExperimentLogger> logger_;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

    // Services
    rclcpp::Service<cs625_nbv::srv::RunNbvEpisode>::SharedPtr run_episode_srv_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    // Latest data
    sensor_msgs::msg::PointCloud2 latest_cloud_;
    sensor_msgs::msg::JointState latest_joints_;
    bool has_cloud_{false};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NbvServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
