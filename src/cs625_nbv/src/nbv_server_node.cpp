/**
 * @file nbv_server_node.cpp
 * @brief ROS2 node that wraps the NbvOrchestrator as a service server.
 *
 * Subscribes to /camera/points (depth camera), /joint_states (robot state),
 * and provides the /cs625_nbv/run_episode service.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include "cs625_nbv/nbv_orchestrator.hpp"
#include "cs625_nbv/baseline_strategies.hpp"
#include "cs625_nbv/experiment_logger.hpp"
#include "cs625_nbv/srv/run_nbv_episode.hpp"
#include "cs625_nbv/srv/get_experiment_results.hpp"
#include "cs625_nbv/srv/export_experiment_data.hpp"

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
        this->declare_parameter("bootstrap_model_from_first_cloud", false);
        this->declare_parameter("use_model_cloud_topic", false);
        this->declare_parameter("virtual_camera_settle_ms", 250);
        this->declare_parameter("covariance_bootstrap_samples", 30);
        this->declare_parameter("scene_name", "default_scene");
        this->declare_parameter("episode_id", 0);
        this->declare_parameter("random_seed", 625);
        this->declare_parameter("data_source", "synthetic_smoke_test");
        this->declare_parameter("validity_label", "interface_only");
        this->declare_parameter("git_commit", "unknown");
        this->declare_parameter("launch_profile", "unknown");
        this->declare_parameter("occlusion_level", "none");
        this->declare_parameter(
            "uncertainty_model", "p4_sequential_information_fusion_virtual_only"
        );
        this->declare_parameter(
            "observability_model", "projected_visibility_times_view_novelty"
        );
        this->declare_parameter("sensor_noise_seed_contract", "unspecified");
        this->declare_parameter("virtual_initial_translation_bias_m", 0.0);
        this->declare_parameter("virtual_initial_covariance_std_m", 0.0);

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
        cs625_nbv::VirtualObservationConfig virtual_sensor;
        const auto occlusion_level = this->get_parameter("occlusion_level").as_string();
        virtual_sensor.occlusion_fraction = occlusion_level == "heavy" ? 0.50 :
            (occlusion_level == "light" ? 0.20 : 0.0);
        orchestrator_.set_virtual_observation_config(virtual_sensor);
        logger_ = std::make_unique<cs625_nbv::ExperimentLogger>(
            this->get_parameter("log_base_dir").as_string()
        );

        // Subscriptions
        sensor_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant
        );
        rclcpp::SubscriptionOptions sensor_subscription_options;
        sensor_subscription_options.callback_group = sensor_callback_group_;
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/points", 10,
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(sensor_mutex_);
                latest_cloud_ = *msg;
                has_cloud_ = true;
            },
            sensor_subscription_options
        );

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(sensor_mutex_);
                latest_joints_ = *msg;
            },
            sensor_subscription_options
        );
        model_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cs625_nbv/model_cloud", rclcpp::QoS(1).transient_local(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(sensor_mutex_);
                latest_model_cloud_ = *msg;
                has_model_cloud_ = !msg->data.empty();
            },
            sensor_subscription_options
        );
        visible_model_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cs625_nbv/visible_model_cloud", 10,
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(sensor_mutex_);
                latest_visible_model_cloud_ = *msg;
                has_visible_model_cloud_ = !msg->data.empty();
            },
            sensor_subscription_options
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
        get_results_srv_ = this->create_service<cs625_nbv::srv::GetExperimentResults>(
            "/cs625_nbv/get_experiment_results",
            [this](
                const std::shared_ptr<cs625_nbv::srv::GetExperimentResults::Request> req,
                std::shared_ptr<cs625_nbv::srv::GetExperimentResults::Response> resp
            ) {
                handle_get_results(req, resp);
            }
        );
        export_data_srv_ = this->create_service<cs625_nbv::srv::ExportExperimentData>(
            "/cs625_nbv/export_experiment_data",
            [this](
                const std::shared_ptr<cs625_nbv::srv::ExportExperimentData::Request> req,
                std::shared_ptr<cs625_nbv::srv::ExportExperimentData::Response> resp
            ) {
                handle_export_data(req, resp);
            }
        );

        // Status topic
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/cs625_nbv/status", 10);
        virtual_camera_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/cs625_nbv/virtual_camera_pose", 10
        );

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
        sensor_msgs::msg::PointCloud2 initial_cloud;
        sensor_msgs::msg::PointCloud2 model_cloud;
        bool has_cloud = false;
        bool has_model_cloud = false;
        {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            initial_cloud = latest_cloud_;
            model_cloud = latest_model_cloud_;
            has_cloud = has_cloud_;
            has_model_cloud = has_model_cloud_;
        }
        if (!has_cloud || initial_cloud.data.empty()) {
            resp->success = false;
            resp->message =
                "No point cloud received on /camera/points; episode not started";
            resp->output_directory = "";
            RCLCPP_WARN(
                this->get_logger(),
                "Rejecting NBV episode because /camera/points is empty"
            );
            return;
        }

        const bool use_model_cloud_topic =
            this->get_parameter("use_model_cloud_topic").as_bool();
        if (use_model_cloud_topic && !has_model_cloud) {
            resp->success = false;
            resp->message = "No complete model cloud received on /cs625_nbv/model_cloud";
            resp->output_directory = "";
            RCLCPP_WARN(this->get_logger(), "%s", resp->message.c_str());
            return;
        }

        // This is only enabled by the remote synthetic-camera launch profile.
        // Real-camera operation must provide a CAD/model cloud explicitly.
        if (use_model_cloud_topic) {
            orchestrator_.set_model_cloud(model_cloud);
            model_cloud_initialized_ = true;
            RCLCPP_INFO(
                this->get_logger(),
                "Using complete virtual model cloud (%u points) for viewpoint-dependent observation",
                model_cloud.width * std::max(1U, model_cloud.height)
            );
        } else if (this->get_parameter("bootstrap_model_from_first_cloud").as_bool() &&
            !model_cloud_initialized_) {
            orchestrator_.set_model_cloud(latest_cloud_);
            model_cloud_initialized_ = true;
            RCLCPP_WARN(
                this->get_logger(),
                "Bootstrapped ICP model from synthetic first cloud; results are software smoke-test data"
            );
        }

        // Set up callbacks for simulation (capture from latest cloud)
        orchestrator_.set_capture_callback([this]() -> sensor_msgs::msg::PointCloud2 {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            return latest_cloud_;
        });
        if (use_model_cloud_topic) {
            orchestrator_.set_observation_model_callback([this]() -> sensor_msgs::msg::PointCloud2 {
                std::lock_guard<std::mutex> lock(sensor_mutex_);
                return latest_visible_model_cloud_;
            });
        } else {
            orchestrator_.set_observation_model_callback({});
        }

        orchestrator_.set_fuse_callback([this](const sensor_msgs::msg::PointCloud2& /*cloud*/) {
            RCLCPP_DEBUG(this->get_logger(), "Point cloud fused");
        });

        // Set move/plan callbacks (stub — in full implementation, call MoveIt services)
        orchestrator_.set_plan_callback([this](const geometry_msgs::msg::Pose& target) -> bool {
            RCLCPP_INFO(this->get_logger(), "Plan to pose: [%.2f, %.2f, %.2f]",
                        target.position.x, target.position.y, target.position.z);
            geometry_msgs::msg::PoseStamped virtual_camera_pose;
            virtual_camera_pose.header.frame_id = "base_link";
            virtual_camera_pose.header.stamp = this->now();
            virtual_camera_pose.pose = target;
            virtual_camera_pose_pub_->publish(virtual_camera_pose);
            const auto settle_ms = this->get_parameter("virtual_camera_settle_ms").as_int();
            if (settle_ms > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));
            }
            return true;
        });
        orchestrator_.set_move_callback([this]() -> bool {
            RCLCPP_INFO(this->get_logger(), "Execute move");
            return true;
        });

        // Parse strategy
        int strategy_id = -1;
        if (req->strategy_name == "single_view") strategy_id = 0;
        else if (req->strategy_name == "fixed_order") strategy_id = 1;
        else if (req->strategy_name == "random_reachable") strategy_id = 2;
        else if (req->strategy_name == "coverage_greedy") strategy_id = 3;
        else if (req->strategy_name == "uncertainty_only") strategy_id = 4;
        else if (req->strategy_name == "pose_gain") strategy_id = 5;
        else if (req->strategy_name == "path_cost_only") strategy_id = 6;
        if (strategy_id < 0) {
            resp->success = false;
            resp->message = "Unknown strategy_name: " + req->strategy_name;
            resp->output_directory = "";
            return;
        }
        orchestrator_.set_random_seed(
            static_cast<uint32_t>(this->get_parameter("random_seed").as_int())
        );
        orchestrator_.set_covariance_bootstrap_samples(
            this->get_parameter("covariance_bootstrap_samples").as_int()
        );
        orchestrator_.set_virtual_initial_pose_bias(
            this->get_parameter("virtual_initial_translation_bias_m").as_double(),
            this->get_parameter("virtual_initial_covariance_std_m").as_double()
        );
        orchestrator_.set_max_views(
            req->max_views > 0
                ? req->max_views
                : this->get_parameter("max_views").as_int()
        );

        // Target position (for now, hardcoded to table center)
        Eigen::Vector3d target(0.5, 0.3, 0.845);

        // Initial viewpoint
        geometry_msgs::msg::PoseStamped init_view;
        init_view.header.frame_id = "base_link";
        init_view.pose.position.x = 0.3;
        init_view.pose.position.y = 0.3;
        init_view.pose.position.z = 0.9;
        const Eigen::Vector3d initial_position(
            init_view.pose.position.x, init_view.pose.position.y, init_view.pose.position.z);
        const Eigen::Vector3d z_axis = (target - initial_position).normalized();
        Eigen::Vector3d x_axis = Eigen::Vector3d::UnitY().cross(z_axis);
        if (x_axis.norm() < 1e-9) x_axis = Eigen::Vector3d::UnitX().cross(z_axis);
        x_axis.normalize();
        Eigen::Matrix3d rotation;
        rotation.col(0) = x_axis;
        rotation.col(1) = z_axis.cross(x_axis);
        rotation.col(2) = z_axis;
        const Eigen::Quaterniond initial_orientation(rotation);
        init_view.pose.orientation.x = initial_orientation.x();
        init_view.pose.orientation.y = initial_orientation.y();
        init_view.pose.orientation.z = initial_orientation.z();
        init_view.pose.orientation.w = initial_orientation.w();

        // Run episode
        const auto episode_start = std::chrono::steady_clock::now();
        auto result = orchestrator_.run_episode(target, strategy_id, init_view);
        result.total_execution_time = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - episode_start
        ).count();

        result.strategy_name = req->strategy_name;
        result.scene_name = this->get_parameter("scene_name").as_string();
        result.episode_id = this->get_parameter("episode_id").as_int();
        result.data_source = this->get_parameter("data_source").as_string();
        result.validity_label = this->get_parameter("validity_label").as_string();
        result.random_seed = this->get_parameter("random_seed").as_int();

        cs625_nbv::EpisodeMetadata metadata;
        metadata.target_object_id = req->target_object_id;
        metadata.data_source = result.data_source;
        metadata.validity_label = result.validity_label;
        metadata.random_seed = result.random_seed;
        metadata.git_commit = this->get_parameter("git_commit").as_string();
        const char* ros_distro = std::getenv("ROS_DISTRO");
        metadata.ros_distro = ros_distro ? ros_distro : "unknown";
        metadata.launch_profile = this->get_parameter("launch_profile").as_string();

        const std::string output_directory = logger_->start_episode(
            result.strategy_name, result.scene_name, result.episode_id, metadata
        );
        result.run_id = std::filesystem::path(output_directory).filename().string();
        for (const auto& step : orchestrator_.get_step_records()) {
            logger_->log_step(
                step.step,
                step.viewpoint.position.x,
                step.viewpoint.position.y,
                step.viewpoint.position.z,
                step.viewpoint.orientation.x,
                step.viewpoint.orientation.y,
                step.viewpoint.orientation.z,
                step.viewpoint.orientation.w,
                step.translation_error,
                step.rotation_error,
                step.add_score,
                step.information_gain,
                step.path_length,
                step.planning_time,
                step.observation_points,
                step.visible_ratio,
                step.registration_rmse,
                step.uncertainty_proxy,
                step.prior_covariance_translation_std,
                step.predicted_posterior_covariance_translation_std,
                step.covariance_translation_std,
                step.observed_covariance_translation_std_reduction,
                step.virtual_initial_translation_bias_m,
                step.view_novelty,
                step.observability_score,
                step.candidates_generated,
                step.reachable_candidates
            );
        }
        std::ostringstream config;
        config << "target_object_id: " << req->target_object_id << "\n"
               << "strategy_name: " << result.strategy_name << "\n"
               << "scene_name: " << result.scene_name << "\n"
               << "max_views: " << (req->max_views > 0 ? req->max_views :
                    this->get_parameter("max_views").as_int()) << "\n"
               << "random_seed: " << result.random_seed << "\n"
               << "data_source: " << result.data_source << "\n"
               << "validity_label: " << result.validity_label << "\n"
               << "model_cloud_topic: " << (use_model_cloud_topic ? "true" : "false") << "\n"
               << "visible_model_topic: " << (has_visible_model_cloud_ ? "true" : "false") << "\n"
               << "model_point_count: " << (has_model_cloud
                    ? model_cloud.width * std::max(1U, model_cloud.height)
                    : 0) << "\n"
               << "ground_truth_pose_contract: T_base_model_identity_virtual_only\n"
               << "planning_cost_model: euclidean_viewpoint_proxy\n"
               << "uncertainty_model: "
               << this->get_parameter("uncertainty_model").as_string() << "\n"
               << "observability_model: "
               << this->get_parameter("observability_model").as_string() << "\n"
               << "sensor_noise_seed_contract: "
               << this->get_parameter("sensor_noise_seed_contract").as_string() << "\n"
               << "virtual_initial_translation_bias_m: "
               << this->get_parameter("virtual_initial_translation_bias_m").as_double() << "\n"
               << "virtual_initial_covariance_std_m: "
               << this->get_parameter("virtual_initial_covariance_std_m").as_double() << "\n"
               << "git_commit: " << metadata.git_commit << "\n"
               << "ros_distro: " << metadata.ros_distro << "\n"
               << "covariance_bootstrap_samples: "
               << this->get_parameter("covariance_bootstrap_samples").as_int() << "\n"
               << "launch_profile: " << metadata.launch_profile << "\n";
        logger_->save_config_snapshot(config.str());
        logger_->end_episode(result);
        results_history_.push_back(result);

        // Fill response
        resp->success = (result.stop_reason != cs625_nbv::NbvOrchestrator::ERROR);
        resp->result = result;
        resp->output_directory = output_directory;
        resp->message = resp->success ? "Episode completed" : "Episode failed";

        RCLCPP_INFO(this->get_logger(),
                    "Episode done: %d views, final error=%.4f m, converged=%s",
                    result.total_views, result.final_translation_error,
                    result.converged ? "yes" : "no");
    }

    void handle_get_results(
        const std::shared_ptr<cs625_nbv::srv::GetExperimentResults::Request> req,
        std::shared_ptr<cs625_nbv::srv::GetExperimentResults::Response> resp)
    {
        std::vector<cs625_nbv::msg::EpisodeResult> filtered;
        for (const auto& result : results_history_) {
            if (req->filter_strategy.empty() ||
                result.strategy_name == req->filter_strategy) {
                filtered.push_back(result);
            }
        }
        if (req->count > 0 && static_cast<size_t>(req->count) < filtered.size()) {
            filtered.erase(filtered.begin(), filtered.end() - req->count);
        }
        resp->results = filtered;
        resp->success = true;
    }

    void handle_export_data(
        const std::shared_ptr<cs625_nbv::srv::ExportExperimentData::Request> req,
        std::shared_ptr<cs625_nbv::srv::ExportExperimentData::Response> resp)
    {
        std::vector<cs625_nbv::msg::EpisodeResult> filtered;
        for (const auto& result : results_history_) {
            if (req->filter_strategy.empty() ||
                result.strategy_name == req->filter_strategy) {
                filtered.push_back(result);
            }
        }
        if (req->latest_count > 0 &&
            static_cast<size_t>(req->latest_count) < filtered.size()) {
            filtered.erase(filtered.begin(), filtered.end() - req->latest_count);
        }
        if (filtered.empty()) {
            resp->success = false;
            resp->message = "No matching in-memory experiment results to export";
            return;
        }

        std::string output_path = req->output_path;
        if (output_path.empty()) {
            std::string base_dir = this->get_parameter("log_base_dir").as_string();
            if (!base_dir.empty() && base_dir.front() == '~') {
                const char* home = std::getenv("HOME");
                if (home) base_dir = std::string(home) + base_dir.substr(1);
            }
            auto now = std::time(nullptr);
            auto tm = *std::localtime(&now);
            std::ostringstream filename;
            filename << "report_export_" << std::put_time(&tm, "%Y%m%d_%H%M%S")
                     << ".csv";
            output_path = (
                std::filesystem::path(base_dir) / filename.str()
            ).string();
        }
        const int rows = cs625_nbv::ExperimentLogger::export_results_csv(
            output_path, filtered
        );
        resp->success = rows == static_cast<int>(filtered.size());
        resp->message = resp->success ? "Export completed" : "Export failed";
        resp->output_file = output_path;
        resp->rows_exported = rows;
    }

    // Components
    cs625_nbv::NbvOrchestrator orchestrator_;
    std::unique_ptr<cs625_nbv::ExperimentLogger> logger_;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr model_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr visible_model_cloud_sub_;
    rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;

    // Services
    rclcpp::Service<cs625_nbv::srv::RunNbvEpisode>::SharedPtr run_episode_srv_;
    rclcpp::Service<cs625_nbv::srv::GetExperimentResults>::SharedPtr get_results_srv_;
    rclcpp::Service<cs625_nbv::srv::ExportExperimentData>::SharedPtr export_data_srv_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr virtual_camera_pose_pub_;

    // Latest data
    sensor_msgs::msg::PointCloud2 latest_cloud_;
    sensor_msgs::msg::JointState latest_joints_;
    sensor_msgs::msg::PointCloud2 latest_model_cloud_;
    sensor_msgs::msg::PointCloud2 latest_visible_model_cloud_;
    std::mutex sensor_mutex_;
    bool has_cloud_{false};
    bool has_model_cloud_{false};
    bool has_visible_model_cloud_{false};
    bool model_cloud_initialized_{false};
    std::vector<cs625_nbv::msg::EpisodeResult> results_history_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NbvServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
