#include <nav_msgs/msg/path.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/future_return_code.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include "cs625_trajectory_tools/srv/get_named_target.hpp"
#include "cs625_trajectory_tools/srv/plan_to_pose.hpp"
#include "cs625_trajectory_tools/srv/plan_to_target.hpp"
#include "cs625_trajectory_tools/srv/execute_last_plan.hpp"
#include "cs625_state_monitor/msg/cs625_state.hpp"

#include "pointcloud_workflow_interfaces/srv/start_sampling.hpp"
#include "pointcloud_workflow_interfaces/srv/stop_sampling.hpp"
#include "pointcloud_workflow_interfaces/srv/preview_sampling_path.hpp"
#include "pointcloud_workflow_interfaces/srv/get_sampling_status.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include <mutex>
#include <atomic>
#include <memory>
#include <cctype>
#include <cstdint>

using namespace std::chrono_literals;

class WorkspacePointcloudSamplerNode : public rclcpp::Node
{
public:
  WorkspacePointcloudSamplerNode()
  : Node("workspace_pointcloud_sampler")
  {
    declareAndLoadParameters();
    sanitizeParameters();

    initHelperNode();

    state_sub_ = this->create_subscription<cs625_state_monitor::msg::CS625State>(
      "/cs625/raw_state",
      50,
      std::bind(&WorkspacePointcloudSamplerNode::stateCallback, this, std::placeholders::_1));

    planned_tcp_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/cs625/planned_tcp_path",
      10,
      std::bind(&WorkspacePointcloudSamplerNode::plannedTcpPathCallback, this, std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    start_sampling_service_ =
      this->create_service<pointcloud_workflow_interfaces::srv::StartSampling>(
        "~/start_sampling",
        std::bind(
          &WorkspacePointcloudSamplerNode::handleStartSampling,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    stop_sampling_service_ =
      this->create_service<pointcloud_workflow_interfaces::srv::StopSampling>(
        "~/stop_sampling",
        std::bind(
          &WorkspacePointcloudSamplerNode::handleStopSampling,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    preview_sampling_path_service_ =
      this->create_service<pointcloud_workflow_interfaces::srv::PreviewSamplingPath>(
        "~/preview_sampling_path",
        std::bind(
          &WorkspacePointcloudSamplerNode::handlePreviewSamplingPath,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    get_sampling_status_service_ =
      this->create_service<pointcloud_workflow_interfaces::srv::GetSamplingStatus>(
        "~/get_sampling_status",
        std::bind(
          &WorkspacePointcloudSamplerNode::handleGetSamplingStatus,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    setState("IDLE");
    setProgress(-1, 0);

    logCurrentConfiguration();

    RCLCPP_INFO(
      this->get_logger(),
      "WorkspacePointcloudSamplerNode initialized. Waiting for service-triggered execution.");
  }

  ~WorkspacePointcloudSamplerNode() override
  {
    requestStop();
    joinWorkerIfNeeded();
    stopHelperNode();
  }

private:
  struct TcpState
  {
    double x_mm{0.0};
    double y_mm{0.0};
    double z_mm{0.0};
    double rx_deg{0.0};
    double ry_deg{0.0};
    double rz_deg{0.0};
  };

  struct QuantizedTcpState
  {
    double x_mm{0.0};
    double y_mm{0.0};
    double z_mm{0.0};
    double rx_deg{0.0};
    double ry_deg{0.0};
    double rz_deg{0.0};

    bool operator==(const QuantizedTcpState & other) const
    {
      return x_mm == other.x_mm &&
             y_mm == other.y_mm &&
             z_mm == other.z_mm &&
             rx_deg == other.rx_deg &&
             ry_deg == other.ry_deg &&
             rz_deg == other.rz_deg;
    }
  };

  struct NamedTarget
  {
    geometry_msgs::msg::PoseStamped pose;
    sensor_msgs::msg::JointState joint_state;
  };

  void initHelperNode()
  {
    helper_node_ = std::make_shared<rclcpp::Node>("workspace_pointcloud_sampler_helper");

    helper_get_named_target_client_ =
      helper_node_->create_client<cs625_trajectory_tools::srv::GetNamedTarget>(
        "/cs625/get_named_target");

    helper_plan_to_pose_client_ =
      helper_node_->create_client<cs625_trajectory_tools::srv::PlanToPose>(
        "/cs625/plan_to_pose_srv");

    helper_plan_to_target_client_ =
      helper_node_->create_client<cs625_trajectory_tools::srv::PlanToTarget>(
        "/cs625/plan_to_target_srv");

    helper_execute_last_plan_client_ =
      helper_node_->create_client<cs625_trajectory_tools::srv::ExecuteLastPlan>(
        "/cs625/execute_last_plan");

    helper_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    helper_executor_->add_node(helper_node_);
  }

  void stopHelperNode()
  {
    if (helper_executor_ && helper_node_) {
      helper_executor_->remove_node(helper_node_);
    }

    helper_execute_last_plan_client_.reset();
    helper_plan_to_target_client_.reset();
    helper_plan_to_pose_client_.reset();
    helper_get_named_target_client_.reset();
    helper_node_.reset();
    helper_executor_.reset();
  }

  void declareAndLoadParameters()
  {
    start_pose_name_ = this->declare_parameter<std::string>(
      "start_pose_name", "box_rough_capture_tcp");
    end_pose_name_ = this->declare_parameter<std::string>(
      "end_pose_name", "slot_rough_capture_tcp");

    sampling_segment_count_ = this->declare_parameter<int>(
      "sampling_segment_count", 5);
    stable_required_count_ = this->declare_parameter<int>(
      "stable_required_count", 5);
    stable_decimal_places_ = this->declare_parameter<int>(
      "stable_decimal_places", 3);

    pcd_root_directory_ = this->declare_parameter<std::string>(
      "pcd_root_directory", "/home/yff/environment_point_cloud");
    pcd_filename_ = this->declare_parameter<std::string>(
      "pcd_filename", "environment_point_cloud.pcd");
    pcd_wait_timeout_sec_ = this->declare_parameter<double>(
      "pcd_wait_timeout_sec", 60.0);
    pcd_poll_interval_ms_ = this->declare_parameter<int>(
      "pcd_poll_interval_ms", 500);

    trigger_ip_ = this->declare_parameter<std::string>(
      "trigger_ip", "192.168.1.100");
    trigger_port_ = this->declare_parameter<int>(
      "trigger_port", 7000);
    trigger_header_ = this->declare_parameter<std::string>(
      "trigger_header", "E1");

    trigger_base_frame_ = this->declare_parameter<std::string>(
      "trigger_base_frame", "base_link");
    trigger_tool_frame_ = this->declare_parameter<std::string>(
      "trigger_tool_frame", "flange");
    post_trigger_wait_sec_ = this->declare_parameter<double>(
      "post_trigger_wait_sec", 2.0);

    position_arrival_tolerance_mm_ = this->declare_parameter<double>(
      "position_arrival_tolerance_mm", 5.0);
    startup_delay_sec_ = this->declare_parameter<double>(
      "startup_delay_sec", 2.0);
  }

  void reloadRuntimeParameters()
  {
    this->get_parameter("start_pose_name", start_pose_name_);
    this->get_parameter("end_pose_name", end_pose_name_);
    this->get_parameter("sampling_segment_count", sampling_segment_count_);
    this->get_parameter("stable_required_count", stable_required_count_);
    this->get_parameter("stable_decimal_places", stable_decimal_places_);
    this->get_parameter("pcd_root_directory", pcd_root_directory_);
    this->get_parameter("pcd_filename", pcd_filename_);
    this->get_parameter("pcd_wait_timeout_sec", pcd_wait_timeout_sec_);
    this->get_parameter("pcd_poll_interval_ms", pcd_poll_interval_ms_);
    this->get_parameter("trigger_ip", trigger_ip_);
    this->get_parameter("trigger_port", trigger_port_);
    this->get_parameter("trigger_header", trigger_header_);
    this->get_parameter("trigger_base_frame", trigger_base_frame_);
    this->get_parameter("trigger_tool_frame", trigger_tool_frame_);
    this->get_parameter("post_trigger_wait_sec", post_trigger_wait_sec_);
    this->get_parameter("position_arrival_tolerance_mm", position_arrival_tolerance_mm_);
    this->get_parameter("startup_delay_sec", startup_delay_sec_);

    sanitizeParameters();
  }

  void sanitizeParameters()
  {
    if (sampling_segment_count_ <= 0) {
      sampling_segment_count_ = 5;
    }
    if (stable_required_count_ <= 0) {
      stable_required_count_ = 5;
    }
    if (stable_decimal_places_ < 0) {
      stable_decimal_places_ = 3;
    }
    if (pcd_wait_timeout_sec_ <= 0.0) {
      pcd_wait_timeout_sec_ = 60.0;
    }
    if (pcd_poll_interval_ms_ <= 0) {
      pcd_poll_interval_ms_ = 500;
    }
    if (position_arrival_tolerance_mm_ <= 0.0) {
      position_arrival_tolerance_mm_ = 5.0;
    }
    if (post_trigger_wait_sec_ < 0.0) {
      post_trigger_wait_sec_ = 0.0;
    }
    if (startup_delay_sec_ < 0.0) {
      startup_delay_sec_ = 0.0;
    }
  }

  void logCurrentConfiguration() const
  {
    RCLCPP_INFO(this->get_logger(), "===== Workspace Pointcloud Sampler Config =====");
    RCLCPP_INFO(this->get_logger(), "start_pose_name: %s", start_pose_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "end_pose_name: %s", end_pose_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "sampling_segment_count: %d", sampling_segment_count_);
    RCLCPP_INFO(this->get_logger(), "stable_required_count: %d", stable_required_count_);
    RCLCPP_INFO(this->get_logger(), "stable_decimal_places: %d", stable_decimal_places_);
    RCLCPP_INFO(this->get_logger(), "pcd_root_directory: %s", pcd_root_directory_.c_str());
    RCLCPP_INFO(this->get_logger(), "pcd_filename: %s", pcd_filename_.c_str());
    RCLCPP_INFO(this->get_logger(), "pcd_wait_timeout_sec: %.3f", pcd_wait_timeout_sec_);
    RCLCPP_INFO(this->get_logger(), "pcd_poll_interval_ms: %d", pcd_poll_interval_ms_);
    RCLCPP_INFO(this->get_logger(), "trigger_ip: %s", trigger_ip_.c_str());
    RCLCPP_INFO(this->get_logger(), "trigger_port: %d", trigger_port_);
    RCLCPP_INFO(this->get_logger(), "trigger_header: %s", trigger_header_.c_str());
    RCLCPP_INFO(this->get_logger(), "trigger_base_frame: %s", trigger_base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "trigger_tool_frame: %s", trigger_tool_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "post_trigger_wait_sec: %.3f", post_trigger_wait_sec_);
    RCLCPP_INFO(this->get_logger(), "position_arrival_tolerance_mm: %.3f", position_arrival_tolerance_mm_);
    RCLCPP_INFO(this->get_logger(), "startup_delay_sec: %.3f", startup_delay_sec_);
    RCLCPP_INFO(this->get_logger(), "================================================");
  }

  std::string buildNamedTargetMissingMessage(const std::string & target_name) const
  {
    std::ostringstream oss;
    oss << "Named target '" << target_name << "' is not available. "
        << "Please record it first in GoalPlannerPanel";

    if (target_name == "box_rough_capture_tcp") {
      oss << " using button '记录当前为 box_rough_capture_tcp'.";
    } else if (target_name == "slot_rough_capture_tcp") {
      oss << " using button '记录当前为 slot_rough_capture_tcp'.";
    } else {
      oss << ".";
    }

    return oss.str();
  }

  void setState(const std::string & new_state)
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    state_ = new_state;
  }

  std::string getState() const
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    return state_;
  }

  void setProgress(int current_index, int total_count)
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    current_sample_index_ = current_index;
    total_sample_count_ = total_count;
  }

  void requestStop()
  {
    stop_requested_ = true;
  }

  void clearStopRequest()
  {
    stop_requested_ = false;
  }

  bool isRunning() const
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    return running_;
  }

  void setRunning(bool value)
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    running_ = value;
  }

  void joinWorkerIfNeeded()
  {
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

  template<typename FutureT>
  bool spinUntilHelperFutureComplete(
    FutureT & future,
    std::chrono::milliseconds timeout,
    bool stop_sensitive,
    const std::string & context)
  {
    if (!helper_executor_) {
      RCLCPP_ERROR(this->get_logger(), "Helper executor is not initialized for %s.", context.c_str());
      return false;
    }

    const auto start = std::chrono::steady_clock::now();
    const auto slice = 100ms;

    while (rclcpp::ok()) {
      if (stop_sensitive && stop_requested_) {
        RCLCPP_WARN(this->get_logger(), "%s interrupted by stop request.", context.c_str());
        return false;
      }

      const auto elapsed = std::chrono::steady_clock::now() - start;
      if (elapsed >= timeout) {
        return false;
      }

      const auto remaining =
        std::chrono::duration_cast<std::chrono::milliseconds>(timeout - elapsed);
      const auto this_wait = std::min(slice, remaining);

      auto ret = helper_executor_->spin_until_future_complete(future, this_wait);

      if (ret == rclcpp::FutureReturnCode::SUCCESS) {
        return true;
      }

      if (ret == rclcpp::FutureReturnCode::INTERRUPTED) {
        if (!rclcpp::ok()) {
          return false;
        }
      }
    }

    return false;
  }

  void plannedTcpPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    std::lock_guard<std::mutex> lock(planned_path_mutex_);
    latest_planned_tcp_path_ = *msg;
    has_latest_planned_tcp_path_ = !msg->poses.empty();
    ++planned_path_version_;
  }

  void handleStartSampling(
    const std::shared_ptr<pointcloud_workflow_interfaces::srv::StartSampling::Request> /*request*/,
    std::shared_ptr<pointcloud_workflow_interfaces::srv::StartSampling::Response> response)
  {
    if (isRunning()) {
      response->accepted = false;
      response->success = false;
      response->message = "Sampling task is already running.";
      return;
    }

    joinWorkerIfNeeded();
    reloadRuntimeParameters();
    logCurrentConfiguration();
    clearStopRequest();
    setRunning(true);
    setState("RUNNING");
    setProgress(0, 0);

    worker_thread_ = std::thread([this]() {
      this->runSamplingTask();
    });

    response->accepted = true;
    response->success = true;
    response->message = "Sampling task started.";
  }

  void handleStopSampling(
    const std::shared_ptr<pointcloud_workflow_interfaces::srv::StopSampling::Request> /*request*/,
    std::shared_ptr<pointcloud_workflow_interfaces::srv::StopSampling::Response> response)
  {
    if (!isRunning()) {
      response->accepted = false;
      response->success = false;
      response->message = "No running sampling task.";
      return;
    }

    setState("STOPPING");
    requestStop();

    response->accepted = true;
    response->success = true;
    response->message = "Stop requested.";
  }

  void handlePreviewSamplingPath(
    const std::shared_ptr<pointcloud_workflow_interfaces::srv::PreviewSamplingPath::Request> /*request*/,
    std::shared_ptr<pointcloud_workflow_interfaces::srv::PreviewSamplingPath::Response> response)
  {
    reloadRuntimeParameters();

    auto samples_opt = buildPreviewSamplingPath();
    if (!samples_opt.has_value()) {
      response->success = false;
      response->message = last_error_message_;
      return;
    }

    response->success = true;
    response->message = "Preview sampling path generated successfully.";
    response->poses = *samples_opt;
  }

  void handleGetSamplingStatus(
    const std::shared_ptr<pointcloud_workflow_interfaces::srv::GetSamplingStatus::Request> /*request*/,
    std::shared_ptr<pointcloud_workflow_interfaces::srv::GetSamplingStatus::Response> response)
  {
    response->success = true;
    response->message = "Sampling status fetched successfully.";
    response->state = getState();

    {
      std::lock_guard<std::mutex> lock(runtime_mutex_);
      response->current_index = current_sample_index_;
      response->total_count = total_sample_count_;
      response->running = running_;
      response->stop_requested = stop_requested_.load();
    }
  }

  bool waitForServices()
  {
    const auto timeout = 10s;

    if (!helper_get_named_target_client_ ||
        !helper_plan_to_pose_client_ ||
        !helper_plan_to_target_client_ ||
        !helper_execute_last_plan_client_)
    {
      RCLCPP_ERROR(this->get_logger(), "Helper clients are not initialized.");
      return false;
    }

    if (!helper_get_named_target_client_->wait_for_service(timeout)) {
      RCLCPP_ERROR(this->get_logger(), "Service /cs625/get_named_target not available.");
      return false;
    }

    if (!helper_plan_to_pose_client_->wait_for_service(timeout)) {
      RCLCPP_ERROR(this->get_logger(), "Service /cs625/plan_to_pose_srv not available.");
      return false;
    }

    if (!helper_plan_to_target_client_->wait_for_service(timeout)) {
      RCLCPP_ERROR(this->get_logger(), "Service /cs625/plan_to_target_srv not available.");
      return false;
    }

    if (!helper_execute_last_plan_client_->wait_for_service(timeout)) {
      RCLCPP_ERROR(this->get_logger(), "Service /cs625/execute_last_plan not available.");
      return false;
    }

    return true;
  }

  bool waitForStateReady(double timeout_sec)
  {
    auto start = this->now();
    rclcpp::Rate rate(20.0);

    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (has_state_) {
          return true;
        }
      }

      if ((this->now() - start).seconds() > timeout_sec) {
        return false;
      }

      rate.sleep();
    }

    return false;
  }

  void stateCallback(const cs625_state_monitor::msg::CS625State::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    TcpState s;
    s.x_mm = msg->actual_tcp_x_mm;
    s.y_mm = msg->actual_tcp_y_mm;
    s.z_mm = msg->actual_tcp_z_mm;
    s.rx_deg = msg->actual_rot_x_deg;
    s.ry_deg = msg->actual_rot_y_deg;
    s.rz_deg = msg->actual_rot_z_deg;

    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_ = s;
    has_state_ = true;
  }

  TcpState getLatestStateCopy()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_state_;
  }

  std::optional<NamedTarget> getNamedTarget(const std::string & name)
  {
    if (!helper_get_named_target_client_) {
      last_error_message_ = "Helper GetNamedTarget client is not initialized.";
      RCLCPP_ERROR(this->get_logger(), "%s", last_error_message_.c_str());
      return std::nullopt;
    }

    auto request =
      std::make_shared<cs625_trajectory_tools::srv::GetNamedTarget::Request>();
    request->name = name;

    auto future = helper_get_named_target_client_->async_send_request(request);

    if (!spinUntilHelperFutureComplete(
          future, 10s, false, "GetNamedTarget"))
    {
      last_error_message_ =
        "GetNamedTarget service call timeout for '" + name + "'.";
      RCLCPP_ERROR(this->get_logger(), "%s", last_error_message_.c_str());
      return std::nullopt;
    }

    auto resp = future.get();
    if (!resp->success) {
      const std::string backend_msg = resp->message;
      if (backend_msg.find("not found") != std::string::npos ||
          backend_msg.find("Not found") != std::string::npos ||
          backend_msg.find("NOT_FOUND") != std::string::npos)
      {
        last_error_message_ = buildNamedTargetMissingMessage(name);
      } else {
        std::ostringstream oss;
        oss << "GetNamedTarget failed for '" << name << "': " << backend_msg;
        last_error_message_ = oss.str();
      }

      RCLCPP_ERROR(this->get_logger(), "%s", last_error_message_.c_str());
      return std::nullopt;
    }

    NamedTarget target;
    target.pose = resp->pose;
    target.joint_state = resp->joint_state;

    RCLCPP_INFO(this->get_logger(),
                "Loaded named target '%s'.", name.c_str());
    return target;
  }

  bool planToPose(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!helper_plan_to_pose_client_) {
      RCLCPP_ERROR(this->get_logger(), "Helper PlanToPose client is not initialized.");
      return false;
    }

    auto request =
      std::make_shared<cs625_trajectory_tools::srv::PlanToPose::Request>();
    request->target_pose = pose;

    auto future = helper_plan_to_pose_client_->async_send_request(request);

    if (!spinUntilHelperFutureComplete(
          future, 20s, true, "PlanToPose"))
    {
      if (stop_requested_) {
        RCLCPP_WARN(this->get_logger(), "PlanToPose interrupted by stop request.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "PlanToPose service call timeout.");
      }
      return false;
    }

    auto resp = future.get();
    if (!resp->success) {
      RCLCPP_ERROR(this->get_logger(),
                   "PlanToPose failed: %s", resp->message.c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "PlanToPose succeeded. planning_time=%.3f sec",
                resp->planning_time);
    return true;
  }

  bool planToTarget(const NamedTarget & target)
  {
    if (!helper_plan_to_target_client_) {
      RCLCPP_ERROR(this->get_logger(), "Helper PlanToTarget client is not initialized.");
      return false;
    }

    auto request =
      std::make_shared<cs625_trajectory_tools::srv::PlanToTarget::Request>();
    request->target_pose = target.pose;
    request->target_joint_state = target.joint_state;
    request->use_joint_target = true;

    auto future = helper_plan_to_target_client_->async_send_request(request);

    if (!spinUntilHelperFutureComplete(
          future, 20s, true, "PlanToTarget"))
    {
      if (stop_requested_) {
        RCLCPP_WARN(this->get_logger(), "PlanToTarget interrupted by stop request.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "PlanToTarget service call timeout.");
      }
      return false;
    }

    auto resp = future.get();
    if (!resp->success) {
      RCLCPP_ERROR(this->get_logger(),
                   "PlanToTarget failed: %s", resp->message.c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "PlanToTarget succeeded. planning_time=%.3f sec",
                resp->planning_time);
    return true;
  }

  bool executeLastPlan()
  {
    if (!helper_execute_last_plan_client_) {
      RCLCPP_ERROR(this->get_logger(), "Helper ExecuteLastPlan client is not initialized.");
      return false;
    }

    auto request =
      std::make_shared<cs625_trajectory_tools::srv::ExecuteLastPlan::Request>();

    auto future = helper_execute_last_plan_client_->async_send_request(request);

    if (!spinUntilHelperFutureComplete(
          future, 60s, true, "ExecuteLastPlan"))
    {
      if (stop_requested_) {
        RCLCPP_WARN(this->get_logger(), "ExecuteLastPlan interrupted by stop request.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "ExecuteLastPlan service call timeout.");
      }
      return false;
    }

    auto resp = future.get();
    if (!resp->success) {
      RCLCPP_ERROR(this->get_logger(),
                   "ExecuteLastPlan failed: %s", resp->message.c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "ExecuteLastPlan succeeded.");
    return true;
  }

  double positionDistanceMm(
    const TcpState & state,
    const geometry_msgs::msg::Pose & pose) const
  {
    const double px_mm = pose.position.x * 1000.0;
    const double py_mm = pose.position.y * 1000.0;
    const double pz_mm = pose.position.z * 1000.0;

    const double dx = state.x_mm - px_mm;
    const double dy = state.y_mm - py_mm;
    const double dz = state.z_mm - pz_mm;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  std::vector<geometry_msgs::msg::PoseStamped> samplePathByIndex(
    const nav_msgs::msg::Path & path,
    int segment_count)
  {
    std::vector<geometry_msgs::msg::PoseStamped> result;

    if (path.poses.empty()) {
      return result;
    }

    const int sample_count = std::max(2, segment_count + 1);
    const int n = static_cast<int>(path.poses.size());

    if (n == 1) {
      result.push_back(path.poses.front());
      return result;
    }

    result.reserve(sample_count);

    for (int j = 0; j < sample_count; ++j) {
      const double alpha =
        static_cast<double>(j) / static_cast<double>(sample_count - 1);
      int idx = static_cast<int>(std::round(alpha * static_cast<double>(n - 1)));
      idx = std::clamp(idx, 0, n - 1);
      result.push_back(path.poses[static_cast<std::size_t>(idx)]);
    }

    return result;
  }

  std::uint64_t getPlannedPathVersion()
  {
    std::lock_guard<std::mutex> lock(planned_path_mutex_);
    return planned_path_version_;
  }

  std::optional<nav_msgs::msg::Path> getLatestPlannedTcpPathCopy()
  {
    std::lock_guard<std::mutex> lock(planned_path_mutex_);
    if (!has_latest_planned_tcp_path_ || latest_planned_tcp_path_.poses.empty()) {
      return std::nullopt;
    }
    return latest_planned_tcp_path_;
  }

  std::optional<nav_msgs::msg::Path> waitForPlannedTcpPathAfterVersion(
    std::uint64_t previous_version,
    double timeout_sec,
    bool stop_sensitive,
    bool allow_cached_fallback,
    const std::string & log_context)
  {
    rclcpp::Rate rate(50.0);
    auto start = this->now();

    while (rclcpp::ok()) {
      if (stop_sensitive && stop_requested_) {
        return std::nullopt;
      }

      {
        std::lock_guard<std::mutex> lock(planned_path_mutex_);
        if (has_latest_planned_tcp_path_ &&
            !latest_planned_tcp_path_.poses.empty() &&
            planned_path_version_ > previous_version)
        {
          RCLCPP_INFO(
            this->get_logger(),
            "%s: received new /cs625/planned_tcp_path version=%llu with %zu poses.",
            log_context.c_str(),
            static_cast<unsigned long long>(planned_path_version_),
            latest_planned_tcp_path_.poses.size());
          return latest_planned_tcp_path_;
        }
      }

      if ((this->now() - start).seconds() > timeout_sec) {
        if (allow_cached_fallback) {
          auto cached_path = getLatestPlannedTcpPathCopy();
          if (cached_path.has_value()) {
            RCLCPP_WARN(
              this->get_logger(),
              "%s: timeout waiting for new /cs625/planned_tcp_path, fallback to latest cached path with %zu poses.",
              log_context.c_str(),
              cached_path->poses.size());
            return cached_path;
          }
        }
        return std::nullopt;
      }

      rate.sleep();
    }

    return std::nullopt;
  }

  std::optional<std::vector<geometry_msgs::msg::PoseStamped>> planAndSampleTcpPathToNamedTarget(
    const NamedTarget & target,
    double path_timeout_sec,
    const std::string & log_context,
    bool stop_sensitive,
    bool allow_cached_path_fallback)
  {
    const auto before_version = getPlannedPathVersion();

    if (!planToTarget(target)) {
      last_error_message_ = log_context + ": planToTarget failed.";
      RCLCPP_ERROR(this->get_logger(), "%s", last_error_message_.c_str());
      return std::nullopt;
    }

    auto path_opt = waitForPlannedTcpPathAfterVersion(
      before_version,
      path_timeout_sec,
      stop_sensitive,
      allow_cached_path_fallback,
      log_context);

    if (!path_opt.has_value()) {
      if (stop_sensitive && stop_requested_) {
        last_error_message_ = log_context + ": interrupted by stop request.";
      } else {
        last_error_message_ = log_context + ": timed out waiting for /cs625/planned_tcp_path.";
      }
      RCLCPP_ERROR(this->get_logger(), "%s", last_error_message_.c_str());
      return std::nullopt;
    }

    auto samples = samplePathByIndex(*path_opt, sampling_segment_count_);
    if (samples.empty()) {
      last_error_message_ = log_context + ": sampled poses are empty.";
      RCLCPP_ERROR(this->get_logger(), "%s", last_error_message_.c_str());
      return std::nullopt;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "%s: planned tcp path points=%zu, sampled poses=%zu",
      log_context.c_str(),
      path_opt->poses.size(),
      samples.size());

    return samples;
  }


  std::optional<std::vector<geometry_msgs::msg::PoseStamped>> buildPreviewSamplingPath()
  {
    last_error_message_.clear();

    if (!waitForStateReady(10.0)) {
      last_error_message_ = "No /cs625/raw_state received within timeout.";
      RCLCPP_ERROR(this->get_logger(), "%s", last_error_message_.c_str());
      return std::nullopt;
    }

    if (!waitForServices()) {
      last_error_message_ = "Required services are not ready.";
      RCLCPP_ERROR(this->get_logger(), "%s", last_error_message_.c_str());
      return std::nullopt;
    }

    auto target_a_opt = getNamedTarget(start_pose_name_);
    if (!target_a_opt) {
      if (last_error_message_.empty()) {
        last_error_message_ = buildNamedTargetMissingMessage(start_pose_name_);
      }
      RCLCPP_ERROR(this->get_logger(), "%s", last_error_message_.c_str());
      return std::nullopt;
    }

    auto target_b_opt = getNamedTarget(end_pose_name_);
    if (!target_b_opt) {
      if (last_error_message_.empty()) {
        last_error_message_ = buildNamedTargetMissingMessage(end_pose_name_);
      }
      RCLCPP_ERROR(this->get_logger(), "%s", last_error_message_.c_str());
      return std::nullopt;
    }

    auto target_a = *target_a_opt;
    auto target_b = *target_b_opt;

    TcpState current_state = getLatestStateCopy();
    const double dist_to_a = positionDistanceMm(current_state, target_a.pose.pose);
    const double dist_to_b = positionDistanceMm(current_state, target_b.pose.pose);

    NamedTarget preview_target;
    

    if (dist_to_a <= dist_to_b) {
      preview_target = target_b;
      
      RCLCPP_INFO(
        this->get_logger(),
        "Preview logic: current TCP is closer to '%s', planning logically toward named target '%s' with joint constraint.",
        start_pose_name_.c_str(),
        end_pose_name_.c_str());
    } else {
      preview_target = target_a;
      
      RCLCPP_INFO(
        this->get_logger(),
        "Preview logic: current TCP is closer to '%s', planning logically toward named target '%s' with joint constraint.",
        end_pose_name_.c_str(),
        start_pose_name_.c_str());
    }

    return planAndSampleTcpPathToNamedTarget(
      preview_target,
      5.0,
      "PreviewSamplingPath",
      false,
      true);
  }
  std::optional<std::vector<geometry_msgs::msg::PoseStamped>> buildRuntimeSamplingPath()
  {
    last_error_message_.clear();

    if (!waitForStateReady(10.0)) {
      last_error_message_ = "No /cs625/raw_state received within timeout.";
      RCLCPP_ERROR(this->get_logger(), "%s", last_error_message_.c_str());
      return std::nullopt;
    }

    if (!waitForServices()) {
      last_error_message_ = "Required services are not ready.";
      RCLCPP_ERROR(this->get_logger(), "%s", last_error_message_.c_str());
      return std::nullopt;
    }

    auto target_a_opt = getNamedTarget(start_pose_name_);
    if (!target_a_opt) {
      if (last_error_message_.empty()) {
        last_error_message_ = buildNamedTargetMissingMessage(start_pose_name_);
      }
      RCLCPP_ERROR(this->get_logger(), "%s", last_error_message_.c_str());
      return std::nullopt;
    }

    auto target_b_opt = getNamedTarget(end_pose_name_);
    if (!target_b_opt) {
      if (last_error_message_.empty()) {
        last_error_message_ = buildNamedTargetMissingMessage(end_pose_name_);
      }
      RCLCPP_ERROR(this->get_logger(), "%s", last_error_message_.c_str());
      return std::nullopt;
    }

    auto target_a = *target_a_opt;
    auto target_b = *target_b_opt;

    TcpState current_state = getLatestStateCopy();
    const double dist_to_a = positionDistanceMm(current_state, target_a.pose.pose);
    const double dist_to_b = positionDistanceMm(current_state, target_b.pose.pose);

    NamedTarget effective_start;
    NamedTarget effective_end;
    std::string effective_start_name;
    std::string effective_end_name;

    if (dist_to_a <= dist_to_b) {
      effective_start = target_a;
      effective_end = target_b;
      effective_start_name = start_pose_name_;
      effective_end_name = end_pose_name_;
    } else {
      effective_start = target_b;
      effective_end = target_a;
      effective_start_name = end_pose_name_;
      effective_end_name = start_pose_name_;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Runtime sampling direction: %s -> %s",
      effective_start_name.c_str(),
      effective_end_name.c_str());

    if (!planToTarget(effective_start)) {
      last_error_message_ = "Failed to plan to effective start target.";
      RCLCPP_ERROR(this->get_logger(), "%s", last_error_message_.c_str());
      return std::nullopt;
    }

    if (!executeLastPlan()) {
      last_error_message_ = "Failed to execute motion to effective start target.";
      RCLCPP_ERROR(this->get_logger(), "%s", last_error_message_.c_str());
      return std::nullopt;
    }

    if (!waitUntilStable(30.0)) {
      last_error_message_ = "TCP did not become stable after moving to effective start target.";
      RCLCPP_ERROR(this->get_logger(), "%s", last_error_message_.c_str());
      return std::nullopt;
    }

    return planAndSampleTcpPathToNamedTarget(
      effective_end,
      5.0,
      "RuntimeSamplingPath",
      true,
      false);
  }

  void runSamplingTask()
  {
    bool success = false;

    if (startup_delay_sec_ > 0.0) {
      RCLCPP_INFO(this->get_logger(),
                  "Startup delay %.2f sec ...", startup_delay_sec_);
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(startup_delay_sec_)));
    }

    auto samples_opt = buildRuntimeSamplingPath();
    if (!samples_opt.has_value()) {
      setState(stop_requested_ ? "STOPPED" : "ERROR");
      setRunning(false);
      return;
    }

    auto samples = *samples_opt;
    setProgress(0, static_cast<int>(samples.size()));

    for (std::size_t i = 0; i < samples.size(); ++i) {
      if (stop_requested_) {
        RCLCPP_WARN(this->get_logger(), "Stop requested, exiting worker thread.");
        setState("STOPPED");
        setRunning(false);
        return;
      }

      setProgress(static_cast<int>(i), static_cast<int>(samples.size()));

      RCLCPP_INFO(this->get_logger(),
                  "==== Sampling point %zu / %zu ====",
                  i + 1, samples.size());

      if (!planToPose(samples[i])) {
        RCLCPP_ERROR(this->get_logger(),
                     "PlanToPose failed at sample index %zu.", i);
        setState("ERROR");
        setRunning(false);
        return;
      }

      if (!executeLastPlan()) {
        RCLCPP_ERROR(this->get_logger(),
                     "ExecuteLastPlan failed at sample index %zu.", i);
        setState("ERROR");
        setRunning(false);
        return;
      }

      if (!waitUntilStable(30.0)) {
        RCLCPP_ERROR(this->get_logger(),
                     "TCP did not become stable within timeout at sample index %zu.", i);
        setState(stop_requested_ ? "STOPPED" : "ERROR");
        setRunning(false);
        return;
      }

      auto before_dirs = snapshotTimestampDirectories();

      if (!sendTriggerMessageFromTf()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to send trigger message at sample index %zu.", i);
        setState("ERROR");
        setRunning(false);
        return;
      }

      if (post_trigger_wait_sec_ > 0.0) {
        RCLCPP_INFO(this->get_logger(),
                    "Post-trigger wait %.2f sec ...", post_trigger_wait_sec_);
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(post_trigger_wait_sec_)));
      }

      if (!waitForNewPcdCompletion(before_dirs)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed waiting for new PCD completion at sample index %zu.", i);
        setState(stop_requested_ ? "STOPPED" : "ERROR");
        setRunning(false);
        return;
      }

      RCLCPP_INFO(this->get_logger(),
                  "Sampling point %zu completed successfully.", i + 1);
    }

    success = true;

    if (success) {
      setProgress(static_cast<int>(samples.size()), static_cast<int>(samples.size()));
      setState("FINISHED");
    } else {
      setState("ERROR");
    }

    setRunning(false);
  }

  double quantize(double value, int decimals) const
  {
    double scale = std::pow(10.0, decimals);
    return std::round(value * scale) / scale;
  }

  QuantizedTcpState quantizeState(const TcpState & s) const
  {
    QuantizedTcpState q;
    q.x_mm = quantize(s.x_mm, stable_decimal_places_);
    q.y_mm = quantize(s.y_mm, stable_decimal_places_);
    q.z_mm = quantize(s.z_mm, stable_decimal_places_);
    q.rx_deg = quantize(s.rx_deg, stable_decimal_places_);
    q.ry_deg = quantize(s.ry_deg, stable_decimal_places_);
    q.rz_deg = quantize(s.rz_deg, stable_decimal_places_);
    return q;
  }

  bool waitUntilStable(double timeout_sec)
  {
    auto start_time = this->now();
    std::optional<QuantizedTcpState> last_q;
    int same_count = 0;

    rclcpp::Rate rate(20.0);

    while (rclcpp::ok() && !stop_requested_) {
      TcpState current = getLatestStateCopy();
      QuantizedTcpState q = quantizeState(current);

      if (last_q.has_value() && q == last_q.value()) {
        ++same_count;
      } else {
        same_count = 1;
        last_q = q;
      }

      if (same_count >= stable_required_count_) {
        RCLCPP_INFO(this->get_logger(),
                    "TCP stable detected with %d consecutive identical quantized states.",
                    same_count);
        return true;
      }

      if ((this->now() - start_time).seconds() > timeout_sec) {
        RCLCPP_WARN(this->get_logger(),
                    "waitUntilStable timeout after %.2f sec.", timeout_sec);
        return false;
      }

      rate.sleep();
    }

    return false;
  }

  std::set<std::string> snapshotTimestampDirectories() const
  {
    std::set<std::string> names;
    std::filesystem::path root(pcd_root_directory_);

    if (!std::filesystem::exists(root) || !std::filesystem::is_directory(root)) {
      return names;
    }

    for (const auto & entry : std::filesystem::directory_iterator(root)) {
      if (!entry.is_directory()) {
        continue;
      }
      const std::string name = entry.path().filename().string();
      if (isTimestampDirectoryName(name)) {
        names.insert(name);
      }
    }

    return names;
  }

  bool isTimestampDirectoryName(const std::string & name) const
  {
    if (name.empty()) {
      return false;
    }
    return std::all_of(name.begin(), name.end(), [](unsigned char c) {
      return std::isdigit(c) != 0;
    });
  }

  bool isFileReadable(const std::filesystem::path & path) const
  {
    std::ifstream ifs(path, std::ios::binary);
    return ifs.good();
  }

  bool waitForNewPcdCompletion(const std::set<std::string> & before_dirs)
  {
    auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(pcd_wait_timeout_sec_));

    std::optional<std::filesystem::path> detected_pcd;
    std::uintmax_t last_size = 0;
    int stable_size_count = 0;

    while (rclcpp::ok() && !stop_requested_) {
      if (std::chrono::steady_clock::now() > deadline) {
        RCLCPP_ERROR(this->get_logger(),
                     "Timeout waiting for new PCD completion.");
        return false;
      }

      std::filesystem::path root(pcd_root_directory_);
      if (!std::filesystem::exists(root) || !std::filesystem::is_directory(root)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(pcd_poll_interval_ms_));
        continue;
      }

      std::vector<std::string> new_dirs;
      for (const auto & entry : std::filesystem::directory_iterator(root)) {
        if (!entry.is_directory()) {
          continue;
        }

        const std::string dir_name = entry.path().filename().string();
        if (!isTimestampDirectoryName(dir_name)) {
          continue;
        }

        if (before_dirs.find(dir_name) == before_dirs.end()) {
          new_dirs.push_back(dir_name);
        }
      }

      std::sort(new_dirs.begin(), new_dirs.end());

      if (!detected_pcd.has_value() && !new_dirs.empty()) {
        for (const auto & dir_name : new_dirs) {
          std::filesystem::path candidate =
            std::filesystem::path(pcd_root_directory_) / dir_name / pcd_filename_;
          if (std::filesystem::exists(candidate) &&
              std::filesystem::is_regular_file(candidate))
          {
            detected_pcd = candidate;
            last_size = 0;
            stable_size_count = 0;

            RCLCPP_INFO(this->get_logger(),
                        "Detected new PCD file candidate: %s",
                        candidate.string().c_str());
            break;
          }
        }
      }

      if (detected_pcd.has_value()) {
        if (!std::filesystem::exists(*detected_pcd) ||
            !std::filesystem::is_regular_file(*detected_pcd))
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(pcd_poll_interval_ms_));
          continue;
        }

        auto current_size = std::filesystem::file_size(*detected_pcd);

        if (current_size > 0 && current_size == last_size) {
          ++stable_size_count;
        } else {
          stable_size_count = 0;
        }

        last_size = current_size;

        if (stable_size_count >= 3 && isFileReadable(*detected_pcd)) {
          RCLCPP_INFO(this->get_logger(),
                      "PCD file completed: %s (size=%ju)",
                      detected_pcd->string().c_str(),
                      static_cast<std::uintmax_t>(current_size));
          return true;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(pcd_poll_interval_ms_));
    }

    return false;
  }

  bool getCurrentTriggerPoseFromTf(
    double & x_mm, double & y_mm, double & z_mm,
    double & rx_deg, double & ry_deg, double & rz_deg)
  {
    if (!tf_buffer_) {
      RCLCPP_ERROR(this->get_logger(), "TF buffer is not initialized.");
      return false;
    }

    try {
      geometry_msgs::msg::TransformStamped tf =
        tf_buffer_->lookupTransform(
          trigger_base_frame_,
          trigger_tool_frame_,
          tf2::TimePointZero,
          tf2::durationFromSec(1.0));

      const double x_m = tf.transform.translation.x;
      const double y_m = tf.transform.translation.y;
      const double z_m = tf.transform.translation.z;

      tf2::Quaternion q(
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w);

      double rx_rad = 0.0;
      double ry_rad = 0.0;
      double rz_rad = 0.0;

      tf2::Matrix3x3 m(q);
      m.getRPY(rx_rad, ry_rad, rz_rad);

      x_mm = x_m * 1000.0;
      y_mm = y_m * 1000.0;
      z_mm = z_m * 1000.0;
      rx_deg = rx_rad * 180.0 / M_PI;
      ry_deg = ry_rad * 180.0 / M_PI;
      rz_deg = rz_rad * 180.0 / M_PI;

      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to lookup TF %s -> %s: %s",
                   trigger_base_frame_.c_str(),
                   trigger_tool_frame_.c_str(),
                   ex.what());
      return false;
    }
  }

  bool sendTriggerMessageFromTf()
  {
    double x_mm = 0.0;
    double y_mm = 0.0;
    double z_mm = 0.0;
    double rx_deg = 0.0;
    double ry_deg = 0.0;
    double rz_deg = 0.0;

    if (!getCurrentTriggerPoseFromTf(
          x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg))
    {
      return false;
    }

    std::ostringstream oss;
    oss.setf(std::ios::fixed, std::ios::floatfield);
    oss.precision(3);
    oss << trigger_header_ << ","
        << x_mm << ","
        << y_mm << ","
        << z_mm << ","
        << rx_deg << ","
        << ry_deg << ","
        << rz_deg << "\n";

    const std::string message = oss.str();

    int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create socket.");
      return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(trigger_port_));
    addr.sin_addr.s_addr = inet_addr(trigger_ip_.c_str());

    if (::connect(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to connect to %s:%d",
                   trigger_ip_.c_str(), trigger_port_);
      ::close(sock);
      return false;
    }

    ssize_t n = ::send(sock, message.c_str(), message.size(), 0);
    if (n < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send trigger message.");
      ::shutdown(sock, SHUT_RDWR);
      ::close(sock);
      return false;
    }

    ::shutdown(sock, SHUT_RDWR);
    ::close(sock);

    RCLCPP_INFO(this->get_logger(),
                "Trigger sent (TF %s -> %s): %s",
                trigger_base_frame_.c_str(),
                trigger_tool_frame_.c_str(),
                message.c_str());
    return true;
  }

private:
  std::string start_pose_name_;
  std::string end_pose_name_;

  int sampling_segment_count_;
  int stable_required_count_;
  int stable_decimal_places_;

  std::string pcd_root_directory_;
  std::string pcd_filename_;
  double pcd_wait_timeout_sec_;
  int pcd_poll_interval_ms_;

  std::string trigger_ip_;
  int trigger_port_;
  std::string trigger_header_;

  std::string trigger_base_frame_;
  std::string trigger_tool_frame_;
  double post_trigger_wait_sec_;

  double position_arrival_tolerance_mm_;
  double startup_delay_sec_;

  rclcpp::Node::SharedPtr helper_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> helper_executor_;

  rclcpp::Client<cs625_trajectory_tools::srv::GetNamedTarget>::SharedPtr helper_get_named_target_client_;
  rclcpp::Client<cs625_trajectory_tools::srv::PlanToPose>::SharedPtr helper_plan_to_pose_client_;
  rclcpp::Client<cs625_trajectory_tools::srv::PlanToTarget>::SharedPtr helper_plan_to_target_client_;
  rclcpp::Client<cs625_trajectory_tools::srv::ExecuteLastPlan>::SharedPtr helper_execute_last_plan_client_;

  rclcpp::Subscription<cs625_state_monitor::msg::CS625State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr planned_tcp_path_sub_;

  rclcpp::Service<pointcloud_workflow_interfaces::srv::StartSampling>::SharedPtr
    start_sampling_service_;
  rclcpp::Service<pointcloud_workflow_interfaces::srv::StopSampling>::SharedPtr
    stop_sampling_service_;
  rclcpp::Service<pointcloud_workflow_interfaces::srv::PreviewSamplingPath>::SharedPtr
    preview_sampling_path_service_;
  rclcpp::Service<pointcloud_workflow_interfaces::srv::GetSamplingStatus>::SharedPtr
    get_sampling_status_service_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex state_mutex_;
  TcpState latest_state_;
  bool has_state_{false};

  std::mutex planned_path_mutex_;
  nav_msgs::msg::Path latest_planned_tcp_path_;
  bool has_latest_planned_tcp_path_{false};
  std::uint64_t planned_path_version_{0};

  mutable std::mutex runtime_mutex_;
  std::string state_{"IDLE"};
  bool running_{false};
  int current_sample_index_{-1};
  int total_sample_count_{0};

  std::thread worker_thread_;
  std::atomic_bool stop_requested_{false};

  std::string last_error_message_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WorkspacePointcloudSamplerNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
