#include <memory>
#include <string>
#include <map>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// MoveIt2
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/exceptions.h>

// 自定义 srv
#include "cs625_trajectory_tools/srv/plan_to_frame.hpp"
#include "cs625_trajectory_tools/srv/plan_to_pose.hpp"
#include "cs625_trajectory_tools/srv/plan_to_target.hpp"
#include "cs625_trajectory_tools/srv/execute_last_plan.hpp"
#include "cs625_trajectory_tools/srv/set_named_pose.hpp"
#include "cs625_trajectory_tools/srv/set_named_pose_from_current.hpp"
#include "cs625_trajectory_tools/srv/set_named_target_from_current.hpp"
#include "cs625_trajectory_tools/srv/cartesian_plan_to_frame.hpp"
#include "cs625_trajectory_tools/srv/get_named_pose.hpp"
#include "cs625_trajectory_tools/srv/get_named_target.hpp"
#include "cs625_trajectory_tools/srv/set_named_target.hpp"


#include <array>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "cs625_trajectory_tools/srv/cartesian_force_compensated_to_frame.hpp"

namespace cs625_trajectory_tools
{

class MoveitPlannerClientNode : public rclcpp::Node
{
public:
  MoveitPlannerClientNode()
  : Node("moveit_planner_client_node")
  {
    this->declare_parameter<std::string>("planning_group", "cs625_arm");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("ee_link", "my_end_effector_link");
    this->declare_parameter<std::string>("robot_description", "");
    this->declare_parameter<std::string>("robot_description_semantic", "");
    
    this->declare_parameter<double>("force_comp.step_translation_m", 0.002);
    this->declare_parameter<double>("force_comp.compensation_translation_m", 0.0005);
    this->declare_parameter<double>("force_comp.force_jump_threshold", 8.0);
    this->declare_parameter<double>("force_comp.max_force_axis_abs", 20.0);
    this->declare_parameter<int>("force_comp.max_compensation_count", 3);
    this->declare_parameter<double>("force_comp.goal_tolerance_m", 0.001);
    
    force_comp_step_translation_m_ =
      this->get_parameter("force_comp.step_translation_m").as_double();
    force_comp_compensation_translation_m_ =
      this->get_parameter("force_comp.compensation_translation_m").as_double();
    force_comp_force_jump_threshold_ =
      this->get_parameter("force_comp.force_jump_threshold").as_double();
    force_comp_max_force_axis_abs_ =
      this->get_parameter("force_comp.max_force_axis_abs").as_double();
    force_comp_max_compensation_count_ =
      this->get_parameter("force_comp.max_compensation_count").as_int();
    force_comp_goal_tolerance_m_ =
      this->get_parameter("force_comp.goal_tolerance_m").as_double();

    planning_group_ = this->get_parameter("planning_group").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    ee_link_ = this->get_parameter("ee_link").as_string();

    auto robot_semantic_xml =
      this->get_parameter("robot_description_semantic").as_string();
    if (robot_semantic_xml.empty())
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Parameter 'robot_description_semantic' is empty in MoveitPlannerClientNode.");
    }
    else
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Parameter 'robot_description_semantic' length = %zu.",
        robot_semantic_xml.size());
    }

    RCLCPP_INFO(this->get_logger(), "MoveitPlannerClientNode starting with:");
    RCLCPP_INFO(this->get_logger(), "  planning_group = %s", planning_group_.c_str());
    RCLCPP_INFO(this->get_logger(), "  base_frame     = %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  ee_link        = %s", ee_link_.c_str());

    robot_description_xml_ = this->get_parameter("robot_description").as_string();
    if (robot_description_xml_.empty())
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Parameter 'robot_description' is empty. "
        "MoveGroupInterface may fail to initialize (no URDF).");
    }
    else
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Parameter 'robot_description' length = %zu.",
        robot_description_xml_.size());
    }

    if (!robot_description_xml_.empty())
    {
      RCLCPP_INFO(
        this->get_logger(),
        "MoveitPlannerClientNode received robot_description parameter (length=%zu), "
        "but will not publish it to /robot_description.",
        robot_description_xml_.size());
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    named_pose_tf_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MoveitPlannerClientNode::broadcastAllNamedPosesTF, this));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      10,
      std::bind(&MoveitPlannerClientNode::jointStateCallback, this, std::placeholders::_1));

    tcp_force_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/cs625/tcp_force",
      10,
      std::bind(&MoveitPlannerClientNode::tcpForceCallback, this, std::placeholders::_1));

    joint_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/cs625/planned_joint_trajectory", 10);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cs625/plan_to_pose",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&MoveitPlannerClientNode::planToPoseCallback, this, std::placeholders::_1));

    grasp_state_event_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/cs625/grasp_state_event",
      10,
      std::bind(&MoveitPlannerClientNode::graspStateEventCallback, this, std::placeholders::_1));
      

    plan_to_frame_srv_ = this->create_service<cs625_trajectory_tools::srv::PlanToFrame>(
      "/cs625/plan_to_frame",
      std::bind(&MoveitPlannerClientNode::planToFrameCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    plan_to_pose_srv_ = this->create_service<cs625_trajectory_tools::srv::PlanToPose>(
      "/cs625/plan_to_pose_srv",
      std::bind(&MoveitPlannerClientNode::planToPoseServiceCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    plan_to_target_srv_ = this->create_service<cs625_trajectory_tools::srv::PlanToTarget>(
      "/cs625/plan_to_target_srv",
      std::bind(&MoveitPlannerClientNode::planToTargetServiceCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    execute_last_plan_srv_ =
      this->create_service<cs625_trajectory_tools::srv::ExecuteLastPlan>(
        "/cs625/execute_last_plan",
        std::bind(&MoveitPlannerClientNode::executeLastPlanCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    set_named_pose_srv_ =
      this->create_service<cs625_trajectory_tools::srv::SetNamedPose>(
        "/cs625/set_named_pose",
        std::bind(&MoveitPlannerClientNode::setNamedPoseCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    set_named_pose_from_current_srv_ =
      this->create_service<cs625_trajectory_tools::srv::SetNamedPoseFromCurrent>(
        "/cs625/set_home_from_current",
        std::bind(&MoveitPlannerClientNode::setNamedPoseFromCurrentCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));
                  
    set_named_target_srv_ =
      this->create_service<cs625_trajectory_tools::srv::SetNamedTarget>(
        "/cs625/set_named_target",
        std::bind(&MoveitPlannerClientNode::setNamedTargetCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    set_named_target_from_current_srv_ =
      this->create_service<cs625_trajectory_tools::srv::SetNamedTargetFromCurrent>(
        "/cs625/set_named_target_from_current",
        std::bind(&MoveitPlannerClientNode::setNamedTargetFromCurrentCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    get_named_pose_srv_ =
      this->create_service<cs625_trajectory_tools::srv::GetNamedPose>(
        "/cs625/get_named_pose",
        std::bind(&MoveitPlannerClientNode::getNamedPoseCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    get_named_target_srv_ =
      this->create_service<cs625_trajectory_tools::srv::GetNamedTarget>(
        "/cs625/get_named_target",
        std::bind(&MoveitPlannerClientNode::getNamedTargetCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    cartesian_plan_to_frame_srv_ =
      this->create_service<cs625_trajectory_tools::srv::CartesianPlanToFrame>(
        "/cs625/cartesian_plan_to_frame",
        std::bind(&MoveitPlannerClientNode::cartesianPlanToFrameCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));
                  
    cartesian_force_compensated_to_frame_srv_ =
      this->create_service<cs625_trajectory_tools::srv::CartesianForceCompensatedToFrame>(
        "/cs625/cartesian_force_compensated_to_frame",
        std::bind(&MoveitPlannerClientNode::cartesianForceCompensatedToFrameCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));
                  


    RCLCPP_INFO(
      this->get_logger(),
      "MoveitPlannerClientNode ready: "
      "topic /cs625/plan_to_pose, "
      "service /cs625/plan_to_frame, /cs625/plan_to_pose_srv, "
      "/cs625/plan_to_target_srv, "
      "/cs625/execute_last_plan, "
      "/cs625/set_named_pose, /cs625/set_home_from_current, "
      "/cs625/set_named_target, /cs625/set_named_target_from_current, "
      "/cs625/get_named_pose, /cs625/get_named_target, "
      "/cs625/cartesian_plan_to_frame, "
      "/cs625/cartesian_force_compensated_to_frame, "
      "subscribed /cs625/grasp_state_event, /joint_states and /cs625/tcp_force.");

  
  }
  public:
  bool initialize()
  {
    try
    {
      auto moveit_node = shared_from_this();

      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        moveit_node, planning_group_);

      move_group_->setPoseReferenceFrame(base_frame_);
      if (!ee_link_.empty())
      {
        move_group_->setEndEffectorLink(ee_link_);
      }

      planning_group_joint_names_ = move_group_->getJointNames();

      RCLCPP_INFO(
        this->get_logger(),
        "MoveGroupInterface for group '%s' initialized.",
        planning_group_.c_str());

      if (planning_group_joint_names_.empty())
      {
        RCLCPP_WARN(
          this->get_logger(),
          "MoveGroupInterface returned empty joint list for planning group '%s'.",
          planning_group_.c_str());
      }
      else
      {
        std::stringstream ss;
        ss << "Cached planning-group joint names: ";
        for (size_t i = 0; i < planning_group_joint_names_.size(); ++i)
        {
          ss << planning_group_joint_names_[i];
          if (i + 1 < planning_group_joint_names_.size())
          {
            ss << ", ";
          }
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
      }

      geometry_msgs::msg::PoseStamped home_pose;
      home_pose.header.stamp = this->now();
      home_pose.header.frame_id = base_frame_;
      home_pose.pose.position.x = 0.638685;
      home_pose.pose.position.y = -0.171145;
      home_pose.pose.position.z = 0.551819;
      home_pose.pose.orientation.x = -1.26335e-06;
      home_pose.pose.orientation.y = 0.965926;
      home_pose.pose.orientation.z = 1.26342e-06;
      home_pose.pose.orientation.w = 0.258817;

      named_poses_["home_tcp"] = home_pose;
      broadcastNamedPoseTF("home_tcp", home_pose);

      RCLCPP_INFO(
        this->get_logger(),
        "Initialized default named pose 'home_tcp': "
        "pos=(%.6f, %.6f, %.6f), quat=(%.6f, %.6f, %.6f, %.6f)",
        home_pose.pose.position.x,
        home_pose.pose.position.y,
        home_pose.pose.position.z,
        home_pose.pose.orientation.x,
        home_pose.pose.orientation.y,
        home_pose.pose.orientation.z,
        home_pose.pose.orientation.w);
    
      geometry_msgs::msg::PoseStamped box_place_pose;
      box_place_pose.header.stamp = this->now();
      box_place_pose.header.frame_id = base_frame_;
      box_place_pose.pose.position.x = 0.498655;
      box_place_pose.pose.position.y = 0.616943;
      box_place_pose.pose.position.z = -0.0562479;
      box_place_pose.pose.orientation.x = -0.69143;
      box_place_pose.pose.orientation.y = 0.722394;
      box_place_pose.pose.orientation.z = 0.00826619;
      box_place_pose.pose.orientation.w = 0.00188482;

      named_poses_["box_place_tcp"] = box_place_pose;
      broadcastNamedPoseTF("box_place_tcp", box_place_pose);

      RCLCPP_INFO(
        this->get_logger(),
        "Initialized default named pose 'box_place_tcp': "
        "pos=(%.6f, %.6f, %.6f), quat=(%.6f, %.6f, %.6f, %.6f)",
        box_place_pose.pose.position.x,
        box_place_pose.pose.position.y,
        box_place_pose.pose.position.z,
        box_place_pose.pose.orientation.x,
        box_place_pose.pose.orientation.y,
        box_place_pose.pose.orientation.z,
        box_place_pose.pose.orientation.w);

      return true;
    }
    catch (const std::exception & ex)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Exception while creating MoveGroupInterface: %s", ex.what());
      RCLCPP_ERROR(
        this->get_logger(),
        "Most likely URDF/SRDF (robot_description) are not available. "
        "Check that launch passes a non-empty 'robot_description' parameter.");
      return false;
    }
  }

private:
  struct NamedTarget
  {
    geometry_msgs::msg::PoseStamped pose;
    sensor_msgs::msg::JointState joint_state;
  };
  struct ForceJumpDetectionResult
  {
    bool triggered{false};
    int axis_index{-1};           // 0=x, 1=y, 2=z
    std::string axis_name{"none"};
    double current_force{0.0};
    double delta_force{0.0};
  };

  void tcpForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!msg || msg->data.size() < 6) {
      return;
    }

    prev_tcp_force_ = latest_tcp_force_;
    for (size_t i = 0; i < 6; ++i) {
      latest_tcp_force_[i] = msg->data[i];
    }
    has_tcp_force_ = true;
  }

  void graspStateEventCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string & event = msg->data;

    if (event == "grasp_confirmed")
    {
      loaded_mode_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Received grasp state event: '%s' -> switched to LOADED mode.",
        event.c_str());
      return;
    }

    if (event == "release_confirmed")
    {
      loaded_mode_ = false;
      RCLCPP_INFO(
        this->get_logger(),
        "Received grasp state event: '%s' -> switched to UNLOADED mode.",
        event.c_str());
      return;
    }

    if (event == "reset")
    {
      loaded_mode_ = false;
      RCLCPP_INFO(
        this->get_logger(),
        "Received grasp state event: '%s' -> reset to UNLOADED mode.",
        event.c_str());
      return;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Received unknown grasp state event: '%s'",
      event.c_str());
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_joint_state_ = *msg;
    has_joint_state_ = true;
  }

  bool getTcpPoseFromTF(geometry_msgs::msg::PoseStamped & out_pose)
  {
    if (!tf_buffer_)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "TF buffer not initialized. Cannot lookup tcp_pose.");
      return false;
    }

    const std::string tcp_frame = "tcp_pose";

    try
    {
      geometry_msgs::msg::TransformStamped tf_msg =
        tf_buffer_->lookupTransform(
          base_frame_,
          tcp_frame,
          tf2::TimePointZero);

      out_pose.header.stamp = this->now();
      out_pose.header.frame_id = base_frame_;
      out_pose.pose.position.x = tf_msg.transform.translation.x;
      out_pose.pose.position.y = tf_msg.transform.translation.y;
      out_pose.pose.position.z = tf_msg.transform.translation.z;
      out_pose.pose.orientation = tf_msg.transform.rotation;

      return true;
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to lookup TF from '%s' to 'tcp_pose': %s",
        base_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool getTargetPoseFromFrame(
    const std::string & target_frame,
    geometry_msgs::msg::PoseStamped & target_pose,
    std::string & error_msg)
  {
    if (target_frame.empty())
    {
      error_msg = "Empty frame_id.";
      return false;
    }

    try
    {
      geometry_msgs::msg::TransformStamped tf_msg =
        tf_buffer_->lookupTransform(
          base_frame_,
          target_frame,
          tf2::TimePointZero);

      target_pose.header.stamp = this->now();
      target_pose.header.frame_id = base_frame_;
      target_pose.pose.position.x = tf_msg.transform.translation.x;
      target_pose.pose.position.y = tf_msg.transform.translation.y;
      target_pose.pose.position.z = tf_msg.transform.translation.z;
      target_pose.pose.orientation = tf_msg.transform.rotation;

      RCLCPP_INFO(
        this->get_logger(),
        "Resolved target pose from TF frame '%s'.",
        target_frame.c_str());
      return true;
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "TF lookup failed for frame '%s': %s. Will try named pose fallback.",
        target_frame.c_str(), ex.what());
    }

    auto it = named_poses_.find(target_frame);
    if (it == named_poses_.end())
    {
      error_msg =
        "TF lookup failed and no named pose stored for frame '" + target_frame + "'";
      return false;
    }

    target_pose = it->second;
    if (target_pose.header.frame_id.empty())
    {
      target_pose.header.frame_id = base_frame_;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Resolved target pose from named pose '%s'.",
      target_frame.c_str());
    return true;
  }

  void broadcastNamedPoseTF(
    const std::string & name,
    const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!tf_broadcaster_)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "TF broadcaster not initialized, cannot broadcast named pose TF.");
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id =
      pose.header.frame_id.empty() ? base_frame_ : pose.header.frame_id;
    tf_msg.child_frame_id = name;
    tf_msg.transform.translation.x = pose.pose.position.x;
    tf_msg.transform.translation.y = pose.pose.position.y;
    tf_msg.transform.translation.z = pose.pose.position.z;
    tf_msg.transform.rotation = pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);

    RCLCPP_DEBUG(
      this->get_logger(),
      "Broadcast TF: %s -> %s",
      tf_msg.header.frame_id.c_str(),
      name.c_str());
  }

  void broadcastAllNamedPosesTF()
  {
    if (!tf_broadcaster_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "TF broadcaster not initialized, cannot periodically broadcast named poses.");
      return;
    }

    for (const auto & kv : named_poses_)
    {
      broadcastNamedPoseTF(kv.first, kv.second);
    }
  }

  bool buildPlanningGroupJointTarget(
    const sensor_msgs::msg::JointState & input_joint_state,
    std::vector<std::string> & group_joint_names,
    std::vector<double> & group_joint_positions,
    std::string & error_msg)
  {
    if (!move_group_)
    {
      error_msg = "MoveGroupInterface not initialized.";
      return false;
    }

    if (planning_group_joint_names_.empty())
    {
      error_msg =
        "Cached planning-group joint names are empty for planning group '" +
        planning_group_ + "'";
      return false;
    }

    if (input_joint_state.name.size() != input_joint_state.position.size())
    {
      error_msg = "Input joint_state name/position size mismatch.";
      return false;
    }

    std::map<std::string, double> input_map;
    for (size_t i = 0; i < input_joint_state.name.size(); ++i)
    {
      input_map[input_joint_state.name[i]] = input_joint_state.position[i];
    }

    group_joint_names.clear();
    group_joint_positions.clear();

    for (const auto & joint_name : planning_group_joint_names_)
    {
      auto it = input_map.find(joint_name);
      if (it == input_map.end())
      {
        error_msg =
          "Target joint_state missing planning-group joint '" + joint_name + "'";
        return false;
      }

      group_joint_names.push_back(joint_name);
      group_joint_positions.push_back(it->second);
    }

    std::stringstream ss;
    ss << "Filtered planning-group joints: ";
    for (size_t i = 0; i < group_joint_names.size(); ++i)
    {
      ss << group_joint_names[i] << "=" << group_joint_positions[i];
      if (i + 1 < group_joint_names.size())
      {
        ss << ", ";
      }
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

    return true;
  }

  bool validatePlannedTrajectoryEndpoint(
    const trajectory_msgs::msg::JointTrajectory & traj,
    const std::vector<std::string> & target_joint_names,
    const std::vector<double> & target_joint_positions,
    std::string & error_msg)
  {
    if (traj.points.empty())
    {
      error_msg = "Trajectory has 0 points, cannot validate endpoint.";
      return false;
    }

    if (traj.joint_names.empty())
    {
      error_msg = "Trajectory has empty joint_names, cannot validate endpoint.";
      return false;
    }

    const auto & final_point = traj.points.back();
    if (final_point.positions.size() != traj.joint_names.size())
    {
      error_msg = "Final trajectory point positions size mismatch.";
      return false;
    }

    std::map<std::string, double> traj_final_map;
    for (size_t i = 0; i < traj.joint_names.size(); ++i)
    {
      traj_final_map[traj.joint_names[i]] = final_point.positions[i];
    }

    const double joint_tolerance = 0.02;
    double max_abs_error = 0.0;
    std::string worst_joint;

    for (size_t i = 0; i < target_joint_names.size(); ++i)
    {
      const auto & joint_name = target_joint_names[i];
      auto it = traj_final_map.find(joint_name);
      if (it == traj_final_map.end())
      {
        error_msg =
          "Planned trajectory endpoint missing target joint '" + joint_name + "'";
        return false;
      }

      const double target = target_joint_positions[i];
      const double actual = it->second;
      const double abs_error = std::fabs(actual - target);

      RCLCPP_INFO(
        this->get_logger(),
        "Endpoint joint check: %s target=%.6f actual=%.6f abs_error=%.6f",
        joint_name.c_str(), target, actual, abs_error);

      if (abs_error > max_abs_error)
      {
        max_abs_error = abs_error;
        worst_joint = joint_name;
      }
    }

    if (max_abs_error > joint_tolerance)
    {
      std::stringstream ss;
      ss << "Planned trajectory endpoint does not match requested joint target. "
         << "worst_joint='" << worst_joint
         << "', max_abs_error=" << max_abs_error
         << " rad, tolerance=" << joint_tolerance << " rad";
      error_msg = ss.str();
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Trajectory endpoint joint validation passed. max_abs_error=%.6f rad",
      max_abs_error);

    return true;
  }

  bool planAndPublish(
    const geometry_msgs::msg::PoseStamped & target_pose,
    std::string & error_msg,
    double & planning_time_out)
  {
    if (!move_group_)
    {
      error_msg = "MoveGroupInterface not initialized. Cannot plan.";
      RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
      has_last_plan_ = false;
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Planning mode before pose plan: %s",
      loaded_mode_ ? "LOADED" : "UNLOADED");

    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(target_pose);
    move_group_->setPlanningTime(5.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_->plan(plan));
    planning_time_out = plan.planning_time;

    if (!success)
    {
      error_msg = "Planning failed (no valid trajectory).";
      RCLCPP_WARN(this->get_logger(), "%s", error_msg.c_str());
      has_last_plan_ = false;
      move_group_->clearPoseTargets();
      move_group_->clearPathConstraints();
      return false;
    }

    if (plan.trajectory.joint_trajectory.points.empty())
    {
      error_msg = "Planning succeeded but trajectory has 0 points.";
      RCLCPP_WARN(this->get_logger(), "%s", error_msg.c_str());
      has_last_plan_ = false;
      move_group_->clearPoseTargets();
      move_group_->clearPathConstraints();
      return false;
    }

    auto traj = plan.trajectory.joint_trajectory;
    traj.header.stamp = this->now();
    traj.header.frame_id = base_frame_;

    joint_traj_pub_->publish(traj);

    RCLCPP_INFO(
      this->get_logger(),
      "Published planned JointTrajectory with %zu points.",
      traj.points.size());

    last_plan_ = plan;
    has_last_plan_ = true;

    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    return true;
  }

  bool planAndPublishToJointTarget(
    const sensor_msgs::msg::JointState & joint_state,
    std::string & error_msg,
    double & planning_time_out)
  {
    if (!move_group_)
    {
      error_msg = "MoveGroupInterface not initialized. Cannot plan.";
      RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
      has_last_plan_ = false;
      return false;
    }

    if (joint_state.name.empty() || joint_state.position.empty())
    {
      error_msg = "Target joint_state is empty.";
      RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
      has_last_plan_ = false;
      return false;
    }

    if (joint_state.name.size() != joint_state.position.size())
    {
      error_msg = "Target joint_state name/position size mismatch.";
      RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
      has_last_plan_ = false;
      return false;
    }

    std::vector<std::string> group_joint_names;
    std::vector<double> group_joint_positions;
    if (!buildPlanningGroupJointTarget(
          joint_state, group_joint_names, group_joint_positions, error_msg))
    {
      RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
      has_last_plan_ = false;
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Planning to filtered joint target with %zu joints, mode=%s",
      group_joint_names.size(),
      loaded_mode_ ? "LOADED" : "UNLOADED");

    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    move_group_->setStartStateToCurrentState();
    move_group_->setPlanningTime(5.0);

    bool set_ok = move_group_->setJointValueTarget(group_joint_names, group_joint_positions);
    if (!set_ok)
    {
      error_msg = "Failed to set filtered joint value target for planning group.";
      RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
      has_last_plan_ = false;
      move_group_->clearPoseTargets();
      move_group_->clearPathConstraints();
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_->plan(plan));
    planning_time_out = plan.planning_time;

    if (!success)
    {
      error_msg = "Planning to joint target failed (no valid trajectory).";
      RCLCPP_WARN(this->get_logger(), "%s", error_msg.c_str());
      has_last_plan_ = false;
      move_group_->clearPoseTargets();
      move_group_->clearPathConstraints();
      return false;
    }

    if (plan.trajectory.joint_trajectory.points.empty())
    {
      error_msg = "Planning to joint target succeeded but trajectory has 0 points.";
      RCLCPP_WARN(this->get_logger(), "%s", error_msg.c_str());
      has_last_plan_ = false;
      move_group_->clearPoseTargets();
      move_group_->clearPathConstraints();
      return false;
    }

    if (!validatePlannedTrajectoryEndpoint(
          plan.trajectory.joint_trajectory,
          group_joint_names,
          group_joint_positions,
          error_msg))
    {
      RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
      has_last_plan_ = false;
      move_group_->clearPoseTargets();
      move_group_->clearPathConstraints();
      return false;
    }

    auto traj = plan.trajectory.joint_trajectory;
    traj.header.stamp = this->now();
    traj.header.frame_id = base_frame_;
    joint_traj_pub_->publish(traj);

    RCLCPP_INFO(
      this->get_logger(),
      "Published joint-target planned JointTrajectory with %zu points.",
      traj.points.size());

    last_plan_ = plan;
    has_last_plan_ = true;

    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    return true;
  }

  void planToPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped target_pose = *msg;
    if (target_pose.header.frame_id.empty())
    {
      target_pose.header.frame_id = base_frame_;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Received planning request to frame '%s' (topic /cs625/plan_to_pose).",
      target_pose.header.frame_id.c_str());

    std::string err;
    double planning_time = 0.0;
    bool ok = planAndPublish(target_pose, err, planning_time);
    if (!ok)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "planToPoseCallback failed: %s", err.c_str());
    }
  }

  void planToPoseServiceCallback(
    const std::shared_ptr<cs625_trajectory_tools::srv::PlanToPose::Request> request,
    std::shared_ptr<cs625_trajectory_tools::srv::PlanToPose::Response> response)
  {
    geometry_msgs::msg::PoseStamped target_pose = request->target_pose;
    if (target_pose.header.frame_id.empty())
    {
      target_pose.header.frame_id = base_frame_;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "PlanToPose service request received. frame_id='%s'",
      target_pose.header.frame_id.c_str());

    std::string err;
    double planning_time = 0.0;
    bool ok = planAndPublish(target_pose, err, planning_time);

    response->success = ok;
    response->planning_time = planning_time;
    response->message = ok ? "Planning succeeded and trajectory published." : err;
  }

  void planToTargetServiceCallback(
    const std::shared_ptr<cs625_trajectory_tools::srv::PlanToTarget::Request> request,
    std::shared_ptr<cs625_trajectory_tools::srv::PlanToTarget::Response> response)
  {
    geometry_msgs::msg::PoseStamped target_pose = request->target_pose;
    if (target_pose.header.frame_id.empty())
    {
      target_pose.header.frame_id = base_frame_;
    }

    std::string err;
    double planning_time = 0.0;
    bool ok = false;

    if (!request->use_joint_target)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "PlanToTarget request received with use_joint_target=false. "
        "Falling back to pose planning.");
      ok = planAndPublish(target_pose, err, planning_time);
    }
    else
    {
      RCLCPP_INFO(
        this->get_logger(),
        "PlanToTarget request received with use_joint_target=true. "
        "Will enforce filtered planning-group joint endpoint validation.");

      ok = planAndPublishToJointTarget(request->target_joint_state, err, planning_time);
    }

    response->success = ok;
    response->planning_time = planning_time;
    response->message = ok ? "Planning succeeded and trajectory published." : err;
  }

  void setNamedPoseCallback(
    const std::shared_ptr<cs625_trajectory_tools::srv::SetNamedPose::Request> request,
    std::shared_ptr<cs625_trajectory_tools::srv::SetNamedPose::Response> response)
  {
    const std::string name = request->name;
    geometry_msgs::msg::PoseStamped pose = request->pose;

    if (name.empty())
    {
      response->success = false;
      response->message = "Named pose name is empty.";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    if (pose.header.frame_id.empty())
    {
      pose.header.frame_id = base_frame_;
    }

    named_poses_[name] = pose;
    broadcastNamedPoseTF(name, pose);

    std::stringstream ss;
    ss << "Set named pose '" << name
       << "' from external pose, frame_id='" << pose.header.frame_id << "'";

    response->success = true;
    response->message = ss.str();

    RCLCPP_INFO(this->get_logger(), "[SetNamedPose] %s", response->message.c_str());
  }

  void setNamedPoseFromCurrentCallback(
    const std::shared_ptr<cs625_trajectory_tools::srv::SetNamedPoseFromCurrent::Request> request,
    std::shared_ptr<cs625_trajectory_tools::srv::SetNamedPoseFromCurrent::Response> response)
  {
    const std::string name = request->name;

    RCLCPP_INFO(
      this->get_logger(),
      "[SetNamedPoseFromCurrent] request received, name='%s'", name.c_str());

    geometry_msgs::msg::PoseStamped current_pose;
    if (!getTcpPoseFromTF(current_pose))
    {
      response->success = false;
      response->message = "Failed to get tcp_pose from TF. Cannot set named pose.";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    named_poses_[name] = current_pose;
    broadcastNamedPoseTF(name, current_pose);

    std::stringstream ss;
    ss << "Set named pose '" << name << "' "
       << "with frame_id='" << current_pose.header.frame_id << "'";

    response->success = true;
    response->message = ss.str();

    RCLCPP_INFO(this->get_logger(), "[SetNamedPoseFromCurrent] %s", response->message.c_str());
  }

  void setNamedTargetCallback(
    const std::shared_ptr<cs625_trajectory_tools::srv::SetNamedTarget::Request> request,
    std::shared_ptr<cs625_trajectory_tools::srv::SetNamedTarget::Response> response)
  {
    const std::string name = request->name;
    geometry_msgs::msg::PoseStamped pose = request->pose;
    sensor_msgs::msg::JointState joint_state = request->joint_state;

    if (name.empty())
    {
      response->success = false;
      response->message = "Named target name is empty.";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    if (pose.header.frame_id.empty())
    {
      pose.header.frame_id = base_frame_;
    }

    if (joint_state.name.empty() || joint_state.position.empty())
    {
      response->success = false;
      response->message = "Target joint_state is empty.";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    if (joint_state.name.size() != joint_state.position.size())
    {
      response->success = false;
      response->message = "Target joint_state name/position size mismatch.";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    // 新增：检查该 joint_state 是否至少覆盖 planning group 所需关节
    std::vector<std::string> group_joint_names;
    std::vector<double> group_joint_positions;
    std::string validation_error;
    if (!buildPlanningGroupJointTarget(
          joint_state, group_joint_names, group_joint_positions, validation_error))
    {
      response->success = false;
      response->message =
        "Target joint_state does not satisfy planning-group requirements: " +
        validation_error;
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    NamedTarget target;
    target.pose = pose;
    target.joint_state = joint_state;

    named_targets_[name] = target;
    named_poses_[name] = pose;
    broadcastNamedPoseTF(name, pose);

    std::stringstream ss;
    ss << "Set named target '" << name
       << "' from external pose+joint_state, frame_id='"
       << pose.header.frame_id
       << "', input_joint_count=" << joint_state.name.size()
       << ", planning_group_joint_count=" << group_joint_names.size();

    response->success = true;
    response->message = ss.str();

    RCLCPP_INFO(this->get_logger(), "[SetNamedTarget] %s", response->message.c_str());
  }

  void setNamedTargetFromCurrentCallback(
    const std::shared_ptr<cs625_trajectory_tools::srv::SetNamedTargetFromCurrent::Request> request,
    std::shared_ptr<cs625_trajectory_tools::srv::SetNamedTargetFromCurrent::Response> response)
  {
    const std::string name = request->name;

    RCLCPP_INFO(
      this->get_logger(),
      "[SetNamedTargetFromCurrent] request received, name='%s'",
      name.c_str());

    geometry_msgs::msg::PoseStamped current_pose;
    if (!getTcpPoseFromTF(current_pose))
    {
      response->success = false;
      response->message = "Failed to get tcp_pose from TF. Cannot set named target.";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    if (!has_joint_state_)
    {
      response->success = false;
      response->message = "No /joint_states received yet. Cannot set named target.";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    NamedTarget target;
    target.pose = current_pose;
    target.joint_state = last_joint_state_;

    named_targets_[name] = target;

    named_poses_[name] = current_pose;
    broadcastNamedPoseTF(name, current_pose);

    std::stringstream ss;
    ss << "Set named target '" << name << "' with pose frame_id='"
       << current_pose.header.frame_id
       << "' and " << target.joint_state.name.size() << " joints.";

    response->success = true;
    response->message = ss.str();

    RCLCPP_INFO(
      this->get_logger(),
      "[SetNamedTargetFromCurrent] %s",
      response->message.c_str());
  }

  void getNamedPoseCallback(
    const std::shared_ptr<cs625_trajectory_tools::srv::GetNamedPose::Request> request,
    std::shared_ptr<cs625_trajectory_tools::srv::GetNamedPose::Response> response)
  {
    const std::string & name = request->name;
    auto it = named_poses_.find(name);

    if (it == named_poses_.end())
    {
      response->success = false;
      response->message = "Named pose not found: " + name;
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    response->success = true;
    response->message = "Named pose found.";
    response->pose = it->second;

    RCLCPP_INFO(
      this->get_logger(),
      "Returned named pose '%s'.", name.c_str());
  }

  void getNamedTargetCallback(
    const std::shared_ptr<cs625_trajectory_tools::srv::GetNamedTarget::Request> request,
    std::shared_ptr<cs625_trajectory_tools::srv::GetNamedTarget::Response> response)
  {
    const std::string & name = request->name;
    auto it = named_targets_.find(name);

    if (it == named_targets_.end())
    {
      response->success = false;
      response->message = "Named target not found: " + name;
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    response->success = true;
    response->message = "Named target found.";
    response->pose = it->second.pose;
    response->joint_state = it->second.joint_state;

    RCLCPP_INFO(
      this->get_logger(),
      "Returned named target '%s'.", name.c_str());
  }

  void planToFrameCallback(
    const std::shared_ptr<cs625_trajectory_tools::srv::PlanToFrame::Request> request,
    std::shared_ptr<cs625_trajectory_tools::srv::PlanToFrame::Response> response)
  {
    if (!move_group_)
    {
      response->success = false;
      response->message = "MoveGroupInterface not initialized. Cannot plan.";
      response->planning_time = 0.0;
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    geometry_msgs::msg::PoseStamped target_pose;
    std::string error_msg;
    if (!getTargetPoseFromFrame(request->frame_id, target_pose, error_msg))
    {
      response->success = false;
      response->message = error_msg;
      response->planning_time = 0.0;
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "PlanToFrame: planning from '%s' to frame '%s'.",
      base_frame_.c_str(), request->frame_id.c_str());

    std::string err;
    double planning_time = 0.0;
    bool ok = planAndPublish(target_pose, err, planning_time);

    response->planning_time = planning_time;
    response->success = ok;
    response->message = ok ? "Planning succeeded and trajectory published."
                           : err;
  }

  bool executeCartesianPoseOnce(
    const geometry_msgs::msg::PoseStamped & target_pose,
    double & fraction_out,
    std::string & error_msg)
  {
    if (!move_group_)
    {
      error_msg = "MoveGroupInterface not initialized.";
      return false;
    }

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose.pose);

    moveit_msgs::msg::RobotTrajectory trajectory_msg;
    const double eef_step = 0.005;
    const double jump_threshold = 0.0;

    fraction_out = move_group_->computeCartesianPath(
      waypoints,
      eef_step,
      jump_threshold,
      trajectory_msg);

    if (fraction_out < 0.999)
    {
      std::stringstream ss;
      ss << "Cartesian path incomplete, fraction=" << fraction_out;
      error_msg = ss.str();
      return false;
    }

    if (trajectory_msg.joint_trajectory.points.empty())
    {
      error_msg = "Cartesian planning succeeded but trajectory has 0 points.";
      return false;
    }

    auto traj = trajectory_msg.joint_trajectory;
    traj.header.stamp = this->now();
    traj.header.frame_id = base_frame_;
    joint_traj_pub_->publish(traj);
  
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory = trajectory_msg;

    bool exec_ok = static_cast<bool>(move_group_->execute(cartesian_plan));
    if (!exec_ok)
    {
      error_msg = "Cartesian trajectory execution failed.";
      return false;
    }
  
    return true;
  }
  
    double distanceBetweenPoses(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b)
  {
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    const double dz = a.pose.position.z - b.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  
    geometry_msgs::msg::PoseStamped buildIntermediatePoseTowardsTarget(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & target_pose,
    double step_m)
  {
    geometry_msgs::msg::PoseStamped out = current_pose;
    out.header.stamp = this->now();
    out.header.frame_id = base_frame_;

    const double dx = target_pose.pose.position.x - current_pose.pose.position.x;
    const double dy = target_pose.pose.position.y - current_pose.pose.position.y;
    const double dz = target_pose.pose.position.z - current_pose.pose.position.z;

    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist <= step_m || dist < 1e-9)
    {
      out.pose = target_pose.pose;
      return out;
    }

    const double scale = step_m / dist;
    out.pose.position.x = current_pose.pose.position.x + dx * scale;
    out.pose.position.y = current_pose.pose.position.y + dy * scale;
    out.pose.position.z = current_pose.pose.position.z + dz * scale;

    // 第一版直接使用目标姿态，避免额外姿态插值复杂度
    out.pose.orientation = target_pose.pose.orientation;
    return out;
  }
  
    ForceJumpDetectionResult detectDominantForceJump()
  {
    ForceJumpDetectionResult result;

    if (!has_tcp_force_)
    {
      return result;
    }

    double best_abs_delta = 0.0;
    int best_axis = -1;
    std::string best_name = "none";
    double best_current_force = 0.0;
    double best_delta_force = 0.0;

    const char * axis_names[3] = {"x", "y", "z"};

    for (int i = 0; i < 3; ++i)
    {
      const double current_force = latest_tcp_force_[i];
      const double prev_force = prev_tcp_force_[i];
      const double delta_force = current_force - prev_force;
      const double abs_current_force = std::fabs(current_force);
      const double abs_delta_force = std::fabs(delta_force);

      if (abs_current_force > force_comp_max_force_axis_abs_ &&
          abs_delta_force > force_comp_force_jump_threshold_)
      {
        if (abs_delta_force > best_abs_delta)
        {
          best_abs_delta = abs_delta_force;
          best_axis = i;
          best_name = axis_names[i];
          best_current_force = current_force;
          best_delta_force = delta_force;
        }
      }
    }

    if (best_axis >= 0)
    {
      result.triggered = true;
      result.axis_index = best_axis;
      result.axis_name = best_name;
      result.current_force = best_current_force;
      result.delta_force = best_delta_force;
    }

    return result;
  }


  geometry_msgs::msg::PoseStamped buildToolCompensationPose(
    const geometry_msgs::msg::PoseStamped & current_pose,
    int axis_index,
    double current_force,
    double compensation_translation_m)
  {
    geometry_msgs::msg::PoseStamped out = current_pose;
    out.header.stamp = this->now();
    out.header.frame_id = base_frame_;

    tf2::Quaternion q(
      current_pose.pose.orientation.x,
      current_pose.pose.orientation.y,
      current_pose.pose.orientation.z,
      current_pose.pose.orientation.w);

    tf2::Matrix3x3 rot(q);

    tf2::Vector3 tool_axis(0.0, 0.0, 0.0);
    if (axis_index == 0)
    {
      tool_axis = tf2::Vector3(1.0, 0.0, 0.0);
    }
    else if (axis_index == 1)
    {
      tool_axis = tf2::Vector3(0.0, 1.0, 0.0);
    }
    else
    {
      tool_axis = tf2::Vector3(0.0, 0.0, 1.0);
    }

    // 工具系轴转到 base 系
    tf2::Vector3 base_axis = rot * tool_axis;

    // 若当前力为正，则沿负方向补偿；若力为负，则沿正方向补偿
    const double sign = (current_force >= 0.0) ? -1.0 : 1.0;

    out.pose.position.x += sign * compensation_translation_m * base_axis.x();
    out.pose.position.y += sign * compensation_translation_m * base_axis.y();
    out.pose.position.z += sign * compensation_translation_m * base_axis.z();

    // 姿态保持不变
    out.pose.orientation = current_pose.pose.orientation;
    return out;
  }
  
  

  void cartesianPlanToFrameCallback(
    const std::shared_ptr<cs625_trajectory_tools::srv::CartesianPlanToFrame::Request> request,
    std::shared_ptr<cs625_trajectory_tools::srv::CartesianPlanToFrame::Response> response)
  {
    if (!move_group_)
    {
      response->success = false;
      response->message = "MoveGroupInterface not initialized. Cannot do Cartesian planning.";
      response->fraction = 0.0;
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    geometry_msgs::msg::PoseStamped target_pose;
    std::string error_msg;
    if (!getTargetPoseFromFrame(request->frame_id, target_pose, error_msg))
    {
      response->success = false;
      response->message = error_msg;
      response->fraction = 0.0;
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "CartesianPlanToFrame: executing Cartesian path to '%s'.",
      request->frame_id.c_str());

    double fraction = 0.0;
    std::string exec_error;
    bool ok = executeCartesianPoseOnce(target_pose, fraction, exec_error);

    response->fraction = fraction;
    response->success = ok;
    response->message = ok ? "Cartesian planning and execution succeeded." : exec_error;

    if (ok)
    {
      RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    }
  }

  void cartesianForceCompensatedToFrameCallback(
    const std::shared_ptr<cs625_trajectory_tools::srv::CartesianForceCompensatedToFrame::Request> request,
    std::shared_ptr<cs625_trajectory_tools::srv::CartesianForceCompensatedToFrame::Response> response)
  {
    response->success = false;
    response->completion_ratio = 0.0;
    response->compensation_count = 0;
    response->dominant_axis = "none";

    if (!move_group_)
    {
      response->message = "MoveGroupInterface not initialized. Cannot do force-compensated Cartesian execution.";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }
    
    RCLCPP_INFO(
      this->get_logger(),
      "Force-compensated Cartesian execution is running without compliant-placement gate "
      "(cs625_compliant_placement is report-only).");

    if (!has_tcp_force_)
    {
      response->message = "No /cs625/tcp_force data received yet.";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    geometry_msgs::msg::PoseStamped target_pose;
    std::string error_msg;
    if (!getTargetPoseFromFrame(request->frame_id, target_pose, error_msg))
    {
      response->message = error_msg;
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    geometry_msgs::msg::PoseStamped start_pose;
    if (!getTcpPoseFromTF(start_pose))
    {
      response->message = "Failed to get current tcp_pose from TF.";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    const double total_distance = distanceBetweenPoses(start_pose, target_pose);
    if (total_distance < 1e-9)
    {
      response->success = true;
      response->completion_ratio = 1.0;
      response->message = "Already at target pose.";
      RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    int compensation_count = 0;
    std::string last_axis = "none";

    RCLCPP_INFO(
      this->get_logger(),
      "CartesianForceCompensatedToFrame: start force-compensated segmented execution to '%s', total_distance=%.6f m",
      request->frame_id.c_str(),
      total_distance);

    while (rclcpp::ok())
    {
      geometry_msgs::msg::PoseStamped current_pose;
      if (!getTcpPoseFromTF(current_pose))
      {
        response->message = "Failed to get current tcp_pose during segmented execution.";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
      }

      const double remaining_distance = distanceBetweenPoses(current_pose, target_pose);
      const double progressed_distance = total_distance - remaining_distance;
      response->completion_ratio = std::clamp(progressed_distance / total_distance, 0.0, 1.0);

      if (remaining_distance <= force_comp_goal_tolerance_m_)
      {
        response->success = true;
        response->compensation_count = compensation_count;
        response->dominant_axis = last_axis;
        response->message = "Force-compensated Cartesian execution reached target.";
        RCLCPP_INFO(
          this->get_logger(),
          "%s completion_ratio=%.3f compensation_count=%d dominant_axis=%s",
          response->message.c_str(),
          response->completion_ratio,
          compensation_count,
          last_axis.c_str());
        return;
      }

      geometry_msgs::msg::PoseStamped intermediate_pose =
        buildIntermediatePoseTowardsTarget(
          current_pose,
          target_pose,
          force_comp_step_translation_m_);

      double fraction = 0.0;
      std::string exec_error;
      if (!executeCartesianPoseOnce(intermediate_pose, fraction, exec_error))
      {
        response->message = "Segment execution failed: " + exec_error;
        response->compensation_count = compensation_count;
        response->dominant_axis = last_axis;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
      }

      auto force_jump = detectDominantForceJump();
      if (!force_jump.triggered)
      {
        continue;
      }

      last_axis = force_jump.axis_name;
      RCLCPP_WARN(
        this->get_logger(),
        "Force jump detected on axis=%s current_force=%.6f delta_force=%.6f, applying tool-frame compensation.",
        force_jump.axis_name.c_str(),
        force_jump.current_force,
        force_jump.delta_force);

      geometry_msgs::msg::PoseStamped compensation_pose =
        buildToolCompensationPose(
          intermediate_pose,
          force_jump.axis_index,
          force_jump.current_force,
          force_comp_compensation_translation_m_);

      double comp_fraction = 0.0;
      std::string comp_error;
      if (!executeCartesianPoseOnce(compensation_pose, comp_fraction, comp_error))
      {
        response->message = "Compensation execution failed: " + comp_error;
        response->compensation_count = compensation_count;
        response->dominant_axis = last_axis;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
      }

      compensation_count++;
      response->compensation_count = compensation_count;
      response->dominant_axis = last_axis;

      if (compensation_count > force_comp_max_compensation_count_)
      {
        response->message = "Exceeded maximum compensation count.";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
      }
    }

    response->message = "ROS shutdown during force-compensated Cartesian execution.";
  }

  void executeLastPlanCallback(
    const std::shared_ptr<cs625_trajectory_tools::srv::ExecuteLastPlan::Request> /*request*/,
    std::shared_ptr<cs625_trajectory_tools::srv::ExecuteLastPlan::Response> response)
  {
    if (!move_group_)
    {
      response->success = false;
      response->message = "MoveGroupInterface not initialized. Cannot execute.";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    if (!has_last_plan_)
    {
      response->success = false;
      response->message = "No last plan available. Please plan first.";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Executing last planned trajectory in %s mode...",
      loaded_mode_ ? "LOADED" : "UNLOADED");

    bool exec_ok = static_cast<bool>(move_group_->execute(last_plan_));

    if (!exec_ok)
    {
      response->success = false;
      response->message = "Execution of last plan failed.";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    response->success = true;
    response->message = "Execution of last plan succeeded.";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }
  

private:
  std::string planning_group_;
  std::string base_frame_;
  std::string ee_link_;
  std::string robot_description_xml_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::vector<std::string> planning_group_joint_names_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grasp_state_event_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tcp_force_sub_;

  std::array<double, 6> latest_tcp_force_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> prev_tcp_force_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool has_tcp_force_ = false;

  double force_comp_step_translation_m_{0.002};
  double force_comp_compensation_translation_m_{0.0005};
  double force_comp_force_jump_threshold_{8.0};
  double force_comp_max_force_axis_abs_{20.0};
  int force_comp_max_compensation_count_{3};
  double force_comp_goal_tolerance_m_{0.001};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr named_pose_tf_timer_;
  
  rclcpp::Service<cs625_trajectory_tools::srv::PlanToFrame>::SharedPtr plan_to_frame_srv_;
  rclcpp::Service<cs625_trajectory_tools::srv::PlanToPose>::SharedPtr plan_to_pose_srv_;
  rclcpp::Service<cs625_trajectory_tools::srv::PlanToTarget>::SharedPtr plan_to_target_srv_;
  rclcpp::Service<cs625_trajectory_tools::srv::ExecuteLastPlan>::SharedPtr execute_last_plan_srv_;
  rclcpp::Service<cs625_trajectory_tools::srv::SetNamedPose>::SharedPtr set_named_pose_srv_;
  rclcpp::Service<cs625_trajectory_tools::srv::SetNamedPoseFromCurrent>::SharedPtr
    set_named_pose_from_current_srv_;
  rclcpp::Service<cs625_trajectory_tools::srv::SetNamedTarget>::SharedPtr
    set_named_target_srv_;
  rclcpp::Service<cs625_trajectory_tools::srv::SetNamedTargetFromCurrent>::SharedPtr
    set_named_target_from_current_srv_;
  rclcpp::Service<cs625_trajectory_tools::srv::GetNamedPose>::SharedPtr
    get_named_pose_srv_;
  rclcpp::Service<cs625_trajectory_tools::srv::GetNamedTarget>::SharedPtr
    get_named_target_srv_;
  rclcpp::Service<cs625_trajectory_tools::srv::CartesianPlanToFrame>::SharedPtr
    cartesian_plan_to_frame_srv_;
    
  rclcpp::Service<cs625_trajectory_tools::srv::CartesianForceCompensatedToFrame>::SharedPtr
    cartesian_force_compensated_to_frame_srv_;
    


  moveit::planning_interface::MoveGroupInterface::Plan last_plan_;
  bool has_last_plan_ = false;
  bool loaded_mode_ = false;

  sensor_msgs::msg::JointState last_joint_state_;
  bool has_joint_state_ = false;

  std::map<std::string, geometry_msgs::msg::PoseStamped> named_poses_;
  std::map<std::string, NamedTarget> named_targets_;
};

}  // namespace cs625_trajectory_tools



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cs625_trajectory_tools::MoveitPlannerClientNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
