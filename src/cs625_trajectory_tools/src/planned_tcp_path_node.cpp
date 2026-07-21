#include "cs625_trajectory_tools/planned_tcp_path_node.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace cs625_trajectory_tools
{

PlannedTcpPathNode::PlannedTcpPathNode()
: Node("planned_tcp_path_node")
{
  this->declare_parameter<std::string>("planning_group", planning_group_name_);
  this->declare_parameter<std::string>("base_frame", base_frame_);
  this->declare_parameter<std::string>("ee_link", ee_link_name_);
  this->declare_parameter<std::string>("robot_description_param", "robot_description");

  planning_group_name_ = this->get_parameter("planning_group").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  ee_link_name_ = this->get_parameter("ee_link").as_string();
  robot_description_param_ =
      this->get_parameter("robot_description_param").as_string();

  RCLCPP_INFO(this->get_logger(), "PlannedTcpPathNode starting with:");
  RCLCPP_INFO(this->get_logger(), "  planning_group = %s", planning_group_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "  base_frame     = %s", base_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  ee_link        = %s", ee_link_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "  robot_description param = %s",
              robot_description_param_.c_str());

  joint_traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/cs625/planned_joint_trajectory",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&PlannedTcpPathNode::jointTrajectoryCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/cs625/planned_tcp_path", 10);
}

bool PlannedTcpPathNode::initialize()
{
  if (!initRobotModel(robot_description_param_)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to initialize MoveIt RobotModel. Node will still run but callbacks will fail.");
    return false;
  }
  return true;
}

bool PlannedTcpPathNode::initRobotModel(const std::string &robot_description_param)
{
  try
  {
    if (!this->has_parameter(robot_description_param)) {
      this->declare_parameter<std::string>(robot_description_param, "");
      RCLCPP_INFO(this->get_logger(),
                  "Declared local parameter '%s' (empty by default).",
                  robot_description_param.c_str());
    }

    std::string semantic_param_name = robot_description_param + std::string("_semantic");
    if (!this->has_parameter(semantic_param_name)) {
      this->declare_parameter<std::string>(semantic_param_name, "");
      RCLCPP_INFO(this->get_logger(),
                  "Declared local parameter '%s' (empty by default).",
                  semantic_param_name.c_str());
    }

    std::string robot_description_xml =
        this->get_parameter(robot_description_param).as_string();
    std::string robot_semantic_xml =
        this->get_parameter(semantic_param_name).as_string();

    if (robot_description_xml.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Local parameter '%s' is empty. "
                  "RobotModelLoader will still try to resolve it from global parameters.",
                  robot_description_param.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Local parameter '%s' has non-empty value (length=%zu).",
                  robot_description_param.c_str(),
                  robot_description_xml.size());
    }

    if (robot_semantic_xml.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Local semantic parameter '%s' is empty. "
                  "RobotModelLoader may not be able to construct RobotModel.",
                  semantic_param_name.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Local semantic parameter '%s' has non-empty value (length=%zu).",
                  semantic_param_name.c_str(),
                  robot_semantic_xml.size());
    }

    auto moveit_node = shared_from_this();

    robot_model_loader_ =
        std::make_shared<robot_model_loader::RobotModelLoader>(
            moveit_node, robot_description_param, true);

    robot_model_ = robot_model_loader_->getModel();
    if (!robot_model_) {
      RCLCPP_ERROR(this->get_logger(),
                   "RobotModel is null. Check if '%s' and its semantic variant are set "
                   "and if MoveIt can parse the URDF/SRDF.",
                   robot_description_param.c_str());
      return false;
    }

    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    robot_state_->setToDefaultValues();

    joint_model_group_ = robot_model_->getJointModelGroup(planning_group_name_);
    if (!joint_model_group_) {
      RCLCPP_ERROR(this->get_logger(),
                   "JointModelGroup '%s' not found in RobotModel.",
                   planning_group_name_.c_str());
      return false;
    }

    const auto &group_joint_names = joint_model_group_->getVariableNames();
    RCLCPP_INFO(this->get_logger(), "Planning group '%s' joint names:",
                planning_group_name_.c_str());
    for (const auto &name : group_joint_names) {
      RCLCPP_INFO(this->get_logger(), "  %s", name.c_str());
    }

    if (!robot_model_->hasLinkModel(ee_link_name_)) {
      RCLCPP_WARN(this->get_logger(),
                  "End-effector link '%s' not found in RobotModel. "
                  "FK calls will likely fail.",
                  ee_link_name_.c_str());
    }

    return true;
  }
  catch (const std::exception &ex)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Exception while initializing RobotModel: %s", ex.what());
    return false;
  }
}

bool PlannedTcpPathNode::setJointPositions(
    const trajectory_msgs::msg::JointTrajectory &traj,
    const trajectory_msgs::msg::JointTrajectoryPoint &point)
{
  if (!robot_state_ || !joint_model_group_) {
    RCLCPP_ERROR(this->get_logger(),
                 "RobotState or JointModelGroup not initialized.");
    return false;
  }

  if (joint_name_to_index_.empty()) {
    for (std::size_t i = 0; i < traj.joint_names.size(); ++i) {
      joint_name_to_index_[traj.joint_names[i]] = i;
    }
  }

  std::vector<double> joint_values;
  joint_values.reserve(joint_model_group_->getVariableCount());

  for (const auto &joint_name : joint_model_group_->getVariableNames()) {
    auto it = joint_name_to_index_.find(joint_name);
    if (it == joint_name_to_index_.end()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Joint '%s' not found in incoming trajectory.",
                   joint_name.c_str());
      return false;
    }
    std::size_t idx = it->second;
    if (idx >= point.positions.size()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Index %zu out of range for positions size %zu.",
                   idx, point.positions.size());
      return false;
    }
    joint_values.push_back(point.positions[idx]);
  }

  robot_state_->setJointGroupPositions(joint_model_group_, joint_values);
  robot_state_->update();
  return true;
}

void PlannedTcpPathNode::jointTrajectoryCallback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (!robot_state_ || !robot_model_) {
    RCLCPP_ERROR(this->get_logger(),
                 "RobotModel/RobotState not initialized. Cannot process trajectory.");
    return;
  }

  if (msg->points.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received JointTrajectory with 0 points, ignoring.");
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "Received JointTrajectory with %zu points.", msg->points.size());

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = base_frame_;

  for (std::size_t i = 0; i < msg->points.size(); ++i) {
    const auto &point = msg->points[i];

    if (!setJointPositions(*msg, point)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to set joint positions for point %zu, aborting path generation.",
                   i);
      return;
    }

    const Eigen::Isometry3d &ee_tf =
        robot_state_->getGlobalLinkTransform(ee_link_name_);

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = base_frame_;
    pose_stamped.header.stamp = this->now();

    pose_stamped.pose.position.x = ee_tf.translation().x();
    pose_stamped.pose.position.y = ee_tf.translation().y();
    pose_stamped.pose.position.z = ee_tf.translation().z();

    Eigen::Quaterniond q(ee_tf.rotation());
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    path_msg.poses.push_back(pose_stamped);
  }

  path_pub_->publish(path_msg);
  last_path_ = path_msg;
  has_valid_path_ = true;

  RCLCPP_INFO(this->get_logger(),
              "Published planned TCP Path with %zu poses.", path_msg.poses.size());
}

}  // namespace cs625_trajectory_tools

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cs625_trajectory_tools::PlannedTcpPathNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
