#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>

namespace cs625_trajectory_tools
{

class PlannedTcpPathNode : public rclcpp::Node
{
public:
  PlannedTcpPathNode();
  bool initialize();

private:
  void jointTrajectoryCallback(
      const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  bool initRobotModel(const std::string &robot_description_param);

  bool copyRobotDescriptionFromMoveGroup(const std::string &robot_description_param);

  bool setJointPositions(
      const trajectory_msgs::msg::JointTrajectory &traj,
      const trajectory_msgs::msg::JointTrajectoryPoint &point);

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  nav_msgs::msg::Path last_path_;
  bool has_valid_path_ {false};

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  std::shared_ptr<moveit::core::RobotState> robot_state_;
  const moveit::core::JointModelGroup *joint_model_group_ {nullptr};

  std::string planning_group_name_ {"cs625_arm"};
  std::string base_frame_ {"base"};
  std::string ee_link_name_ {"my_end_effector_link"};
  std::string robot_description_param_ {"robot_description"};

  std::unordered_map<std::string, std::size_t> joint_name_to_index_;
};

}  // namespace cs625_trajectory_tools
