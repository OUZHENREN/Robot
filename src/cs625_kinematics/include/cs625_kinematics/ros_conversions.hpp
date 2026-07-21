#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace cs625_kinematics {

Eigen::Matrix4d poseToEigen(const geometry_msgs::msg::Pose& pose);
Eigen::Matrix4d poseStampedToEigen(const geometry_msgs::msg::PoseStamped& pose_stamped);

geometry_msgs::msg::Pose eigenToPose(const Eigen::Matrix4d& T);
geometry_msgs::msg::PoseStamped eigenToPoseStamped(
    const Eigen::Matrix4d& T,
    const std::string& frame_id = "");

bool jointStateToArray(const sensor_msgs::msg::JointState& msg, std::array<double,6>& q);

sensor_msgs::msg::JointState arrayToJointState(
    const std::array<double,6>& q,
    const std::vector<std::string>& names = {});

std::vector<std::string> defaultJointNames();

}  // namespace cs625_kinematics
