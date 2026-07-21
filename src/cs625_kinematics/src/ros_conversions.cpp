#include "cs625_kinematics/ros_conversions.hpp"

#include <cmath>
#include <Eigen/Geometry>

namespace cs625_kinematics {

Eigen::Matrix4d poseToEigen(const geometry_msgs::msg::Pose& pose)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    T(0, 3) = pose.position.x;
    T(1, 3) = pose.position.y;
    T(2, 3) = pose.position.z;

    const double qx = pose.orientation.x;
    const double qy = pose.orientation.y;
    const double qz = pose.orientation.z;
    const double qw = pose.orientation.w;

    const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    if (norm > 1e-12) {
        Eigen::Quaterniond q(qw / norm, qx / norm, qy / norm, qz / norm);
        R = q.toRotationMatrix();
    }

    T.block<3,3>(0,0) = R;
    return T;
}

Eigen::Matrix4d poseStampedToEigen(const geometry_msgs::msg::PoseStamped& pose_stamped)
{
    return poseToEigen(pose_stamped.pose);
}

geometry_msgs::msg::Pose eigenToPose(const Eigen::Matrix4d& T)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = T(0, 3);
    pose.position.y = T(1, 3);
    pose.position.z = T(2, 3);

    const Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Quaterniond q(R);
    q.normalize();

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}

geometry_msgs::msg::PoseStamped eigenToPoseStamped(
    const Eigen::Matrix4d& T,
    const std::string& frame_id)
{
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame_id;
    ps.pose = eigenToPose(T);
    return ps;
}

bool jointStateToArray(const sensor_msgs::msg::JointState& msg, std::array<double,6>& q)
{
    if (msg.position.size() < 6) {
        return false;
    }

    for (size_t i = 0; i < 6; ++i) {
        q[i] = msg.position[i];
    }

    return true;
}

sensor_msgs::msg::JointState arrayToJointState(
    const std::array<double,6>& q,
    const std::vector<std::string>& names)
{
    sensor_msgs::msg::JointState msg;
    msg.position.resize(6);

    for (size_t i = 0; i < 6; ++i) {
        msg.position[i] = q[i];
    }

    if (!names.empty()) {
        msg.name = names;
    } else {
        msg.name = defaultJointNames();
    }

    return msg;
}

std::vector<std::string> defaultJointNames()
{
    return {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };
}

}  // namespace cs625_kinematics
