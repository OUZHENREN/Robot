#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class DisplayTrajectoryBridgeNode : public rclcpp::Node
{
public:
  DisplayTrajectoryBridgeNode()
  : Node("display_trajectory_bridge_node")
  {
    // 参数：输入 DisplayTrajectory topic，输出 JointTrajectory topic
    this->declare_parameter<std::string>("input_topic", "/display_planned_path");
    this->declare_parameter<std::string>("output_topic", "/cs625/planned_joint_trajectory");

    input_topic_  = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();

    RCLCPP_INFO(get_logger(), "DisplayTrajectoryBridgeNode starting with:");
    RCLCPP_INFO(get_logger(), "  input_topic  = %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  output_topic = %s", output_topic_.c_str());

    sub_ = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      input_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&DisplayTrajectoryBridgeNode::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      output_topic_, 10);
  }

private:
  void callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
  {
    if (!msg)
    {
      RCLCPP_WARN(get_logger(), "Received null DisplayTrajectory message, ignore.");
      return;
    }

    if (msg->trajectory.empty())
    {
      RCLCPP_WARN(get_logger(),
                  "Received DisplayTrajectory with 0 trajectories, ignore.");
      return;
    }

    // 这里选择使用最后一个 RobotTrajectory
    const auto &robot_traj = msg->trajectory.back();

    if (robot_traj.joint_trajectory.points.empty())
    {
      RCLCPP_WARN(get_logger(),
                  "RobotTrajectory has 0 points, ignore.");
      return;
    }

    trajectory_msgs::msg::JointTrajectory jt = robot_traj.joint_trajectory;

    // 确保 header 有合理的时间戳
    jt.header.stamp = this->now();

    if (jt.header.frame_id.empty())
    {
      // frame_id 不是必须的，但可以填成 MoveIt 常用的 "base_link" 或你自己的 base_frame
      jt.header.frame_id = "base_link";
    }

    pub_->publish(jt);

    RCLCPP_INFO(get_logger(),
                "Published JointTrajectory with %zu points from DisplayTrajectory.",
                jt.points.size());
  }

  std::string input_topic_;
  std::string output_topic_;

  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DisplayTrajectoryBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
