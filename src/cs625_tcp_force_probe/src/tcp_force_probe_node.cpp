#include <memory>
#include <string>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

class TcpForceProbeNode : public rclcpp::Node
{
public:
  TcpForceProbeNode()
  : Node("tcp_force_probe_node")
  {
    input_wrench_topic_ = this->declare_parameter<std::string>(
      "input_wrench_topic",
      "/force_torque_sensor_broadcaster/wrench");

    output_force_topic_ = this->declare_parameter<std::string>(
      "output_force_topic",
      "/cs625/tcp_force");

    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      output_force_topic_, 10);

    subscription_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      input_wrench_topic_,
      10,
      std::bind(&TcpForceProbeNode::wrenchCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "tcp_force_probe_node started. input_wrench_topic=%s output_force_topic=%s",
      input_wrench_topic_.c_str(),
      output_force_topic_.c_str());
  }

private:
  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    std_msgs::msg::Float64MultiArray out_msg;
    out_msg.data.resize(6, 0.0);

    out_msg.data[0] = msg->wrench.force.x;
    out_msg.data[1] = msg->wrench.force.y;
    out_msg.data[2] = msg->wrench.force.z;
    out_msg.data[3] = msg->wrench.torque.x;
    out_msg.data[4] = msg->wrench.torque.y;
    out_msg.data[5] = msg->wrench.torque.z;

    publisher_->publish(out_msg);

    // 不再持续打印桥接日志，避免常驻节点刷屏。
    // 如需调试，可临时改为 DEBUG 或恢复节流日志。
  }

private:
  std::string input_wrench_topic_;
  std::string output_force_topic_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TcpForceProbeNode>());
  rclcpp::shutdown();
  return 0;
}
