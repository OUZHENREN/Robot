#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class PreviewJointStateRelay : public rclcpp::Node
{
public:
  PreviewJointStateRelay()
  : Node("preview_joint_state_relay")
  {
    input_topic_ = this->declare_parameter<std::string>(
      "input_topic", "/cs625/selected_multi_config_target_joint_state");
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/cs625/preview_joint_states");
    prefix_ = this->declare_parameter<std::string>(
      "joint_prefix", "preview_");

    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      input_topic_, 10,
      std::bind(&PreviewJointStateRelay::jointStateCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(output_topic_, 10);

    RCLCPP_INFO(this->get_logger(),
      "PreviewJointStateRelay started. input_topic='%s', output_topic='%s', joint_prefix='%s'",
      input_topic_.c_str(), output_topic_.c_str(), prefix_.c_str());
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    auto out = sensor_msgs::msg::JointState();
    out.header = msg->header;

    if (out.header.stamp.sec == 0 && out.header.stamp.nanosec == 0) {
      out.header.stamp = this->now();
    }

    out.name.reserve(msg->name.size());
    for (const auto & name : msg->name) {
      out.name.push_back(prefix_ + name);
    }

    out.position = msg->position;
    out.velocity = msg->velocity;
    out.effort = msg->effort;

    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string prefix_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreviewJointStateRelay>());
  rclcpp::shutdown();
  return 0;
}
