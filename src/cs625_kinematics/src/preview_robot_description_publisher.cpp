#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class PreviewRobotDescriptionPublisher : public rclcpp::Node
{
public:
  PreviewRobotDescriptionPublisher()
  : Node("preview_robot_description_publisher")
  {
    topic_name_ = this->declare_parameter<std::string>(
      "topic_name", "/preview_robot_description");
    robot_description_ = this->declare_parameter<std::string>(
      "robot_description", "");

    pub_ = this->create_publisher<std_msgs::msg::String>(
      topic_name_,
      rclcpp::QoS(1).transient_local().reliable());

    timer_ = this->create_wall_timer(
      1000ms,
      std::bind(&PreviewRobotDescriptionPublisher::publishDescription, this));

    RCLCPP_INFO(
      this->get_logger(),
      "PreviewRobotDescriptionPublisher started. topic='%s', description_length=%zu",
      topic_name_.c_str(),
      robot_description_.size());
  }

private:
  void publishDescription()
  {
    std_msgs::msg::String msg;
    msg.data = robot_description_;
    pub_->publish(msg);
  }

  std::string topic_name_;
  std::string robot_description_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreviewRobotDescriptionPublisher>());
  rclcpp::shutdown();
  return 0;
}
