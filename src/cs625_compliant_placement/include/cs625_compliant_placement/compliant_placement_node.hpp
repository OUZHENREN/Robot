#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "cs625_state_monitor/msg/cs625_state.hpp"

#include "cs625_compliant_placement/msg/compliant_placement_state.hpp"
#include "cs625_compliant_placement/srv/get_compliant_placement_state.hpp"

#include "cs625_compliant_placement/contact_detector.hpp"
#include "cs625_compliant_placement/placement_types.hpp"

namespace cs625_compliant_placement
{

class CompliantPlacementNode : public rclcpp::Node
{
public:
  CompliantPlacementNode();

private:
  void declare_and_load_parameters();
  void raw_state_callback(const cs625_state_monitor::msg::CS625State::SharedPtr msg);
  void control_loop();
  void publish_state();

  void handle_get_state(
    const std::shared_ptr<srv::GetCompliantPlacementState::Request> request,
    std::shared_ptr<srv::GetCompliantPlacementState::Response> response);

private:
  std::string raw_state_topic_;
  std::string state_topic_;
  std::string metrics_topic_ {"/cs625/compliant_placement/metrics"};
  std::string public_state_name_ {"REPORT_ONLY"};
  std::string public_stop_reason_ {"report_only"};

  double control_rate_hz_ {50.0};

  rclcpp::Subscription<cs625_state_monitor::msg::CS625State>::SharedPtr raw_state_sub_;
  rclcpp::Publisher<msg::CompliantPlacementState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr metrics_pub_;
  rclcpp::Service<srv::GetCompliantPlacementState>::SharedPtr get_state_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  cs625_state_monitor::msg::CS625State latest_raw_state_;
  bool has_raw_state_ {false};

  DetectorResult detector_result_;

  ContactDetector contact_detector_;
};

}  // namespace cs625_compliant_placement
