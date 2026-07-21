#include "cs625_compliant_placement/compliant_placement_node.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace cs625_compliant_placement
{

CompliantPlacementNode::CompliantPlacementNode()
: Node("cs625_compliant_placement_node")
{
  declare_and_load_parameters();

  contact_detector_.configure(
    this->get_parameter("contact_alpha").as_double(),
    this->get_parameter("baseline_alpha").as_double()
  );

  raw_state_sub_ = this->create_subscription<cs625_state_monitor::msg::CS625State>(
    raw_state_topic_,
    10,
    std::bind(&CompliantPlacementNode::raw_state_callback, this, std::placeholders::_1)
  );

  state_pub_ = this->create_publisher<msg::CompliantPlacementState>(
    state_topic_,
    10
  );

  metrics_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    metrics_topic_,
    10
  );

  get_state_srv_ = this->create_service<srv::GetCompliantPlacementState>(
    "get_compliant_placement_state",
    std::bind(
      &CompliantPlacementNode::handle_get_state,
      this,
      std::placeholders::_1,
      std::placeholders::_2)
  );

  const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&CompliantPlacementNode::control_loop, this)
  );

  RCLCPP_INFO(this->get_logger(), "cs625_compliant_placement_node started in report-only mode");
}

void CompliantPlacementNode::declare_and_load_parameters()
{
  this->declare_parameter<std::string>("raw_state_topic", "/cs625/raw_state");
  this->declare_parameter<std::string>("state_topic", "/cs625/compliant_placement/state");

  this->declare_parameter<double>("control_rate_hz", 50.0);
  this->declare_parameter<double>("contact_alpha", 0.2);
  this->declare_parameter<double>("baseline_alpha", 0.01);

  raw_state_topic_ = this->get_parameter("raw_state_topic").as_string();
  state_topic_ = this->get_parameter("state_topic").as_string();
  control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
}

void CompliantPlacementNode::raw_state_callback(
  const cs625_state_monitor::msg::CS625State::SharedPtr msg)
{
  latest_raw_state_ = *msg;
  has_raw_state_ = true;
}

void CompliantPlacementNode::control_loop()
{
  if (has_raw_state_) {
    detector_result_ = contact_detector_.update(latest_raw_state_);
  } else {
    detector_result_ = DetectorResult{};
  }

  publish_state();
}

void CompliantPlacementNode::publish_state()
{
  msg::CompliantPlacementState msg;
  msg.stamp = this->now();

  msg.state = public_state_name_;
  msg.active = false;
  msg.contact_detected = false;
  msg.jam_detected = false;
  msg.target_reached = false;
  msg.aborted = false;

  msg.tau_metric = detector_result_.tau_metric;
  msg.filtered_tau_metric = detector_result_.filtered_tau_metric;

  msg.current_tcp_x_mm = latest_raw_state_.actual_tcp_x_mm;
  msg.current_tcp_y_mm = latest_raw_state_.actual_tcp_y_mm;
  msg.current_tcp_z_mm = latest_raw_state_.actual_tcp_z_mm;
  msg.current_tcp_rx_deg = latest_raw_state_.actual_rot_x_deg;
  msg.current_tcp_ry_deg = latest_raw_state_.actual_rot_y_deg;
  msg.current_tcp_rz_deg = latest_raw_state_.actual_rot_z_deg;

  msg.stop_reason = public_stop_reason_;

  state_pub_->publish(msg);

  std_msgs::msg::Float64MultiArray metrics_msg;
  metrics_msg.data = {
    detector_result_.tau_metric,
    detector_result_.filtered_tau_metric
  };
  metrics_pub_->publish(metrics_msg);
}

void CompliantPlacementNode::handle_get_state(
  const std::shared_ptr<srv::GetCompliantPlacementState::Request> /*request*/,
  std::shared_ptr<srv::GetCompliantPlacementState::Response> response)
{
  msg::CompliantPlacementState state;
  state.stamp = this->now();

  state.state = public_state_name_;
  state.active = false;
  state.contact_detected = false;
  state.jam_detected = false;
  state.target_reached = false;
  state.aborted = false;

  state.tau_metric = detector_result_.tau_metric;
  state.filtered_tau_metric = detector_result_.filtered_tau_metric;

  state.current_tcp_x_mm = latest_raw_state_.actual_tcp_x_mm;
  state.current_tcp_y_mm = latest_raw_state_.actual_tcp_y_mm;
  state.current_tcp_z_mm = latest_raw_state_.actual_tcp_z_mm;
  state.current_tcp_rx_deg = latest_raw_state_.actual_rot_x_deg;
  state.current_tcp_ry_deg = latest_raw_state_.actual_rot_y_deg;
  state.current_tcp_rz_deg = latest_raw_state_.actual_rot_z_deg;

  state.stop_reason = public_stop_reason_;

  response->success = true;
  response->message = "State query success";
  response->state = state;
}

}  // namespace cs625_compliant_placement
