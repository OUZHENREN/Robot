#!/bin/bash
SOURCE_FILE="src/eli_cs_controllers/src/scaled_joint_trajectory_controller.cpp"
if [ -f "$SOURCE_FILE" ]; then
    cat > "$SOURCE_FILE" << 'SOURCE_EOF'
#include "eli_cs_controllers/scaled_joint_trajectory_controller.hpp"
#include <angles/angles.h>

namespace ELITE_CS_CONTROLLER
{
controller_interface::CallbackReturn ScaledJointTrajectoryController::on_init()
{
  auto ret = joint_trajectory_controller::JointTrajectoryController::on_init();
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ScaledJointTrajectoryController::state_interface_configuration() const
{
  auto conf = joint_trajectory_controller::JointTrajectoryController::state_interface_configuration();
  conf.names.push_back(scaled_params_.speed_scaling_interface_name);
  return conf;
}

controller_interface::CallbackReturn ScaledJointTrajectoryController::on_activate(const rclcpp_lifecycle::State& state)
{
  return joint_trajectory_controller::JointTrajectoryController::on_activate(state);
}

controller_interface::return_type ScaledJointTrajectoryController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  if (!state_interfaces_.empty()) {
    auto& scaling_interface = state_interfaces_.back();
    if (scaling_interface.get_name() == scaled_params_.speed_scaling_interface_name) {
      scaling_factor_ = scaling_interface.get_value();
    }
  }
  
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    return controller_interface::return_type::OK;
  }
  
  auto scaled_period = rclcpp::Duration::from_nanoseconds(
    static_cast<int64_t>(period.nanoseconds() * scaling_factor_));
  
  return joint_trajectory_controller::JointTrajectoryController::update(time, scaled_period);
}
}  // namespace ELITE_CS_CONTROLLER

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ELITE_CS_CONTROLLER::ScaledJointTrajectoryController, controller_interface::ControllerInterface)
SOURCE_EOF
    echo "✓ 修复了轨迹控制器实现"
else
    echo "✗ 源文件不存在: $SOURCE_FILE"
fi
