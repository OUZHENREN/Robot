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
  // 获取速度缩放因子
  if (!state_interfaces_.empty()) {
    auto& scaling_interface = state_interfaces_.back();
    if (scaling_interface.get_name() == scaled_params_.speed_scaling_interface_name) {
      auto value = scaling_interface.get_optional();
      scaling_factor_ = value.has_value() ? value.value() : 1.0;
    }
  }
  
  // 应用速度缩放
  auto scaled_period = rclcpp::Duration::from_nanoseconds(
    static_cast<int64_t>(period.nanoseconds() * scaling_factor_));
  
  // 直接调用基类的update方法，基类会处理状态检查
  return joint_trajectory_controller::JointTrajectoryController::update(time, scaled_period);
}
}  // namespace ELITE_CS_CONTROLLER

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ELITE_CS_CONTROLLER::ScaledJointTrajectoryController, controller_interface::ControllerInterface)
SOURCE_EOF
echo "✓ 创建了更安全的实现版本"
else
echo "✗ 源文件不存在: $SOURCE_FILE"
fi
