#!/bin/bash
HEADER_FILE="src/eli_cs_controllers/include/eli_cs_controllers/scaled_joint_trajectory_controller.hpp"
if [ -f "$HEADER_FILE" ]; then
    cat > "$HEADER_FILE" << 'HEADER_EOF'
#pragma once
#include <joint_trajectory_controller/joint_trajectory_controller.hpp>
#include <controller_interface/controller_interface.hpp>
#include <angles/angles.h>
#include <realtime_tools/realtime_buffer.h>

namespace ELITE_CS_CONTROLLER
{
struct ScaledJointTrajectoryControllerParams
{
    std::string speed_scaling_interface_name = "speed_scaling";
};

class ScaledJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  template<typename T>
  void assign_interface_from_point(const T& joint_interface, const std::vector<double>& trajectory_point_interface)
  {
    for (size_t index = 0; index < joint_interface.size(); ++index)
    {
      (void)joint_interface[index].get().set_value(trajectory_point_interface[index]);
    }
  }
private:
  double scaling_factor_{1.0};
  ScaledJointTrajectoryControllerParams scaled_params_;}  // namespace ELITE_CS_CONTROLLER
HEADER_EOF
    echo "✓ 修复了控制器头文件"
else
    echo "✗ 头文件不存在: $HEADER_FILE"
fi
