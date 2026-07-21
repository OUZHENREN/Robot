#!/bin/bash

HEADER_FILE="src/eli_cs_controllers/include/eli_cs_controllers/scaled_joint_trajectory_controller.hpp"
TEMP_FILE="${HEADER_FILE}.tmp"

if [ -f "$HEADER_FILE" ]; then
    # 创建修复后的头文件
    cat > "$TEMP_FILE" << 'HEADER_EOF'
#pragma once

#include <joint_trajectory_controller/joint_trajectory_controller.hpp>
#include <controller_interface/controller_interface.hpp>

namespace ELITE_CS_CONTROLLER
{

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
      [[maybe_unused]] auto result = joint_interface[index].get().set_value(trajectory_point_interface[index]);
    }
  }

private:
  double scaling_factor_{1.0};
};

}  // namespace ELITE_CS_CONTROLLER
HEADER_EOF

    # 替换原文件
    mv "$TEMP_FILE" "$HEADER_FILE"
    echo "✓ 控制器头文件已修复"
else
    echo "✗ 控制器头文件不存在: $HEADER_FILE"
fi
