#!/bin/bash
echo "🔧 修复URDF错误并启动CS625机器人"

# 创建修复后的URDF文件
URDF_FILE="/tmp/cs625_fixed.urdf"

cat > "$URDF_FILE" << 'XML'
<?xml version="1.0"?>
<robot name="eli_cs625">
  <!-- 定义材料 -->
  <material name="steel"><color rgba="0.7 0.7 0.7 1.0"/></material>
  <material name="blue"><color rgba="0.0 0.3 0.8 1.0"/></material>
  <material name="red"><color rgba="0.8 0.1 0.1 1.0"/></material>
  <material name="green"><color rgba="0.1 0.8 0.1 1.0"/></material>
  <material name="yellow"><color rgba="0.8 0.8 0.1 1.0"/></material>
  <material name="purple"><color rgba="0.6 0.1 0.8 1.0"/></material>
  <material name="orange"><color rgba="0.9 0.5 0.1 1.0"/></material>
<!-- ELI CS625 6轴工业机器人 -->
  <link name="base_link">
    <visual><geometry><cylinder length="0.3" radius="0.3"/></geometry><material name="steel"/></visual>
  </link>
  
  <link name="link1">
    <visual><geometry><cylinder length="0.6" radius="0.15"/></geometry><material name="red"/></visual>
  </link>
<link name="link2">
    <visual><geometry><cylinder length="0.5" radius="0.12"/></geometry><material name="green"/></visual>
  </link>
  
  <link name="link3">
    <visual><geometry><cylinder length="0.4" radius="0.1"/></geometry><material name="blue"/></visual>
  </link>
<link name="link4">
    <visual><geometry><cylinder length="0.3" radius="0.08"/></geometry><material name="yellow"/></visual>
  </link>
  
  <link name="link5">
    <visual><geometry><cylinder length="0.2" radius="0.06"/></geometry><material name="purple"/></visual>
  </link>
<link name="link6">
    <visual><geometry><cylinder length="0.15" radius="0.04"/></geometry><material name="orange"/></visual>
  </link>
  
  <!-- 6轴关节，包含完整的limit属性 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0.3"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100.0" velocity="1.0"/>
  </joint>
<joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0.6"/><axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="100.0" velocity="1.0"/>
  </joint>
  
  <joint name="joint3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0 0 0.5"/><axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5" effort="80.0" velocity="1.0"/>
  </joint>
<joint name="joint4" type="revolute">
    <parent link="link3"/><child link="link4"/>
    <origin xyz="0 0 0.4"/><axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="60.0" velocity="1.5"/>
  </joint>
  
  <joint name="joint5" type="revolute">
    <parent link="link4"/><child link="link5"/>
    <origin xyz="0 0 0.3"/><axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="40.0" velocity="1.5"/>
  </joint>
<joint name="joint6" type="revolute">
    <parent link="link5"/><child link="link6"/>
    <origin xyz="0 0 0.2"/><axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="30.0" velocity="2.0"/>
  </joint>
</robot>
XML
echo "✅ 修复后的URDF文件创建完成: $URDF_FILE"

# 验证URDF文件
echo "验证URDF文件..."
if ros2 run urdf check_urdf "$URDF_FILE" > /dev/null 2>&1; then
    echo "✅ URDF验证成功!"
else
    echo "❌ URDF验证失败，显示错误:"
    ros2 run urdf check_urdf "$URDF_FILE"
    exit 1
fi
# 设置ROS环境
source install/setup.bash

echo "🚀 启动完整的CS625机器人可视化系统..."

# 启动Robot State Publisher
echo "启动Robot State Publisher..."
ros2 run robot_state_publisher robot_state_publisher "$URDF_FILE" &
RSP_PID=$!
echo "Robot State Publisher PID: $RSP_PID"
sleep 2

# 启动Joint State Publisher GUI（用于控制关节）
echo "启动Joint State Publisher GUI..."
ros2 run joint_state_publisher_gui joint_state_publisher_gui --ros-args -p source_list:="[/joint_states]" &
JSP_PID=$!
echo "Joint State Publisher GUI PID: $JSP_PID"
sleep 2
# 启动RViz
echo "启动RViz..."
rviz2 -d $(ros2 pkg prefix rviz2)/share/rviz2/default.rviz &
RVIZ_PID=$!
echo "RViz PID: $RVIZ_PID"
echo ""
echo "🎯 CS625机器人可视化系统已启动!"
echo "================================"
echo "📝 操作指南:"
echo "1. RViz窗口中:"
echo "   - 确保Fixed Frame设置为: base_link"
echo "   - 添加RobotModel显示（如果未自动添加）"
echo "2. Joint State Publisher窗口会出现"
echo "   - 拖动滑块控制机器人各个关节"
echo "   - 观察RViz中机器人的实时运动"
echo ""
echo "🔄 如果看不到机器人，请检查:"
echo "   - RViz中的Fixed Frame是否正确"
echo "   - 是否添加了RobotModel显示"
echo ""
echo "按Ctrl+C退出所有进程"
# 设置信号处理
cleanup() {
    echo "正在关闭进程..."
    kill $RVIZ_PID $RSP_PID $JSP_PID 2>/dev/null
    exit 0
}

trap cleanup INT TERM

# 等待进程
wait $RVIZ_PID
# 清理
cleanup
