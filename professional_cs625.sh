#!/bin/bash
echo "🏭 创建专业的CS625工业机器人模型"

URDF_FILE="/tmp/cs625_professional.urdf"

cat > "$URDF_FILE" << 'XML'
<?xml version="1.0"?>
<robot name="eli_cs625">
  <!-- 材料定义 -->
  <material name="metal"><color rgba="0.6 0.6 0.6 1.0"/></material>
  <material name="dark_metal"><color rgba="0.4 0.4 0.4 1.0"/></material>
  <material name="red_metal"><color rgba="0.7 0.2 0.2 1.0"/></material>
  <material name="blue_metal"><color rgba="0.2 0.4 0.8 1.0"/></material>
<!-- CS625 6轴工业机器人 - 专业版 -->
  <link name="world"/>
  
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.15" radius="0.35"/>
      </geometry>
      <material name="dark_metal"/>
    </visual>
  </link>
 <link name="pedestal_link">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.25"/>
      </geometry>
      <material name="metal"/>
    </visual>
  </link>
 <link name="shoulder_link">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.15"/>
      </geometry>
      <material name="red_metal"/>
    </visual>
  </link>
<link name="upper_arm_link">
    <visual>
      <geometry>
        <cylinder length="0.8" radius="0.1"/>
      </geometry>
      <material name="blue_metal"/>
    </visual>
  </link>
<link name="forearm_link">
    <visual>
      <geometry>
        <cylinder length="0.7" radius="0.08"/>
      </geometry>
      <material name="metal"/>
    </visual>
  </link>
<link name="wrist1_link">
    <visual>
      <geometry>
        <cylinder length="0.15" radius="0.06"/>
      </geometry>
      <material name="red_metal"/>
    </visual>
  </link>
<link name="wrist2_link">
    <visual>
      <geometry>
        <cylinder length="0.12" radius="0.05"/>
      </geometry>
      <material name="blue_metal"/>
    </visual>
  </link>
<link name="flange_link">
    <visual>
      <geometry>
        <cylinder length="0.08" radius="0.04"/>
      </geometry>
      <material name="dark_metal"/>
    </visual>
  </link>
<!-- 固定基座 -->
  <joint name="world_to_base" type="fixed">
    <parent link="world"/><child link="base_link"/>
    <origin xyz="0 0 0.075"/>
  </joint>
<!-- 6个旋转关节 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="pedestal_link"/>
    <origin xyz="0 0 0.225"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="150" velocity="1.5"/>
  </joint>
 <joint name="joint2" type="revolute">
    <parent link="pedestal_link"/><child link="shoulder_link"/>
    <origin xyz="0 0 0.4"/><axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="120" velocity="1.2"/>
  </joint>
<joint name="joint3" type="revolute">
    <parent link="shoulder_link"/><child link="upper_arm_link"/>
    <origin xyz="0 0 0.3"/><axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5" effort="100" velocity="1.0"/>
  </joint>
<joint name="joint4" type="revolute">
    <parent link="upper_arm_link"/><child link="forearm_link"/>
    <origin xyz="0 0 0.8"/><axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="80" velocity="2.0"/>
  </joint>
<joint name="joint5" type="revolute">
    <parent link="forearm_link"/><child link="wrist1_link"/>
    <origin xyz="0 0 0.7"/><axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="60" velocity="2.0"/>
  </joint>
<joint name="joint6" type="revolute">
    <parent link="wrist1_link"/><child link="wrist2_link"/>
    <origin xyz="0 0 0.15"/><axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="40" velocity="2.5"/>
  </joint>
 <joint name="flange_joint" type="fixed">
    <parent link="wrist2_link"/><child link="flange_link"/>
    <origin xyz="0 0 0.16"/>
  </joint>
</robot>
XML

echo "✅ 专业版CS625 URDF创建完成: $URDF_FILE"
# 验证URDF
if ros2 run urdf check_urdf "$URDF_FILE"; then
    echo "✅ URDF验证成功!"
    
    # 启动可视化
    source install/setup.bash
# 启动Robot State Publisher
    ros2 run robot_state_publisher robot_state_publisher "$URDF_FILE" &
    RSP_PID=$!
# 启动Joint State Publisher GUI
    ros2 run joint_state_publisher_gui joint_state_publisher_gui --ros-args -p source_list:="[/joint_states]" &
    JSP_PID=$!
# 启动RViz
    rviz2 -d $(ros2 pkg prefix rviz2)/share/rviz2/default.rviz &
    RVIZ_PID=$!
    
    echo "🎯 专业版CS625机器人已启动!"
    echo "固定帧(Fixed Frame)设置为: world"
    echo "使用Joint State Publisher GUI控制机器人关节"
# 等待
    wait $RVIZ_PID
    kill $RSP_PID $JSP_PID 2>/dev/null
else
    echo "❌ URDF验证失败"
fi
