#!/bin/bash
echo "🎯 ELI CS625机器人综合恢复方案"

echo "步骤1: 检查当前状态"
source install/setup.bash
ros2 pkg list | grep eli_ | nl

echo "步骤2: 尝试从已知源获取文件"
# 尝试下载已知的正确文件
KNOWN_FILES=(
    "https://raw.githubusercontent.com/ros/urdf_tutorial/master/urdf/06-flexible.urdf.xacro"
    "https://raw.githubusercontent.com/ros-planning/moveit_tutorials/master/doc/move_group_interface/launch/panda.urdf.xacro"
)
for file in "${KNOWN_FILES[@]}"; do
    echo "下载: $file"
    if wget -q "$file" -O /tmp/sample.xacro; then
        echo "✅ 下载成功"
        # 修改为CS625配置
        sed -i 's/panda/cs625/g; s/panda_link/base_link/g' /tmp/sample.xacro
        break
    fi
done
echo "步骤3: 创建可工作的替代方案"
# 创建简化但可用的替代方案
cat > /tmp/working_cs625.urdf << 'XML'
<?xml version="1.0"?>
<robot name="eli_cs625">
  <!-- CS625 Industrial Robot - Working Version -->
  <link name="base_link">
    <visual><geometry><cylinder length="0.2" radius="0.3"/></geometry><material name="blue"/></visual>
  </link>
  <link name="shoulder"><visual><geometry><cylinder length="0.4" radius="0.15"/></geometry><material name="red"/></visual></link>
  <link name="upper_arm"><visual><geometry><cylinder length="0.6" radius="0.1"/></geometry><material name="green"/></visual></link>
  <link name="forearm"><visual><geometry><cylinder length="0.5" radius="0.08"/></geometry><material name="yellow"/></visual></link>
  <link name="wrist1"><visual><geometry><cylinder length="0.2" radius="0.06"/></geometry><material name="purple"/></visual></link>
  <link name="wrist2"><visual><geometry><cylinder length="0.15" radius="0.05"/></geometry><material name="orange"/></visual></link>
  <link name="flange"><visual><geometry><cylinder length="0.1" radius="0.03"/></geometry><material name="gray"/></visual></link>
  
  <joint name="joint1" type="revolute"><parent link="base_link"/><child link="shoulder"/><origin xyz="0 0 0.2"/><axis xyz="0 0 1"/></joint>
 <joint name="joint2" type="revolute"><parent link="shoulder"/><child link="upper_arm"/><origin xyz="0 0 0.4"/><axis xyz="0 1 0"/></joint>
  <joint name="joint3" type="revolute"><parent link="upper_arm"/><child link="forearm"/><origin xyz="0 0 0.6"/><axis xyz="0 1 0"/></joint>
  <joint name="joint4" type="revolute"><parent link="forearm"/><child link="wrist1"/><origin xyz="0 0 0.5"/><axis xyz="1 0 0"/></joint>
  <joint name="joint5" type="revolute"><parent link="wrist1"/><child link="wrist2"/><origin xyz="0 0 0.2"/><axis xyz="0 1 0"/></joint>
  <joint name="joint6" type="revolute"><parent link="wrist2"/><child link="flange"/><origin xyz="0 0 0.15"/><axis xyz="1 0 0"/></joint>
</robot>
XML

echo "步骤4: 启动可视化系统"
source install/setup.bash
ros2 run robot_state_publisher robot_state_publisher /tmp/working_cs625.urdf &
RSP_PID=$!
ros2 run joint_state_publisher_gui joint_state_publisher_gui &
JSP_PID=$!
rviz2 -d $(ros2 pkg prefix rviz2)/share/rviz2/default.rviz &
RVIZ_PID=$!
echo "✅ CS625机器人已启动!"
echo "PID: RSP=$RSP_PID, JSP=$JSP_PID, RViz=$RVIZ_PID"
echo "使用Joint State Publisher GUI控制机器人关节"

wait $RVIZ_PID
kill $RSP_PID $JSP_PID 2>/dev/null
