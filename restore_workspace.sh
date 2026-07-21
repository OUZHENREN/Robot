#!/bin/bash
echo "🔄 恢复工作空间到原始状态"

# 备份当前有问题的文件
echo "备份当前文件..."
BACKUP_DIR="/tmp/eli_backup_$(date +%s)"
mkdir -p "$BACKUP_DIR"
cp -r src/eli_cs_robot_description/ "$BACKUP_DIR/" 2>/dev/null || echo "无文件可备份"
# 重新克隆或下载原始包
echo "重新获取原始包..."
cd src

# 检查是否有git仓库
if [ -d "eli_cs_robot_description/.git" ]; then
    echo "重置git仓库..."
    cd eli_cs_robot_description
    git checkout -- .
    git clean -fd
    cd ..
else
    echo "非git仓库，尝试其他方法"
fi
# 如果上面方法失败，创建基础文件结构
echo "创建基础文件结构..."
mkdir -p eli_cs_robot_description/urdf
mkdir -p eli_cs_robot_description/launch

# 创建最小可工作的Xacro文件
cat > eli_cs_robot_description/urdf/cs.urdf.xacro << 'XACRO'
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="cs">
  <xacro:arg name="name" default="cs"/>
  <xacro:arg name="cs_type" default="cs63"/>
<!-- 基础机器人结构 -->
  <link name="$(arg name)_base_link">
    <visual><geometry><cylinder length="0.3" radius="0.3"/></geometry></visual>
  </link>
  
  <!-- 根据类型创建不同配置 -->
  <xacro:if value="$(arg cs_type) == 'cs625'">
    <link name="$(arg name)_link1"><visual><geometry><cylinder length="0.8" radius="0.15"/></geometry></visual></link>
    <link name="$(arg name)_link2"><visual><geometry><cylinder length="0.7" radius="0.12"/></geometry></visual></link>
    <link name="$(arg name)_link3"><visual><geometry><cylinder length="0.6" radius="0.1"/></geometry></visual></link>
    <link name="$(arg name)_link4"><visual><geometry><cylinder length="0.4" radius="0.08"/></geometry></visual></link>
    <link name="$(arg name)_link5"><visual><geometry><cylinder length="0.3" radius="0.06"/></geometry></visual></link>
    <link name="$(arg name)_link6"><visual><geometry><cylinder length="0.2" radius="0.04"/></geometry></visual></link>
  </xacro:if>
<!-- 关节定义 -->
  <joint name="$(arg name)_joint1" type="revolute">
    <parent link="$(arg name)_base_link"/><child link="$(arg name)_link1"/>
    <origin xyz="0 0 0.3"/><axis xyz="0 0 1"/>
  </joint>
  
  <joint name="$(arg name)_joint2" type="revolute">
    <parent link="$(arg name)_link1"/><child link="$(arg name)_link2"/>
    <origin xyz="0 0 0.8"/><axis xyz="0 1 0"/>
  </joint>
  
  <!-- 更多关节... -->
</robot>
XACRO
# 创建launch文件
cat > eli_cs_robot_description/launch/view_cs.launch.py << 'PYTHON'
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': '''<?xml version="1.0"?>
<robot name="cs625">
<link name="base_link"><visual><geometry><box size="0.5 0.5 0.1"/></geometry></visual></link>
  <link name="link1"><visual><geometry><cylinder length="0.5" radius="0.1"/></geometry></visual></link>
  <joint name="joint1" type="revolute"><parent link="base_link"/><child link="link1"/><origin xyz="0 0 0.1"/><axis xyz="0 0 1"/></joint>
</robot>'''}]
),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])
PYTHON
cd ..

echo "重新编译..."
colcon build

echo "✅ 工作空间恢复完成"
