#!/bin/bash
# ====================================================================================
# ==   艾利特 CS625 机器人与视觉系统 - 统一启动脚本 (v11 - 采用Marker显示方案)      ==
# ====================================================================================

echo "--- [步骤 1/5] 正在自动清理所有旧的ROS及相关进程..."
# 清理所有我们自定义的节点
pkill -f "ros2 run tcp_bridge" # [简化] 清理所有tcp_bridge包下的节点
pkill -f "ros2 run vision_bridge tcp_pose_receiver"

# 清理机器人驱动相关的进程
killall -q -w -9 eli_ros2_control_node rviz2 spawner controller_stopper_node robot_state_publisher eli_components_loader 2>/dev/null

# 重置ROS 2守护进程
ros2 daemon stop > /dev/null 2>&1 && ros2 daemon start > /dev/null 2>&1
echo "--- 清理完成。等待系统资源释放..."
sleep 2

# --- 分割线 ---
echo ""
echo "--- [步骤 2/5] 正在加载ROS2工作空间环境..."
source /opt/ros/jazzy/setup.bash
source ~/elite_ros_ws/install/setup.bash
echo "--- 环境加载成功！"
echo ""

# --- 分割线 ---
echo "--- [步骤 3/5] 正在后台启动机器人通信与视觉节点..."

# 0. TCP Server (S/R消息总入口)
gnome-terminal --title="[TCP Server] S->Robot, R->Model" -- bash -c "\
source ~/elite_ros_ws/install/setup.bash; \
echo '--- 启动TCP服务 (端口:9999)...'; \
ros2 run tcp_bridge tcp_server; \
exec bash" &

# 1. Robot Commander
gnome-terminal --title="[Robot Commander] To Robot" -- bash -c "\
source ~/elite_ros_ws/install/setup.bash; \
echo '--- 启动机器人指令节点...'; \
ros2 run tcp_bridge robot_commander; \
exec bash" &

# 2. Pose to TF Broadcaster (RViz中显示机器人目标点)
gnome-terminal --title="[TF Broadcaster]" -- bash -c "\
source ~/elite_ros_ws/install/setup.bash; \
echo '--- 启动 Pose-to-TF 广播器...'; \
ros2 run tcp_bridge pose_to_tf_broadcaster; \
exec bash" &

# 3. [核心新增] GLB 模型显示节点
gnome-terminal --title="[Model Publisher]" -- bash -c "\
source ~/elite_ros_ws/install/setup.bash; \
echo '--- 启动GLB模型显示节点...'; \
ros2 run tcp_bridge static_model_publisher; \
exec bash" &

echo "--- 4个核心节点已在新的终端窗口中启动。"
echo ""

# --- 分割线 ---
echo "--- [步骤 4/5] 正在启动机器人主驱动与RViz..."
echo "--- 等待5秒，确保核心节点已准备就绪..."
sleep 5

ros2 launch eli_cs_robot_driver elite_control.launch.py robot_ip:=192.168.1.200 cs_type:=cs625

# --- 脚本执行结束 ---
echo ""
echo "--- [步骤 5/5] 主启动脚本已执行完毕。"
read -p "--- 按下 [Enter] 键即可关闭此主终端窗口。"

