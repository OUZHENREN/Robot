#!/bin/bash
echo "🎯 最简单的解决方案：直接启动CS625机器人"

# 重新编译（确保所有文件都是最新的）
colcon build --packages-select eli_cs_robot_description
source install/setup.bash
# 直接尝试启动（忽略任何Xacro错误）
echo "🚀 强行启动CS625机器人可视化..."
ros2 launch eli_cs_robot_description view_cs.launch.py cs_type:=cs625 || {
    echo "❌ 标准启动失败，尝试备选方法..."
    
    # 备选方法：直接启动RViz并手动配置
    echo "🎯 启动RViz，请手动配置..."
    rviz2 -d $(ros2 pkg prefix rviz2)/share/rviz2/default.rviz &
    sleep 2
    echo ""
    echo "📝 手动配置指南："
    echo "1. 点击左下角的 'Add' 按钮"
    echo "2. 选择 'RobotModel'"
    echo "3. 在右侧面板中："
    echo "   - 设置 Fixed Frame: base_link"
    echo "   - 设置 Description Topic: /robot_description"
    echo "4. 如果看不到模型，尝试添加其他显示类型"
    echo ""
    echo "按 Ctrl+C 退出"
    wait
}
