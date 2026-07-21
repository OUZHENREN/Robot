#!/bin/bash
echo "=== ELI CS机器人快速启动脚本 ==="
echo "选择要启动的模式:"
echo "1) 可视化机器人模型"
echo "2) 启动仿真环境"
echo "3) 启动MoveIt运动规划"
echo "4) 查看所有可用启动文件"
echo "5) 退出"

read -p "请输入选择 (1-5): " choice
case $choice in
    1)
        echo "启动机器人模型可视化..."
        source install/setup.bash
        ros2 launch eli_cs_robot_description view_cs.launch.py
        ;;
    2)
        echo "启动Gazebo仿真环境..."
        source install/setup.bash
        ros2 launch eli_cs_robot_simulation_gz cs_sim_control.launch.py
        ;;
    3)
        echo "启动MoveIt运动规划..."
        source install/setup.bash
        ros2 launch eli_cs_robot_moveit_config cs_moveit.launch.py
        ;;
    4)
        echo "=== 可用的启动文件 ==="
        find install -name "*.launch.py" | sort
        echo ""
        read -p "输入要启动的完整路径: " launch_file
        if [ -f "$launch_file" ]; then
            source install/setup.bash
            ros2 launch "$launch_file"
        else
            echo "错误: 文件不存在"
        fi
        ;;
    5)
        echo "退出"
        exit 0
        ;;
    *)
        echo "无效选择"
        exit 1
        ;;
esac
