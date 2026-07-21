#!/bin/bash
echo "=== ELI CS机器人快速启动脚本 (修复版) ==="
echo ""

# 设置环境变量
source install/setup.bash

# 查询并显示可用型号
echo "可用的机器人型号: cs63, cs66, cs612, cs616, cs620, cs625"
read -p "请输入机器人型号 (默认: cs63): " cs_type
cs_type=${cs_type:-cs63}
# 验证型号是否有效
valid_types=("cs63" "cs66" "cs612" "cs616" "cs620" "cs625")
if [[ ! " ${valid_types[@]} " =~ " ${cs_type} " ]]; then
    echo "警告: 型号 '$cs_type' 可能无效，使用默认值 cs63"
    cs_type="cs63"
fi
echo ""
echo "选择要启动的模式:"
echo "1) 可视化机器人模型 (型号: $cs_type)"
echo "2) 启动仿真环境 (型号: $cs_type)" 
echo "3) 启动MoveIt运动规划 (型号: $cs_type)"
echo "4) 启动真实机器人控制"
echo "5) 查看所有可用启动文件"
echo "6) 退出"

read -p "请输入选择 (1-6): " choice
case $choice in
    1)
        echo "启动机器人模型可视化... 型号: $cs_type"
        ros2 launch eli_cs_robot_description view_cs.launch.py cs_type:=$cs_type
        ;;
    2)
        echo "启动Gazebo仿真环境... 型号: $cs_type"
        ros2 launch eli_cs_robot_simulation_gz cs_sim_control.launch.py cs_type:=$cs_type
        ;;
    3)
        echo "启动MoveIt运动规划... 型号: $cs_type"
        ros2 launch eli_cs_robot_moveit_config cs_moveit.launch.py cs_type:=$cs_type
        ;;
    4)
        echo "启动真实机器人控制..."
        ros2 launch eli_cs_robot_driver elite_control.launch.py
        ;;
    5)
        echo "=== 所有可用的启动文件 ==="
        find install -name "*.launch.py" | sort | nl
        echo ""
        read -p "输入要启动的文件编号 (或输入q退出): " file_num
        
        if [ "$file_num" = "q" ]; then
            exit 0
        fi
launch_files=($(find install -name "*.launch.py" | sort))
        if [ -n "$file_num" ] && [ "$file_num" -gt 0 ] && [ "$file_num" -le "${#launch_files[@]}" ]; then
            launch_file="${launch_files[$((file_num-1))]}"
            echo "启动: $launch_file"
# 检查是否需要cs_type参数
            if [[ "$launch_file" == *"description"* ]] || [[ "$launch_file" == *"moveit"* ]] || [[ "$launch_file" == *"simulation"* ]]; then
                ros2 launch "$launch_file" cs_type:=$cs_type
            else
                ros2 launch "$launch_file"
            fi
else
            echo "无效的文件编号"
        fi
;;
    6)
        echo "退出"
        exit 0
        ;;
    *)
        echo "无效选择"
        exit 1
        ;;
esac
