#!/bin/bash

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 设置环境
source ~/elite_ros_ws/install/setup.bash
# 欢迎界面
welcome_screen() {
    echo -e "${CYAN}"
    echo "╔════════════════════════════════════════════════════════════╗"
    echo "║                  ELI CS625 机器人启动向导                  ║"
    echo "║                 Interactive Launch Wizard                 ║"
    echo "╚════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
    echo -e "${YELLOW}欢迎使用ELI CS625工业机器人可视化系统${NC}"
    echo -e "版本: 正式版 | 工作空间: ~/elite_ros_ws"
    echo ""
}
# 显示系统状态
show_status() {
    echo -e "${BLUE}📊 系统状态检查:${NC}"
    if ros2 topic list | grep -q "/robot_description"; then
        echo -e "  ✅ 机器人描述已发布"
    else
        echo -e "  ⚠️  机器人描述未发布"
    fi
    
    if pgrep -f "rviz2" > /dev/null; then
        echo -e "  ✅ RViz正在运行"
    else
        echo -e "  ❌ RViz未运行"
    fi
    
    if pgrep -f "joint_state_publisher_gui" > /dev/null; then
        echo -e "  ✅ 关节控制器正在运行"
    else
        echo -e "  ❌ 关节控制器未运行"
    fi
    echo ""
}
# 主菜单
main_menu() {
    while true; do
        clear
        welcome_screen
        show_status
        
        echo -e "${GREEN}请选择操作:${NC}"
        echo -e "  ${YELLOW}1${NC}. 🚀 完整启动CS625机器人 (RViz + 控制界面)"
echo -e "  ${YELLOW}2${NC}. 👁️  仅启动RViz查看器"
        echo -e "  ${YELLOW}3${NC}. 🎮  仅启动关节控制界面"
        echo -e "  ${YELLOW}4${NC}. ⚙️  高级选项"
        echo -e "  ${YELLOW}5${NC}. 🔧 工具和诊断"
        echo -e "  ${YELLOW}6${NC}. 📖 使用指南"
        echo -e "  ${YELLOW}0${NC}. ❌ 退出向导"
        echo ""
        read -p "请输入选择 [0-6]: " choice
case $choice in
            1) launch_full_system ;;
            2) launch_rviz_only ;;
            3) launch_controller_only ;;
            4) advanced_menu ;;
            5) tools_menu ;;
            6) show_guide ;;
            0) echo -e "${GREEN}再见！${NC}"; exit 0 ;;
            *) echo -e "${RED}无效选择，请重新输入${NC}"; sleep 1 ;;
        esac
    done
}
# 完整启动系统
launch_full_system() {
    echo -e "${CYAN}🚀 启动完整CS625机器人系统...${NC}"
    echo -e "${YELLOW}这将启动:${NC}"
    echo -e "  • RViz 3D可视化界面"
    echo -e "  • 关节状态控制面板"
    echo -e "  • 机器人状态发布器"
    echo ""
    echo -e "${GREEN}启动后请在RViz中:${NC}"
    echo -e "  1. 设置Fixed Frame为: ${YELLOW}base_link${NC}"
    echo -e "  2. 添加RobotModel显示"
    echo -e "  3. 使用关节控制窗口操作机器人"
    echo ""
    
    read -p "按Enter键继续，或按Ctrl+C取消..."
    
    # 启动系统
    ros2 launch eli_cs_robot_description view_cs.launch.py cs_type:=cs625 &
    LAUNCH_PID=$!
    
    echo -e "${GREEN}✅ 系统已启动! PID: $LAUNCH_PID${NC}"
    echo -e "按Ctrl+C停止所有进程"
# 等待用户中断
    wait $LAUNCH_PID
}

# 仅启动RViz
launch_rviz_only() {
    echo -e "${CYAN}👁️ 启动RViz查看器...${NC}"
    echo -e "${YELLOW}注意: 需要单独启动robot_state_publisher才能看到机器人${NC}"
    echo ""
rviz2 -d $(ros2 pkg prefix rviz2)/share/rviz2/default.rviz &
    RVIZ_PID=$!
    
    echo -e "${GREEN}✅ RViz已启动! PID: $RVIZ_PID${NC}"
    echo -e "${YELLOW}配置指南:${NC}"
    echo -e "  1. 设置Fixed Frame: base_link"
    echo -e "  2. 添加 → RobotModel"
    echo -e "  3. 确保robot_description主题正常"
    echo ""
    echo -e "按Ctrl+C关闭RViz"
    
    wait $RVIZ_PID
}
# 仅启动控制器
launch_controller_only() {
    echo -e "${CYAN}🎮 启动关节状态控制界面...${NC}"
    echo -e "${YELLOW}注意: 需要robot_state_publisher才能控制机器人${NC}"
    echo ""
    
    ros2 run joint_state_publisher_gui joint_state_publisher_gui &
    CONTROLLER_PID=$!
    
    echo -e "${GREEN}✅ 关节控制器已启动! PID: $CONTROLLER_PID${NC}"
    echo -e "拖动滑块控制机器人关节角度"
    echo ""
    echo -e "按Ctrl+C关闭控制器"
    
    wait $CONTROLLER_PID
}
# 高级菜单
advanced_menu() {
    while true; do
        clear
        echo -e "${PURPLE}⚙️  高级选项${NC}"
        echo ""
        echo -e "${GREEN}请选择:${NC}"
        echo -e "  ${YELLOW}1${NC}. 🔄 重启机器人系统"
        echo -e "  ${YELLOW}2${NC}. 📁 查看机器人文件"
        echo -e "  ${YELLOW}3${NC}. 🧪 测试机器人模型"
        echo -e "  ${YELLOW}4${NC}. 🌐 检查ROS2环境"
        echo -e "  ${YELLOW}5${NC}. 🖥️  启动简化版机器人"
        echo -e "  ${YELLOW}0${NC}. 🔙 返回主菜单"
        echo ""
        read -p "请输入选择 [0-5]: " choice
   case $choice in
            1) restart_system ;;
            2) show_robot_files ;;
            3) test_robot_model ;;
            4) check_ros_environment ;;
            5) launch_simple_robot ;;
            0) break ;;
            *) echo -e "${RED}无效选择${NC}"; sleep 1 ;;
        esac
    done
}
# 工具菜单
tools_menu() {
    while true; do
        clear
        echo -e "${PURPLE}🔧 工具和诊断${NC}"
        echo ""
        echo -e "${GREEN}请选择:${NC}"
        echo -e "  ${YELLOW}1${NC}. 📡 查看ROS2主题"
        echo -e "  ${YELLOW}2${NC}. ⚙️  查看ROS2参数"
        echo -e "  ${YELLOW}3${NC}. 🔍 检查TF框架"
        echo -e "  ${YELLOW}4${NC}. 📊 系统性能监控"
        echo -e "  ${YELLOW}5${NC}. 🐛 诊断问题"
        echo -e "  ${YELLOW}0${NC}. 🔙 返回主菜单"
        echo ""
        read -p "请输入选择 [0-5]: " choice
case $choice in
            1) show_ros_topics ;;
            2) show_ros_params ;;
            3) check_tf_frames ;;
            4) system_monitor ;;
            5) run_diagnostics ;;
            0) break ;;
            *) echo -e "${RED}无效选择${NC}"; sleep 1 ;;
        esac
    done
}
# 重启系统
restart_system() {
    echo -e "${YELLOW}🔄 停止现有进程...${NC}"
    pkill -f "ros2 launch" 2>/dev/null
    pkill -f "rviz2" 2>/dev/null
    pkill -f "joint_state_publisher" 2>/dev/null
    sleep 2
    
    echo -e "${GREEN}✅ 重启系统...${NC}"
    launch_full_system
}
# 显示机器人文件
show_robot_files() {
    echo -e "${CYAN}📁 机器人文件列表:${NC}"
    echo ""
    find ~/elite_ros_ws/src/eli_cs_robot_description -name "*.xacro" -o -name "*.yaml" -o -name "*.launch.py" | sort | head -15
    echo ""
    read -p "按Enter键继续..."
}
# 测试机器人模型
test_robot_model() {
    echo -e "${CYAN}🧪 测试URDF/Xacro文件...${NC}"
    XACRO_FILE="src/eli_cs_robot_description/urdf/cs.urdf.xacro"
    
    if ros2 run xacro xacro $XACRO_FILE cs_type:=cs625 > /tmp/cs625_test.urdf 2>&1; then
        echo -e "${GREEN}✅ Xacro文件处理成功!${NC}"
        echo -e "生成的URDF行数: $(wc -l < /tmp/cs625_test.urdf)"
        echo -e "前3行内容:"
        head -3 /tmp/cs625_test.urdf
    else
echo -e "${RED}❌ Xacro文件处理失败:${NC}"
        cat /tmp/cs625_test.urdf
    fi
    echo ""
    read -p "按Enter键继续..."
}
# 检查ROS环境
check_ros_environment() {
    echo -e "${CYAN}🌐 检查ROS2环境...${NC}"
    echo ""
    echo -e "ROS_DISTRO: $ROS_DISTRO"
    echo -e "工作空间: ~/elite_ros_ws"
    echo -e "已安装的ELI包:"
    ros2 pkg list | grep eli_ | nl
    echo ""
    read -p "按Enter键继续..."
}
# 启动简化版机器人
launch_simple_robot() {
    echo -e "${CYAN}🖥️  启动简化版机器人...${NC}"
    # 创建简化URDF
    cat > /tmp/simple_cs625.urdf << 'XML'
<?xml version="1.0"?>
<robot name="simple_cs625">
  <link name="base_link"><visual><geometry><box size="0.5 0.5 0.1"/></geometry></visual></link>
  <link name="link1"><visual><geometry><cylinder length="0.4" radius="0.08"/></geometry></visual></link>
  <link name="link2"><visual><geometry><cylinder length="0.3" radius="0.06"/></geometry></visual></link>
  <joint name="joint1" type="revolute"><parent link="base_link"/><child link="link1"/><origin xyz="0 0 0.1"/><axis xyz="0 0 1"/></joint>
  <joint name="joint2" type="revolute"><parent link="link1"/><child link="link2"/><origin xyz="0 0 0.4"/><axis xyz="0 1 0"/></joint>
</robot>
XML
ros2 run robot_state_publisher robot_state_publisher /tmp/simple_cs625.urdf &
    RSP_PID=$!
    ros2 run joint_state_publisher_gui joint_state_publisher_gui &
    JSP_PID=$!
    rviz2 -d $(ros2 pkg prefix rviz2)/share/rviz2/default.rviz &
    RVIZ_PID=$!
    
    echo -e "${GREEN}✅ 简化版机器人已启动!${NC}"
    echo -e "PID: RSP=$RSP_PID, JSP=$JSP_PID, RViz=$RVIZ_PID"
    echo -e "按Ctrl+C退出"
wait $RVIZ_PID
    kill $RSP_PID $JSP_PID 2>/dev/null
}
# 显示ROS主题
show_ros_topics() {
    echo -e "${CYAN}📡 ROS2主题列表:${NC}"
    echo ""
    ros2 topic list | grep -E "(robot|joint|tf)" | head -10
    echo ""
    read -p "按Enter键继续..."
}
# 显示ROS参数
show_ros_params() {
    echo -e "${CYAN}⚙️  ROS2参数列表:${NC}"
    echo ""
    ros2 param list | head -10
    echo ""
    read -p "按Enter键继续..."
}
# 检查TF框架
check_tf_frames() {
    echo -e "${CYAN}🔍 检查TF框架...${NC}"
    echo ""
    ros2 run tf2_tools view_frames 2>/dev/null
    if [ -f "frames.pdf" ]; then
        echo -e "${GREEN}✅ TF框架图已生成: frames.pdf${NC}"
    else
        echo -e "${RED}❌ 无法生成TF框架图${NC}"
    fi
    echo ""
    read -p "按Enter键继续..."
}
# 系统监控
system_monitor() {
    echo -e "${CYAN}📊 系统资源监控 (按Ctrl+C退出)${NC}"
    echo ""
    top -b -n 1 | head -10
    echo ""
    read -p "按Enter键继续..."
}
# 运行诊断
run_diagnostics() {
    echo -e "${CYAN}🐛 运行系统诊断...${NC}"
    echo ""
    
    # 检查关键进程
    echo -e "进程检查:"
    if pgrep -f "ros2" > /dev/null; then
        echo -e "  ✅ ROS2进程运行中"
    else
        echo -e "  ❌ 未发现ROS2进程"
    fi
# 检查主题
    echo -e "主题检查:"
    if ros2 topic list | grep -q "/robot_description"; then
        echo -e "  ✅ 机器人描述主题正常"
    else
        echo -e "  ❌ 机器人描述主题缺失"
    fi
echo ""
    read -p "按Enter键继续..."
}
# 使用指南
show_guide() {
    clear
    echo -e "${CYAN}📖 ELI CS625 机器人使用指南${NC}"
    echo ""
    echo -e "${GREEN}基本操作:${NC}"
    echo -e "  1. 启动完整系统后，RViz会自动打开"
    echo -e "  2. 关节控制窗口会出现，可以拖动滑块"
    echo -e "  3. 机器人模型会实时响应控制输入"
    echo ""
    echo -e "${GREEN}RViz配置:${NC}"
    echo -e "  • Fixed Frame: ${YELLOW}base_link${NC}"
    echo -e "  • 添加显示: RobotModel, TF, Axes等"
    echo -e "  • 保存配置避免重复设置"
    echo ""
echo -e "${GREEN}故障排除:${NC}"
    echo -e "  • 看不到机器人? 检查Fixed Frame设置"
    echo -e "  • 关节不运动? 确保所有进程正常启动"
    echo -e "  • 模型异常? 使用诊断工具检查URDF"
    echo ""
    echo -e "${GREEN}文件位置:${NC}"
    echo -e "  • URDF文件: 
~/elite_ros_ws/src/eli_cs_robot_description/urdf/"
    echo -e "  • 配置文件: ~/elite_ros_ws/src/eli_cs_robot_description/config/cs625/"
    echo ""
    read -p "按Enter键返回主菜单..."
}
# 主程序
main() {
    # 检查是否在正确的目录
    if [ ! -d "~/elite_ros_ws/install" ]; then
        echo -e "${RED}错误: 请在elite_ros_ws工作空间目录中运行此脚本${NC}"
        echo "当前目录: $(pwd)"
        exit 1
    fi
# 检查环境
    if ! source ~/elite_ros_ws/install/setup.bash 2>/dev/null; then
        echo -e "${RED}错误: 无法设置ROS2环境${NC}"
        exit 1
    fi
    
    main_menu
}
# 启动主程序
main
