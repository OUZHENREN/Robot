#!/bin/bash
# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 检查是否在正确的工作空间
check_workspace() {
    if [ ! -d "install" ] || [ ! -d "src" ]; then
        echo -e "${RED}错误: 请在elite_ros_ws工作空间目录中运行此脚本${NC}"
        echo -e "当前目录: $(pwd)"
        echo -e "请执行: ${YELLOW}cd ~/elite_ros_ws${NC}"
        return 1
    fi
    return 0
}
# 设置环境
setup_environment() {
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        return 0
    else
        echo -e "${RED}错误: 找不到环境设置文件${NC}"
        return 1
    fi
}
# 欢迎界面
welcome_screen() {
    clear
    echo -e "${CYAN}"
    echo "╔════════════════════════════════════════════════════════════╗"
    echo "║                  ELI CS625 机器人启动向导                  ║"
    echo "║                 Interactive Launch Wizard                 ║"
    echo "╚════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
    echo -e "${YELLOW}欢迎使用ELI CS625工业机器人可视化系统${NC}"
    echo -e "工作空间: $(pwd)"
    echo ""
}
# 显示系统状态
show_status() {
    echo -e "${BLUE}📊 系统状态检查:${NC}"
    
    # 检查环境设置
    if [ -n "$ROS_DISTRO" ]; then
        echo -e "  ✅ ROS2环境已设置 ($ROS_DISTRO)"
    else
        echo -e "  ❌ ROS2环境未设置"
    fi
# 检查进程
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
if pgrep -f "robot_state_publisher" > /dev/null; then
        echo -e "  ✅ 机器人状态发布器正在运行"
    else
        echo -e "  ❌ 机器人状态发布器未运行"
    fi
    echo ""
}
# 完整启动系统
launch_full_system() {
    echo -e "${CYAN}🚀 启动完整CS625机器人系统...${NC}"
    echo -e "${YELLOW}这将启动:${NC}"
    echo -e "  • RViz 3D可视化界面"
    echo -e "  • 关节状态控制面板"
    echo -e "  • 机器人状态发布器"
    echo ""
# 停止可能存在的旧进程
    pkill -f "ros2 launch" 2>/dev/null
    pkill -f "rviz2" 2>/dev/null
    pkill -f "joint_state_publisher" 2>/dev/null
    sleep 2
    
    # 启动系统
    ros2 launch eli_cs_robot_description view_cs.launch.py cs_type:=cs625 &
    LAUNCH_PID=$!
echo -e "${GREEN}✅ 系统已启动! PID: $LAUNCH_PID${NC}"
    echo -e "${YELLOW}RViz配置指南:${NC}"
    echo -e "  1. 设置Fixed Frame为: ${GREEN}base_link${NC}"
    echo -e "  2. 添加RobotModel显示"
    echo -e "  3. 使用关节控制窗口操作机器人"
    echo ""
    echo -e "按Ctrl+C停止所有进程"
    
    # 等待用户中断
    wait $LAUNCH_PID
}
# 仅启动RViz
launch_rviz_only() {
    echo -e "${CYAN}👁️ 启动RViz查看器...${NC}"
    
    # 停止可能存在的旧RViz进程
    pkill -f "rviz2" 2>/dev/null
    sleep 1
rviz2 -d $(ros2 pkg prefix rviz2)/share/rviz2/default.rviz &
    RVIZ_PID=$!
    
    echo -e "${GREEN}✅ RViz已启动! PID: $RVIZ_PID${NC}"
    echo -e "${YELLOW}配置指南:${NC}"
    echo -e "  1. 设置Fixed Frame: ${GREEN}base_link${NC}"
    echo -e "  2. 点击Add → 选择RobotModel → OK"
    echo ""
    echo -e "按Ctrl+C关闭RViz"
    
    wait $RVIZ_PI
}

# 启动简化版机器人
launch_simple_robot() {
    echo -e "${CYAN}🤖 启动简化版CS625机器人...${NC}"
    
# 创建简化URDF
    cat > /tmp/simple_cs625.urdf << 'XML'
<?xml version="1.0"?>
<robot name="simple_cs625">
  <link name="base_link">
    <visual><geometry><cylinder length="0.2" radius="0.3"/></geometry><material name="blue"/></visual>
  </link>
  <link name="link1"><visual><geometry><cylinder length="0.6" radius="0.15"/></geometry><material name="red"/></visual></link>
  <link name="link2"><visual><geometry><cylinder length="0.5" radius="0.12"/></geometry><material name="green"/></visual></link>
  <link name="link3"><visual><geometry><cylinder length="0.4" radius="0.1"/></geometry><material name="yellow"/></visual></link>
  
<joint name="joint1" type="revolute"><parent link="base_link"/><child link="link1"/><origin xyz="0 0 0.2"/><axis xyz="0 0 1"/></joint>
  <joint name="joint2" type="revolute"><parent link="link1"/><child link="link2"/><origin xyz="0 0 0.6"/><axis xyz="0 1 0"/></joint>
  <joint name="joint3" type="revolute"><parent link="link2"/><child link="link3"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint>
</robot>
XML
# 停止可能存在的旧进程
    pkill -f "robot_state_publisher" 2>/dev/null
    pkill -f "joint_state_publisher" 2>/dev/null
    pkill -f "rviz2" 2>/dev/null
    sleep 2
echo -e "${GREEN}启动组件...${NC}"
    ros2 run robot_state_publisher robot_state_publisher /tmp/simple_cs625.urdf &
    RSP_PID=$!
    sleep 1
ros2 run joint_state_publisher_gui joint_state_publisher_gui &
    JSP_PID=$!
    sleep 1
    
    rviz2 -d $(ros2 pkg prefix rviz2)/share/rviz2/default.rviz &
    RVIZ_PID=$!
echo -e "${GREEN}✅ 简化版机器人已启动!${NC}"
    echo -e "PID: RSP=$RSP_PID, JSP=$JSP_PID, RViz=$RVIZ_PID"
    echo -e "${YELLOW}在RViz中设置Fixed Frame为: ${GREEN}base_link${NC}"
    echo -e "按Ctrl+C退出"
    
    wait $RVIZ_PID
    kill $RSP_PID $JSP_PID 2>/dev/null
}
# 系统诊断
run_diagnostics() {
    echo -e "${CYAN}🔍 系统诊断...${NC}"
    echo ""
    
    # 检查ROS2环境
    echo -e "${YELLOW}ROS2环境检查:${NC}"
    if [ -n "$ROS_DISTRO" ]; then
        echo -e "  ✅ ROS_DISTRO: $ROS_DISTRO"
    else
        echo -e "  ❌ ROS环境未设置"
    fi
# 检查包
    echo -e "${YELLOW}包检查:${NC}"
    if ros2 pkg list | grep -q "eli_cs_robot_description"; then
        echo -e "  ✅ eli_cs_robot_description 包已安装"
    else
        echo -e "  ❌ eli_cs_robot_description 包未找到"
    fi
# 检查URDF文件
    echo -e "${YELLOW}文件检查:${NC}"
    if [ -f "src/eli_cs_robot_description/urdf/cs.urdf.xacro" ]; then
        echo -e "  ✅ URDF文件存在"
    else
        echo -e "  ❌ URDF文件不存在"
    fi
# 检查主题
    echo -e "${YELLOW}主题检查:${NC}"
    ros2 topic list | grep -E "(robot_description|joint_states)" | head -5 | while read topic; do
        echo -e "  📡 $topic"
    done
echo ""
    read -p "按Enter键继续..."
}
# 清理系统
cleanup_system() {
    echo -e "${YELLOW}🧹 清理系统进程...${NC}"
    pkill -f "ros2 launch" 2>/dev/null
    pkill -f "rviz2" 2>/dev/null
    pkill -f "joint_state_publisher" 2>/dev/null
    pkill -f "robot_state_publisher" 2>/dev/null
    sleep 2
    echo -e "${GREEN}✅ 清理完成${NC}"
    sleep 1
}
# 主菜单
main_menu() {
    while true; do
        welcome_screen
        show_status
        
        echo -e "${GREEN}请选择操作:${NC}"
        echo -e "  ${YELLOW}1${NC}. 🚀 完整启动官方CS625机器人"
        echo -e "  ${YELLOW}2${NC}. 🤖 启动简化版机器人(备用)"
        echo -e "  ${YELLOW}3${NC}. 👁️  仅启动RViz查看器"
        echo -e "  ${YELLOW}4${NC}. 🔍 系统诊断"
        echo -e "  ${YELLOW}5${NC}. 🧹 清理系统进程"
        echo -e "  ${YELLOW}0${NC}. ❌ 退出"
        echo ""
read -p "请输入选择 [0-5]: " choice
        
        case $choice in
            1) launch_full_system ;;
            2) launch_simple_robot ;;
            3) launch_rviz_only ;;
            4) run_diagnostics ;;
            5) cleanup_system ;;
            0) echo -e "${GREEN}再见！${NC}"; exit 0 ;;
            *) echo -e "${RED}无效选择，请重新输入${NC}"; sleep 2 ;;
        esac
    done
}
# 主程序
main() {
    # 检查工作空间
    if ! check_workspace; then
        exit 1
    fi
    
    # 设置环境
    if ! setup_environment; then
        echo -e "${RED}请先编译工作空间: ${YELLOW}colcon build${NC}"
        exit 1
    fi
    
    # 显示主菜单
    main_menu
}
# 启动主程序
main
