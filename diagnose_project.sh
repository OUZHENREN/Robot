#!/bin/bash

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}===========================================${NC}"
echo -e "${BLUE}🤖 ELI CS625 项目健康诊断开始...${NC}"
echo -e "${BLUE}===========================================${NC}"

# 1. 环境检查
echo -e "\n${YELLOW}[1/5] 检查ROS2环境和路径...${NC}"
if [ -n "$ROS_DISTRO" ]; then
    echo -e "  ${GREEN}✅ ROS_DISTRO: $ROS_DISTRO${NC}"
else
    echo -e "  ${RED}❌ ROS2环境未设置! 请先执行: source /opt/ros/$ROS_DISTRO/setup.bash${NC}"
fi
echo "  -> 当前目录: $(pwd)"
if [[ "$(pwd)" != "$HOME/elite_ros_ws" ]]; then
    echo -e "  ${RED}❌ 警告: 脚本不在 '~/elite_ros_ws' 目录中运行!${NC}"
fi

# 2. 文件结构完整性检查
echo -e "\n${YELLOW}[2/5] 检查项目文件结构...${NC}"
REQUIRED_FILES=(
    "src/eli_cs_robot_description/package.xml"
    "src/eli_cs_robot_description/launch/view_cs.launch.py"
    "src/eli_cs_robot_description/urdf/cs.urdf.xacro"
    "src/eli_cs_robot_description/config/cs625/joint_limits.yaml"
)
ALL_OK=true
for file in "${REQUIRED_FILES[@]}"; do
    if [ -f "$file" ]; then
        echo -e "  ${GREEN}✅ 找到关键文件: $file${NC}"
    else
        echo -e "  ${RED}❌ 缺失关键文件: $file${NC}"
        ALL_OK=false
    fi
done
if $ALL_OK; then
    echo -e "  ${GREEN}项目文件结构看起来完整。${NC}"
else
    echo -e "  ${RED}项目文件缺失! 这可能是问题的主要原因。${NC}"
fi

# 3. 检查Xacro文件语法
echo -e "\n${YELLOW}[3/5] 测试机器人模型(Xacro)文件语法...${NC}"
XACRO_FILE="src/eli_cs_robot_description/urdf/cs.urdf.xacro"
if [ -f "$XACRO_FILE" ]; then
    if ros2 run xacro xacro "$XACRO_FILE" cs_type:=cs625 > /tmp/xacro_test.urdf 2>/tmp/xacro_error.log; then
        echo -e "  ${GREEN}✅ Xacro文件语法正确! 生成的URDF大小: $(wc -c < /tmp/xacro_test.urdf) bytes.${NC}"
    else
        echo -e "  ${RED}❌ Xacro文件存在语法错误! 错误详情:${NC}"
        echo "----------------- XACRO ERROR -----------------"
        head -10 /tmp/xacro_error.log
        echo "-----------------------------------------------"
    fi
else
echo -e "  ${RED}❌ 无法测试，因为找不到 $XACRO_FILE${NC}"
fi


# 4. 尝试重新编译 (最关键的诊断步骤)
echo -e "\n${YELLOW}[4/5] 尝试重新编译工作空间...${NC}"
echo "此步骤将暴露所有编译时错误。日志将保存到 build_log.txt"
if colcon build --symlink-install --event-handlers console_direct+ > build_log.txt 2>&1; then
    echo -e "  ${GREEN}✅ 编译成功! 项目工程文件应该没有问题。${NC}"
else
    echo -e "  ${RED}❌ 编译失败! 这是问题的关键所在。请查看下面的错误摘要和'build_log.txt'文件。${NC}"
    echo "----------------- BUILD ERROR SUMMARY -----------------"
    grep -i -E "error|fail|abort" build_log.txt | head -10
    echo "-------------------------------------------------------"
fi
# 5. 尝试启动
echo -e "\n${YELLOW}[5/5] 尝试启动项目 (如果编译成功)...${NC}"
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "正在尝试启动... 日志将保存到 launch_log.txt"
    ros2 launch eli_cs_robot_description view_cs.launch.py cs_type:=cs625 > launch_log.txt 2>&1 &
    LAUNCH_PID=$!
    sleep 5 # 等待几秒钟让进程启动或失败
    if ps -p $LAUNCH_PID > /dev/null; then
        echo -e "  ${GREEN}✅ 启动进程已运行 (PID: $LAUNCH_PID)。可能没有问题。${NC}"
        kill $LAUNCH_PID
    else
        echo -e "  ${RED}❌ 启动失败! 进程未能成功启动。请检查'launch_log.txt'中的错误。${NC}"
        echo "----------------- LAUNCH ERROR SUMMARY ----------------"
        head -15 launch_log.txt
        echo "--------------------------------------------------------"
    fi
else
    echo -e "  ${YELLOW}⚠️ 跳过启动测试，因为编译未完成。${NC}"
fi

echo -e "\n${BLUE}===========================================${NC}"
echo -e "${BLUE}📄 诊断完成!${NC}"
echo -e "${BLUE}===========================================${NC}"
echo -e "\n${GREEN}请将上面的【所有输出】以及【build_log.txt】和【launch_log.txt】文件的内容发送给我进行分析。${NC}"
