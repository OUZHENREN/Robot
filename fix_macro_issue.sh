#!/bin/bash
echo "🔧 修复Xacro宏未定义错误"

# 查找并检查Xacro文件
XACRO_FILE="install/eli_cs_robot_description/share/eli_cs_robot_description/urdf/cs.urdf.xacro"
MACRO_FILE="install/eli_cs_robot_description/share/eli_cs_robot_description/urdf/cs_macro.xacro"

echo "检查Xacro文件: $XACRO_FILE"
echo "检查宏文件: $MACRO_FILE"

# 检查文件是否存在
if [ ! -f "$XACRO_FILE" ]; then
    echo "❌ Xacro文件不存在"
    exit 1
fi

if [ ! -f "$MACRO_FILE" ]; then
    echo "❌ 宏文件不存在"
    exit 1
fi
echo "✅ 文件存在"

# 检查宏文件中是否定义了 eli_cs_robot 宏
echo "检查宏定义..."
if grep -q 'xacro:macro.*name="eli_cs_robot"' "$MACRO_FILE"; then
    echo "✅ eli_cs_robot 宏已定义"
else
    echo "❌ eli_cs_robot 宏未定义"
    echo "宏文件内容摘要:"
    grep -n "xacro:macro" "$MACRO_FILE" | head -10
fi
# 检查Xacro文件中的包含语句
echo "检查包含语句..."
grep -n "xacro:include" "$XACRO_FILE"

# 修复包含路径问题
echo "修复包含路径..."
# 备份文件
cp "$XACRO_FILE" "${XACRO_FILE}.backup"

# 修复包含路径，使用绝对路径或相对路径
sed -i 's#xacro:include filename="\$(find eli_cs_robot_description)/urdf/cs_macro.xacro"#xacro:include filename="cs_macro.xacro"#' "$XACRO_FILE"
echo "修复后的包含语句:"
grep -n "xacro:include" "$XACRO_FILE"
# 测试修复结果
echo "测试Xacro处理..."
source install/setup.bash
if ros2 run xacro xacro "$XACRO_FILE" cs_type:=cs625 > /dev/null 2>&1; then
    echo "✅ Xacro处理成功!"
else
    echo "❌ Xacro处理仍然失败，尝试其他修复方法"
    # 方法2: 使用绝对路径
    sed -i "s#xacro:include filename=\"cs_macro.xacro\"#xacro:include filename=\"$MACRO_FILE\"#" "$XACRO_FILE"
    
    if ros2 run xacro xacro "$XACRO_FILE" cs_type:=cs625 > /dev/null 2>&1; then
        echo "✅ 使用绝对路径修复成功!"
    else
        echo "❌ 绝对路径也失败，恢复备份并尝试其他方法"
        cp "${XACRO_FILE}.backup" "$XACRO_FILE"
    fi
fi
# 如果修复成功，重新编译
if ros2 run xacro xacro "$XACRO_FILE" cs_type:=cs625 > /dev/null 2>&1; then
    echo "🔄 重新编译包..."
    colcon build --packages-select eli_cs_robot_description
    source install/setup.bash
    
    echo "🚀 启动CS625机器人..."
    ros2 launch eli_cs_robot_description view_cs.launch.py cs_type:=cs625
else
echo "⚠️ 使用备选方案启动..."
    ./start_without_xacro.sh
fi
