#!/bin/bash
echo "=== ELI CS机器人基础功能测试 ==="
source install/setup.bash

echo "1. 测试机器人描述文件..."
if ros2 pkg prefix eli_cs_robot_description >/dev/null 2>&1; then
    echo "✅ 机器人描述包已加载"
    
    # 测试URDF生成
    echo "测试URDF生成..."
    urdf_file="/tmp/cs_test.urdf"
    if ros2 run xacro xacro $(ros2 pkg prefix eli_cs_robot_description)/share/eli_cs_robot_description/urdf/cs.urdf.xacro cs_type:=cs63 > "$urdf_file" 2>/dev/null; then
        if [ -s "$urdf_file" ]; then
            echo "✅ URDF生成成功 (文件大小: $(wc -l < "$urdf_file") 行)"
        else
            echo "❌ URDF生成失败"
        fi
    else
        echo "❌ Xacro处理失败"
    fi
else
    echo "❌ 机器人描述包未找到"
fi
echo ""
echo "2. 测试接口包..."
for pkg in eli_common_interface eli_dashboard_interface; do
    if ros2 interface list | grep -q "$pkg"; then
        echo "✅ $pkg 接口可用"
    else
        echo "❌ $pkg 接口不可用"
    fi
done
echo ""
echo "3. 测试启动文件..."
echo "测试机器人可视化启动文件 (不实际启动GUI)..."
timeout 5 ros2 launch eli_cs_robot_description view_cs.launch.py cs_type:=cs63 --noninteractive > /tmp/launch_test.log 2>&1 &
pid=$!
sleep 3
if kill -0 $pid 2>/dev/null; then
    echo "✅ 启动文件语法正确"
    kill $pid 2>/dev/null
else
    echo "❌ 启动文件有错误"
    cat /tmp/launch_test.log
fi
echo ""
echo "4. 测试ROS 2组件..."
components=$(ros2 component types | grep eli_ | wc -l)
echo "发现 $components 个ELI相关组件"

echo ""
echo "=== 测试完成 ==="
