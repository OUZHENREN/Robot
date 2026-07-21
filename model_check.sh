#!/bin/bash
echo "=== ELI CS机器人型号配置检查 ==="
source install/setup.bash

config_dir=$(ros2 pkg prefix eli_cs_robot_description)/share/eli_cs_robot_description/config

echo "可用的机器人型号配置:"
for model in cs63 cs66 cs612 cs616 cs620 cs625; do
    model_dir="$config_dir/$model"
    if [ -d "$model_dir" ]; then
        echo ""
        echo "🔍 型号: $model"
        echo "  配置文件:"
        ls "$model_dir"/*.yaml 2>/dev/null | xargs -I {} basename {} | sed 's/^/     - /'
        
        # 检查关键文件
        for file in joint_limits.yaml physical_parameters.yaml; do
            if [ -f "$model_dir/$file" ]; then
                echo "    ✅ $file"
            else
                echo "    ❌ $file (缺失)"
            fi
        done
    else
        echo "❌ 型号 $model 配置目录不存在"
    fi
done
echo ""
echo "=== 配置检查完成 ==="
