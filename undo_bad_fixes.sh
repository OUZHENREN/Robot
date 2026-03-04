#!/bin/bash
echo "撤销错误的修复..."
find src -name "*.cpp" -exec sed -i 's/\[\[maybe_unused\]\] auto command_interfaces_.*\.set_value(.*);_result = //g' {} \;
find src -name "*.cpp" -exec sed -i 's/;_.*$//g' {} \;
echo "✓ 已撤销错误的修复"
