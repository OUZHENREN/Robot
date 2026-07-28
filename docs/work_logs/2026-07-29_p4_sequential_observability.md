# 工作日志：P4 顺序状态融合与可观测性诊断

- 日期：2026-07-29
- 开发分支：`agent/p4-sequential-observability`
- 基线：`main@d38bad0`
- 安全边界：仅使用合成点云；未启动真实机械臂驱动、真实相机、夹爪 IO 或实体轨迹。

## 背景

P3 的两轮虚拟验证均未通过“不确定性代理与下一步虚拟真值误差下降一致”的门槛。P4 不将旧数据包装为正向结论，而是重新预注册顺序融合与可观测性模型的判定条件。

## 实施内容

1. 增加信息形式的局部 6D 顺序状态/协方差融合，并对协方差进行对称化和最小特征值正则化。
2. 候选评分加入可见投影面积与已执行视线新颖性的乘积；预测只使用已知虚拟几何和历史视线，不使用下一步真值误差。
3. 逐视点 CSV 新增先验标准差、预测后验标准差、融合后验标准差、视角新颖性和可观测性分数。
4. 新增 P4 预注册设计文档和 MATLAB 分析程序；MATLAB 强制检查 `none/light/heavy × 3 策略 × 每单元至少 10 个匹配种子` 的样本设计。

## 验证记录

- VM 隔离目录 `/home/yff/codex_p4_build_JlBQhF` 中，`colcon build --packages-select cs625_nbv` 成功。
- 纯合成、无 Gazebo 的 6 视点 `fixed_order` 烟雾测试成功，服务返回 `success=True`，并导出全部 P4 新字段。
- 烟雾数据已导出到 Windows：`report_exports/20260729_p4_runtime_smoke/`；MATLAB 输出在 `matlab_output/20260729_p4_runtime_smoke/`。
- 单次 smoke 仅有 5 个多视角转移，MATLAB 已正确判定 `Dataset meets preregistered design: NO` 与 `Decision: FAIL`。该输出仅证明接口、数值稳定性和拒绝逻辑可运行，不构成算法有效性结论。

## 后续步骤

在执行完整的三遮挡、三策略、匹配种子批次前，不得报告 P4 为通过或 PoseGain 为有效。完整批次必须使用本提交的配置快照和 MATLAB 脚本复核。
