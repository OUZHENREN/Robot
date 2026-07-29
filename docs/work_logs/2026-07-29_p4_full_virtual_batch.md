# 工作日志：P4 完整纯虚拟批次与预注册审计

- 日期：2026-07-29
- 代码提交：`98249d53d8cbfe54c8b1ae5b9d7bb52365474076`
- 场景：`virtual_cuboid_none_p4`、`virtual_cuboid_light_p4`、`virtual_cuboid_heavy_p4`
- 策略：`uncertainty_only`、`path_cost_only`、`pose_gain`
- 安全边界：仅无界面 Gazebo、合成点云和虚拟真值；未启动真实机械臂驱动、相机、夹爪 IO 或实体轨迹。

## 执行与数据完整性

虚拟机批次运行目录为：

- `/home/yff/nbv_p4_batches/20260729T051512Z`（无遮挡）
- `/home/yff/nbv_p4_batches/20260729T052523Z`（轻遮挡）
- `/home/yff/nbv_p4_batches/20260729T053242Z`（重遮挡）

三个批次的退出码均为 `0`，每个批次包含 30 份 `report_summary.csv`；共 90 个 episode。每个“场景 × 策略”单元具有 10 个匹配种子（625--634），满足 P4 预注册的最小样本设计。

原始 CSV 已导出到 Windows：
`Ubuntu_Share/report_exports/20260729_p4_full_virtual_batch/`。

## MATLAB 审计

使用 `matlab/analyze_nbv_p4_sequential_observability.m` 对三个批次根目录进行分析，输出位于：
`Ubuntu_Share/matlab_output/20260729_p4_full_virtual_batch/`。

预注册的通过条件为：总体 `r >= 0.30`、95% bootstrap CI 下界大于 0，且三个遮挡场景的相关方向均非负。

| 组别 | 多视点转移数 | Pearson r | 95% bootstrap CI |
| --- | ---: | ---: | --- |
| 全部 | 376 | 0.0401 | [-0.0188, 0.1153] |
| 无遮挡 | 116 | 0.0763 | [-0.2343, 0.3363] |
| 轻遮挡 | 120 | 0.0107 | [-0.0366, 0.1680] |
| 重遮挡 | 140 | -0.0679 | [-0.2171, 0.0499] |

结果：数据集**满足设计要求**，但统计判定为**失败**。预测的平移协方差标准差下降没有与下一步虚拟平移误差下降建立预注册要求的正相关；重遮挡条件方向为负。

## 结论与限制

本记录是负结果：不能据此宣称 P4 不确定性代理、PoseGain 或 NBV 在虚拟环境中已通过验证。结果仅适用于该合成、视角依赖观测模型；不构成真实相机、真实机械臂、CAD ADD/ADD-S 或因果性能结论。后续若继续研究，应在新的预注册设计中更换观测/误差代理，而不是放宽当前门槛或重解释该批次。
