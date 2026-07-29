# 工作日志：P1--P4 审计与 P5 观测模型修复

- 日期：2026-07-29
- 开发分支：`agent/p5-pose-aware-observation`
- 安全边界：所有编译和运行均为 headless Gazebo 与合成点云；未启动真实机械臂驱动、真实相机、夹爪 IO 或实体轨迹。

## 已审计的问题

P3 已有主检验失败（`r=-0.0325`），P4 完整设计也失败（`r=0.0401`，重遮挡为负）。对 P4 的不可变原始 CSV 进行附加诊断后，发现：

- 81.6% 的多视点转移融合后平移标准差低于 0.1 mm（重遮挡 99.3%）；
- P4 预测可见性与实际合成可见比例相关仅为 0.201；
- 旧合成相机仅以位置和点法线判断可见面，忽略候选姿态、成像平面和 z-buffer；P3/P4 的连续投影面积预测与该生成模型不匹配。

原始 P1--P4 数据、原来的预注册判定及其负结果均未修改。详细范围见 `docs/experiments/P1_P4_EVIDENCE_AUDIT_AND_OPTIMIZATION.md`；MATLAB 派生诊断输出在 Windows 的 `matlab_output/20260729_p4_failure_diagnostics/`。

## P5 修复

1. 新增 `VirtualObservationModel`：以完整候选 `SE(3)`、160×120 虚拟成像平面和 z-buffer 产生离散可见点。
2. 合成相机与候选预测器调用同一个曲面/投影模型；遮挡只作为事前已知场景条件的期望保留率。
3. 顺序协方差改为等权 covariance intersection，避免把相关 ICP bootstrap 估计当作独立测量。
4. 将已声明的 bootstrap 5 mm/0.02 rad 扰动作为协方差下限，防止数值重复性被报告为亚毫米校准。
5. 初始虚拟相机改为 look-at 目标；空首云改为最多等待 1 秒后以 `bootstrap_cloud_missing` 拒绝 episode，禁止产生假收敛数据。

## 验证

- 隔离 VM 构建目录 `/home/yff/codex_p5_build_20260729` 中，依赖链和 `cs625_nbv` 成功构建。
- VM 实际工作区 `/home/yff/elite_ros_ws` 中，`colcon build --packages-select cs625_nbv` 成功。
- 最终有效的纯虚拟 P5 smoke：`/home/yff/nbv_experiments/20260729_145017_fixed_order_virtual_cuboid_light_p5_smoke_0`。
  - 服务返回 `success=True`，2 个视点、42/42 候选可达；
  - 逐视点实际可见比例为 0.594 和 0.490；
  - 融合平移标准差为 8.66 mm，未出现 P4 的 <0.1 mm 塌缩。

该烟雾测试只证明 P5 数据通路、保护逻辑和数值下限可运行；P5 是否通过仍取决于预注册的 3 场景 × 10 匹配种子校准批次与 MATLAB 审计。
