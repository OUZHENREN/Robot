# 工作日志：虚拟 NBV 实验管线与数据导出

- 日期：2026-07-29
- 发布分支：`agent/virtual-nbv-p2-p3-exports`
- 合并基线：`main` 的 `a037eab`（`Add headless simulation and NBV smoke test`）
- 发布提交：`f24a600`（`Add virtual NBV experiment pipeline and data exports`）

## 本次完成内容

1. 将 NBV 编排器扩展为支持虚拟实验流程、策略比较与批量运行。
2. 增加合成相机/点云输入、视角相关观测处理、姿态/协方差估计和信息增益计算的接口衔接。
3. 增加实验结果记录与 `ExportExperimentData` 服务，供后续 MATLAB 分析与课题报告导出使用。
4. 更新 NBV 启动文件、参数、消息与服务定义，并保留无界面 Gazebo 启动支持。
5. 新增三个可复现实验脚本：
   - `tools/run_headless_regression.sh`
   - `tools/run_view_dependent_observation_test.sh`
   - `tools/run_virtual_baseline_batch.sh`

## 已完成验证

- 已在 Ubuntu VM 中完成纯虚拟、无硬件的编译与烟雾/批量测试。
- 已执行策略比较的纯虚拟批次；结果导出接口及 MATLAB 端分析流程已纳入当前工作流。
- 本次 Git 提交前已执行 `git diff --check`，未发现空白错误。

## 边界与后续事项

- 本次验证不包含真实机械臂、真实相机或实体环境；硬件接入应在独立的安全检查后进行。
- 课题报告使用导出数据时，应保留对应运行配置、随机种子和脚本版本，以保证可追溯性。
