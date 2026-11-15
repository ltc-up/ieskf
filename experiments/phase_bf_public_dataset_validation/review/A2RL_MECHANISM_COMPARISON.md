# 与既有A2RL证据对照

复用[BE5 Stage2A](../../phase_be5_stage2a/FROZEN_RESULTS_AND_VERTICAL_DECOMPOSITION.md)、[BD结论](../../phase_bd_ndt_health_bridge/FINAL_CONCLUSION.md)、[BE结论](../../phase_be_e0_e4/FINAL_CONCLUSION.md)、[AY结论](../../phase_ay_localization_solution/FINAL_CONCLUSION.md)，并只读49个BE配对冻结样本。没有重放A2RL或重新获取原生点云。

| 机制 | KITTI已有证据 | A2RL已有证据 | 推论边界 |
|---|---|---|---|
|数值收敛不等于正确匹配|64/65帧收敛但纵向比Seed更差|BD12632不同目标盆地均可正定；OFF/GLL/MT行为不同|不以Hessian正定、Fitness或终止单独选解|
|弱方向|末段主要纵向、支持下降，条件数不爆炸|BE Directional实际未触发有效弱轴抑制|不能照搬二元退化门限|
|Native方向性|46帧方差比中位[s,d,yaw]=[3.174,1.210,1.533]|49个BE为[9.562,9.446,0.570]|绝对尺度及方向信任跨场景不同|
|完整P的间接速度/姿态修正|断更前δv/Bias延续到最后11帧|Native可显著改变Vz候选、旋转候选|小增量不是精度保证|
|局部收益与正常段代价|XY改善、横向/Yaw退化|AY困难弯改善，正常8横向P95 0.344→0.437m|必须同时保护正常段|

相同prior候选：A2RL49例absΔVz P95 Legacy0.127759→Native0.023091m/s；KITTI46例则0.006406→0.007683m/s。A2RL absYaw候选P95约0.001043→0.002475rad；KITTI约0.000381→0.000291rad。Native不是统一缩小所有状态修正，P和完整R共同决定响应。49例49/49两模式原本均would-admit，不能称错误分支识别收益；右弯3/9、正常5/8各6例仅抽样，部分分组重叠，不能替代全路段性能。

旧Z记录4.35–6.27m/s为优化候选，不是已确认实际提交Vz跳变。部分历史分支时刻不同于最终发布时刻，不能混用branch.v推断车辆真实Vz。Stage2A候选全部Shadow，未融合；本次也没有新增状态提交。

历史DIRECT27“两点震荡、37→3”等叙述缺少本次可完整追溯证据，仍不据此确认KITTI有相同机制。BD21初值中OFF15/21、MT11/21、GLL16/21只说明步长策略可能改变盆地，不能据此当前改线搜索。当前审查aa50源码GLL3与旧c647 OFF必须分开，实际部署组件仍缺运行时证据。

A2RL各次运行点数、保存负载、实现阶段及主机计时合同不同，不能把其较低P95与KITTI直接相减后归因于算法或IO。BE多圈GMF/恢复的horizon失败及50ms失败仍保留；本复盘不恢复这些方案，也不新增分支。

结论：后续优先解释既有完整R/P及优化支持变化。当前证据不足以生产更换Native、加大Q、修改Bias、改变门控或新增匹配后端。
