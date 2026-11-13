# 55案例冻结诊断与Z/Vz增量归因

CONFIRMED：首次54案例加一个不同的原始NDT离线代理困难帧，共55个不同案例；没有重跑前54、没有NDT/GICP/Deskew或因果回放。49个BE帧保留相同prior/P/header、base seed、NDT final、ID、原始BE实际R、原提交记录和发布时刻状态。Native只替换候选R；A独立提案，不先更新prior再做NDT。C无合格参考，不伪造输出。

## 可配对49 BE帧

| 指标 | 实际BE Legacy R候选 | Native Raw候选 |
|---|---:|---:|
|与原Legacy提案最大偏差|0|不适用（改变R）|
|原更新域would-admit|49/49|49/49|
| abs(dz) P50 m|0.005193|0.000629|
| abs(dz) P95 m|0.028695|0.005456|
| abs(dz) max m|0.477923|0.301731|
| abs(dvz) P50 m/s|0.005774|0.000536|
| abs(dvz) P95 m/s|0.127759|0.023091|
| abs(dvz) max m/s|0.269884|0.164500|
|本阶段实际提交|0|0|

注意：would-admit只是原ExternalPose域检查，不是新统计可信度或全系统接受率。所有49记录本来匹配有效。候选变小不能称为垂向精度提高，可能也削弱正确修正；没有独立Z/Vz真值。

右弯3/9、正常5/8各6帧（组可能重叠），Legacy/Native各6/6 would-admit；不能说整段横向P95通过或“保留全部正确横向修正”。相对高Fitness组8帧；本批fitness范围0.113980～0.561113，**没有f>3或f>15的实数据样本**，不能将相对高值称为历史异常门限覆盖。大Fitness解耦只由合成测试独立验证。

补充帧的原始NDT75ms XY代理误差约2.018m，是工程参考，不是确定错峰标签。两种R仍would-admit；Native不是错误匹配检测器。本次选样从后验轨迹代理改为额外核查原始匹配代理，单独保存selection审查和补充批；原54结果不覆盖。

## 垂向增量归因

每个BE候选保存完整6×6 R、H、S、15×6 K、15D delta、15×15 Joseph/reset前后P及P差。记录6个创新分量对Z/Vz的有符号K_ij r_j，及按P中position/velocity/theta/bg/ba列块分解；和与总delta逐项一致，vz最大和误差Legacy=5.55e-17、Native=2.78e-17。

这是冻结条件高斯增益分解，不是因果/统计校准证明。A也保存完整15D提案/P和同样块归因，相关IMU噪声未建模标志为false。Ptheta是右误差，不直接改名Euler RP协方差。

历史5个Z窗口保存原prior/fullP、A1/A2提案及旧Z1有符号pose/约束分解，不重做几何优化。4.351277和6.268822m/s仍是Y候选；V_returned单列，不改写成真实提交跳变。旧记录不具备配对BE Native测量，Native对照明确停止。12632 matcher-only记录缺相同时刻15D prior/P，所有候选更新停止。

实际旧BE更新记录保留原时刻；规范文件verified_55/cases.jsonl区分branch完整状态及branch.t，和最终发布的base位姿/P及row.state_stamp。45/49帧这两个时刻不同；原trace没有最终发布速度导出，标为UNKNOWN，绝不用较早branch.v顶替。初版错误字段命名原样保留为失败证据；规范化只改标签/补原始发布字段，全部R/K/delta/P候选逐值保持，无候选重算。scan-time提案、branch快照和最终发布状态不能混成同次实际修正。本阶段原输入state/P未改变。

## 计算时间

首48帧冻结代数P50/P95/P99/max约1.491/1.527/1.909/2.228ms；补充单帧约2.319ms。这不含完整文件读取、序列化/实际传播/点云/NDT，不能作为50ms全系统资格。短窗口、全圈误差、预测漂移、恢复和NEES均未测，状态为NOT_TESTED/UNKNOWN。
