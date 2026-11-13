# BD-P0 actual AY closed-loop results

All four runs completed requested frame counts, without per-lap reset, using each branch's own15D mean/P, original raw CUDA Deskew, persistent CUDA-NDT, native observation Header, measured delivery delay and bounded checkpoint/repropagation. AY optical-XY is reused byte-for-byte: no new velocity source, Kistler model tuning, Q/R change or GNSS feedback. Existing original main15D results remain frozen separately; do not equate the six-state ROS EKF with this research filter.

AY controls use original GLL3, step epsilon0.01, objective epsilon0.001, max100. Selected MT uses step epsilon0.001, objective termination disabled, max35. Actual selected MT iteration maxima22(single) and24(triple), so its cap never fires. This compares complete selected configurations; frozen13-way grid separates individual parameters. It does NOT attribute every closed-loop difference to More-Thuente alone.

|run|frames|valid75|XY RMSE75 m|corner3 latP95 m|corner9|normal5|normal8|
|---|---:|---:|---:|---:|---:|---:|---:|
|AY_lap|1468|1204|0.518380|0.567092|0.427361|0.419286|0.436535|
|AY_multilap|4134|3635|0.516548|0.600394|0.441564|0.395349|0.444984|
|MT001_lap|1468|1204|0.490126|0.567025|0.416206|0.425092|0.432512|
|MT001_multilap|4134|3635|0.487066|0.601176|0.432912|0.387663|0.441457|

Reference: raw GPS1/full-attitude engineering reference at exact native stamps; same fixed segment masks and same valid mask for compared runs. Historical75ms CSV was not resolved. The original AY private ledger was later located in the existing new-volume AY directory; new AY public0.518380 is not presented as an exact replay of published0.516665. Re-run AY native segment values closely reproduce the published segment results; raw-mask RMSE is0.518380. All0/75/150ms metrics, full segment lateral/longitudinal statistics and yaw errors are in JSON. The150ms ranking reverses: this remains conditional engineering evidence, not certified physical accuracy.

Selected MT improves aggregate75ms RMSE on both durations and improves corner9 and normal8 slightly on single lap; normal5 worsens. Corner3 is essentially unchanged. Therefore the no-regression success criterion is not established and this is NOT a production-qualified setting. More-Thuente's lower iteration count alone is not the acceptance criterion.

The first single-lap candidate run preceded baseline restart; triple runs partly overlapped. Hardware timing from overlapping runs is explicitly contaminated. Separate serial timing/parity execution is required; do not claim a hard20Hz pass from these timings. Full cycle maxima in these initial runs exceed50ms and all overruns remain included. No difficult frames were dropped.

## 原AY账本补充核验

迁移后找到既有新卷AY私有账本，原/本轮optical.json和optical_audit.json SHA完全相同。原AY与本轮AY1468帧逐值差异最大值：seed0.0146501（矩阵元素）、priorP3.82547e-5、NDT矩阵0.168198、delta0.0104230、K0.0283376；接纳决策全部一致。这些差异不等于地图坐标误差，不能把混合矩阵max称为欧氏位置RMSE。实际延迟/到达时序与重运行条件差异尚未逐事件完全归因，因此本轮重跑只作为并列控制，不能宣称冻结AY数值逐帧复现。75ms收益仍受参考掩码、重跑变动与未标定输入限制；生产资格结论保持NO-GO。
