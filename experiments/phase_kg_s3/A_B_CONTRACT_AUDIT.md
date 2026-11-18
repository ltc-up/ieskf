# KG-S3 总代理源码与数据合同审查（验收进行中）

本表区分实现合同与定位/实时资格。实现符合合同不等于统计、精度或生产GO；结果不得由参考选择。A/B仍不融合，生产仓库不修改，原研究基线不修改。

|审查项|源码与实际证据|当前资格|
|---|---|---|
|A2保留原NDT几何|A `patch_private_matcher.py::patch` 只在忽略的私有副本增加研究接口和受控pitch–pitch H符号开关；`score_service.cpp` 用原`cuda_detail::linearize`/activeTargetView、DIRECT7/q50/1.5m；原Align仍走默认false开关|原Score/g/Align/GLL保留；完整21项FD证据另存；不把H当统计信息|
|A2完整Prior与两初值|A `score_map.py::FrozenMap.value/align_start/solve`：`.5 zᵀP⁻¹z−S`，完整15D右误差；位姿J投影、二阶chart项、完整交叉相关；原Prior/rawAlign及分别refine共四候选|两图完整候选共同目标检查通过；beta唯一固定1|
|线搜索目标与扫描去重|`FrozenMap.refine` 用包含Prior的F作GLL Armijo；`ScoreMapObserver` 直接替代Pose观测，Root `causal_runner.run` 每Source单唯一observation_id|无预先融合Align；生命周期日志检查完整Prior hash/P/信息台账，不重复接纳相同扫描|
|数值域与协方差资格|`score_domain/certify_snap_path` 拒绝吸附区或无法证明合法的路径；`conditional_information` 正曲率条件尺度截顶，`chart_map`重置P|原Score吸附保留；不回退Legacy R；P为未校准条件模型，非LM逆，完整P/拒绝均保留|
|B0真实上一帧|B `bridge.py::ShadowGroup` 与局部history每次保存实际前Source之前的dense Deskew点云；拒绝后仍保存上一真实扫描，位姿取预测锚点|完整两图Legacy审查通过；并非固定三帧或裁剪Source拼接冒充B0|
|相同Source与正确物理ROI|`kinematic_causal_runner.py`/`kinematic_bridge.py` 复用global Source并校验payload；历史dense以自身O位姿变换，在重力对齐O执行120m/相对z[-10,30]/.8m|0105新旧完整7798帧Source/Deskew/global Prior+P/Legacy B0逐项一致；共用Deskew历史地图相关性仍明确存在|
|新独立运动预测|B `kinematic_predictor.py::IntervalKinematicPredictor` 真实gyro/Wheel、固定零bg、SE(3)指数ZOH；`kinematic_bridge` 固定一次H_init，无后验反馈|25/100ms held/header/available合同与数值测试通过；非完整15D LiDAR滤波；未添加滑移门限|
|数据延迟/缺口|HeaderCoverageAudit只用完整header元数据证明物理覆盖；真实motion值按available释放，DELAY仅下一实际输入时刻重试|0105 7798/7798提交、无终止；future值不提供运动；资源失败/延迟/物理gap分开|
|多帧/GLIM边界|B-M滚动、旧B-K关键帧、B-K-MR独立粗细NDT；signed-surface仅诊断不改原Score；原GLIM近期帧/关键帧/多分辨率源码已审|不是原生GLIM或GTSAM图；不能以接纳率宣称精度收益|
|输入/初始化与真实反馈|两图130–525s/135s一次合法初始化，完整原始身份一致；每A独立自身反馈，B连续O不被全局重置|已完成A0/A1/A2两图；新KIN0105完成、1013执行中|
|实时预算|因果串行完整CPU/GPU时长全部记账；实际wallclock新框架单worker，DDS域173/localhost隔离，GPU共享flock|串行不是实际20Hz/100Hz；A2含超50ms；实际RT排队未获资格|
|713/失败/存储|713完整130前史至724.9636已核验，不复用525成熟checkpoint或旧五帧；全部失败、超时与日志保留；SHA逐字节无损压缩|713定位待执行；总5GiB、A/B各2GiB，未向系统盘回退|

## 已证实的问题与剩余审查

A1_0105明显失稳且几何R几乎全受预冻结floor控制，不能作为方向性估计成功。A2的条件old21保留约1.976m修正，但两图连续工程精度没有稳定改善；原Score相对Prior未统计标定。历史C4_22约4.5m错误盆地的同帧Source/Target已恢复，但原成熟15D Prior/P缺失，不借用别帧、优化后P或新model冒充恢复状态。

KIN0105运动预测相邻误差显著改善，局部NDT匹配却从最早静止/低速阶段就产生垂向和roll错误，长期漂移严重；12个有效配对还显示ROI改变带来额外接纳差异。全段连续、相邻以及0.5/1s误差分别记录，参考不可用区间明确统计，不能以初值或接纳率改善掩盖匹配失败。

最终A/B/联合GO/PARTIAL/NO-GO待已批准连续长回放、实际时钟测试及独立复核完成后提交。本审查不批准新增几何模型、条件Prior或生产部署。
