# A1/A2 Shadow模型与验证结果

CONFIRMED：新增vertical_velocity.py只有观测函数/Jacobian，没有correct、ingest、ROS或状态提交接口。state的p/v是map中IMU原点，R是map←body，omega须先转body；r为base到IMU的body杆臂。实际诊断按原TF旋转Bosch测量，保留w=omega-bg及参考点转换。

A1：h=nᵀ(v−R(w×r))，固定map单位n，H=[0,nᵀ,nᵀR[w×r]×,−nᵀR[r]×,0]。
A2：h=e_zᵀ(Rᵀv−w×r)，H=[0,e_zᵀRᵀ,e_zᵀ[Rᵀv]×,−e_zᵀ[r]×,0]。
A2的n依赖state，不能用冻结n的A1 Jacobian。位置和ba无直接H，不等于全P下不会变化。

## 有限差分

32个合成状态，非零杆臂/bg、不同yaw/RP、侧滑和速度；A1法向独立包含坡/横坡。对每个模型全部15列，步长1e−4/1e−5/1e−6中心差分。

| 模型 | 最大绝对H误差 | 结果 |
|---|---:|---|
|A1|2.34486813e-09|PASS|
|A2|8.80501503e-08|PASS|

合成反例证明：A1若n取预测body up，与A2的h可以相同，但姿态H不同；斜坡上真实map-Vz非零而body法向速度可为零。测试没有把map-Vz清零。坏法向/缺来源拒绝，未静默归一化来源不明数据。

## 完整候选与P

shadow_candidate.py计算K、15D delta、Joseph P、右Jr reset及完整P差，返回committed=false和实际应用delta=0；不写输入state/P。带交叉项合成P通过SPD、Joseph恒等式、间接z/vz和输入不变测试。对所有冻结原输入检查未改变。

实数据可形成A假设候选的54帧：49 BE+5旧Z。A1只使用明确标注的“坐标map-up假设”，不是已知路面法向；A2只用body-up假设。实际R_A=0.25(m/s)²是预先声明的0.5m/s工程假设，不经数据调参，不称标定。54帧全部physical_fusion_eligible=false。

| 候选 | abs(dvz) P95 m/s | 最大m/s | 原域超限 |
|---|---:|---:|---:|
|A1|0.629201|0.942539|0|
|A2|0.241083|0.531090|1|

旧窗口A2提案dba范数0.382398m/s²，超过未改的0.3m/s²域。仍保存全部15D提案/P，不裁剪、不提交。达到域不代表其他提案物理可信。

UNKNOWN：gyro复用导致的state/测量噪声相关性、接触/悬架heave、body-road偏差、跳跃/路肩/颠簸和真实垂向参考。当前独立噪声Joseph候选只是条件代数模型，不是相关噪声已处理的后验。不能用小Vz、小P或平滑视为准确。

**短窗口A软融合：当前NO-GO。** 数学层通过，但法向/接触残差/噪声相关性先决条件不满足；继续Shadow或来源审查有价值，不自动开展12s或其他实际融合。

未归一化P变化Frobenius只作数值变化记录，不是跨单位的物理信息强度或模型优劣指标。
