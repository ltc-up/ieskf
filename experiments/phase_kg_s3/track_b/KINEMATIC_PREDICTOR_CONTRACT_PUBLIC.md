# Approved contract record (de-identified)

The user approved all three decisions and implementation of this eight-item contract. The original local review copy is preserved unchanged outside this commit; only absolute machine prefixes in its source appendix are abbreviated here. Original approved copy SHA256: `969901871464091466a5bd2f09a74a1d838ab584102dbdba73917d6d282f0717`.

The original pre-approval wording below records the decision being approved; its historical “not implemented / wait for confirmation” status no longer describes current work. The subsequently approved delay-versus-physical-gap scheduling clarification is recorded in [KINEMATIC_DELAY_SCHEDULING.md](KINEMATIC_DELAY_SCHEDULING.md) and governs explicit retry availability. The new implementation is CPU validated; GPU geometric/continuous and real-time qualification remain pending.

---

# B_KINEMATIC_PREDICTOR_CONTRACT — 待用户确认，禁止据此直接实施

本合同仅提出新预测器定义，不修改现冻结算法、数据或结果。原版保留为 **Legacy B-Prediction**。双轴命名：`predictor=LEGACY_15D_DIFF / KINEMATIC_BK`，`target=PREVIOUS / ROLLING / KEYFRAME`；旧运行的 `B-K` 标签始终表示 **LEGACY_15D_DIFF + KEYFRAME Target**，不得改名覆盖。下文用户所称“新 B-K”仅指 `KINEMATIC_BK` 预测器。

## 1. 3D姿态与平移积分

每个 `[t_prev,t_ref]` 相对积分独立从 `ΔT=I` 开始；已提交旧区间不被后续数据重写。采用完整3D角速度与base前向速度，body twist为 `ω=R_base_imu ω_imu−b_g0`、`v_body=[s_wheel,0,0]ᵀ`。在IMU/Wheel测量header的并集处分段，各段零阶保持：

`R⁺=R Exp([ω]× Δt)`；`p⁺=p+R J_l(φ) v_body Δt`，`φ=ωΔt`；

`J_l(φ)=I+(1−cosθ)/θ²[φ]×+(θ−sinθ)/θ³[φ]×²`，`θ=||φ||`，零附近用同式Taylor展开。即常值body twist的SE(3)指数积分，不退化为平面Yaw积分，不积分加速度。最终 `ΔU_kin=ΔT`，局部原式 `O_pred=O_prev ΔU_kin` 不变。若需接口累计量，仅令 `U_kin(135s)=I`、`U_kin←U_kin ΔU_kin`；其差分不得夹带任何观测位置修正。

## 2. 时间、外参与杆臂

查询使用当前LiDAR的原Deskew reference `t_ref` 和真实cloud available `a_ref`。只允许 `available≤a_ref` 的原始IMU/Wheel，且各积分时刻仅使用 `header≤该时刻` 的样本；未来available即使header更早也禁止。每段用最新合法已到达测量ZOH，不使用未来样本插值、GNSS、当前global NDT final或B配准后验。相同header按原IMU→Wheel→clock/消息ID顺序处理。已提交相对区间不因后来迟到输入返写；当前区间可使用截至当前available已经到达、header位于区间前的合法held样本。

gyro沿原实际外参只做IMU→base旋转；角速度不做平移杆臂修正。生产 `wheel_velocity/src/wheel_velocity.cpp::model/timer_callback` 已由轮速、转角及 `/a2rl/vn/imu_2_eagleye` 的VN角速度按轮位校正到base，再发布 `header.frame_id=base_link`；其 `linear.x` 形如 `v_fl cos(α·steering)−ω_VN·l_b`，`linear.y` 也存在（转角及轮位模型生成），并非没有侧向量。新方案按用户指定仅取已记录 `linear.x`，明确忽略其 `linear.y`，不重新用Bosch角速度修正已校正的Wheel x。Bosch只用于新姿态积分。现15D `WheelObservation.model` 的 `Rᵀv_IMU−ω×lever_base_imu` 与之对应，故这里直接把该topic的 `linear.x` 作为 **base前向速度**，不再重复扣除IMU杆臂。新模型不使用加计，不新增角加速度杆臂补偿。Wheel topic的header由上游timer的now生成，存在VN/轮速缓存及发布延迟，不能声称等于轮硬件原始测量时刻；本轮沿用录包header/available合同，物理延迟标定仍UNKNOWN。若核验实际录包测量点/轴与此定义不符，停止受影响分支，先询问，不能自行移植或拟合外参。

## 3. gyro Bias、初始化与坐标轴

`b_g0` 固定为原135s合法初始化使用的base轴零均值 `[0,0,0] rad/s`（原初始化记录声明无成熟Bias/未来拟合，原runner `State(...,zeros(3),zeros(3),init)`）。不从Legacy/global/B后验读取Bias，不在线估计或按参考修正；残余gyro Bias作为未解决误差报告。相对预测器 `U_kin(135s)=I` 不需要历史地图姿态来修正运动。

**保持原O轴的一次性初始化提议，须用户明确确认：**现源码实际在首个合法扫描 `firstRef` 取当时独立pure U，去yaw/平移得到 `O0_legacy`，并非直接取135s记录mean的roll/pitch。其来源是合法135s初始化及截至首cloud available已经到达的独立IMU/Wheel前史，不含global LiDAR更新。新对照仅一次沿用这一 `O0_legacy`，设 `H_init=O0_legacy inv(U_kin(firstRef))`，接口输入为 `H_init U_kin`；这样首个O与原版完全一致，保留原Target/ROI坐标轴，而不是把首O置I丢掉tilt。

`H_init` 此后永不更新，且 `inv(H_init U_prev) (H_init U_cur)=ΔU_kin`；它只是一次初始化相关性，不接入Legacy后续运动增量、velocity、Bias或任何map姿态校正。原135s记录mean仍有一次历史地图规范来源，不能声称完全无地图前史。若用户不接受这一首帧轴继承，应另行批准新的O轴定义，不能同时声称原轴不变。

## 4. body横向/竖向速度零的适用性

`v_body_y=v_body_z=0` 是非完整运动学假设，不是强制map/O的z不变：3D姿态、坡度、俯仰和侧倾仍会使世界三轴位置变化。赛车侧滑、悬架/跳动、路肩、侧向速度及Wheel滑移可能破坏该假设；gyro积分也会受Bias影响。不声称其优于完整15D物理模型，不用低残差/SPD/平滑证明假设成立，必须在高速弯道和历史BB异常单独报告。

## 5. 缺失、滑移及异常的明确行为

继承真实IMU间隔/held上限 **25ms**、Wheel held测量年龄上限 **100ms**，在每段末检查，不得无限hold。测量需finite、Wheel variance finite且≥0、单位/轴/时间戳合法。任何区间不能满足合法输入时，输出 `KINEMATIC_INTERVAL_INVALID`，不伪造Δ、不填0速度、不把旧O改时间戳称新状态，也不退回global/Legacy后验。为保留严格上一帧Target的合法锚点，拟议第一版在首个不可恢复运动缺口终止该新预测器分支并保留前缀，A和原版分支继续；不自动恢复或重初始化。此失败行为也需本合同确认。

旧15D的 `ZERO_CONFLICT`（近零Wheel与模型预测冲突）和 `5m/s residual`（Wheel与完整状态模型之差）依赖15D预测，**不能原样继承为无该模型的新门限，也不能偷换成相邻Wheel跳变门限**。拟议第一版不新增速度/加速度数值门限，不伪造可靠侧滑检测；上游model给出的linear.y不能独立认证真实侧滑；外观合法但实际滑移的Wheel可能照常进入预测，标记 `SLIP_NOT_OBSERVABLE_FROM_THIS_TOPIC`，由现有配准工程guard保留接受/拒绝诊断，不能称已识别滑移。若需要新异常门限、传感器状态来源或失效后恢复策略，须另有明确证据及用户确认。

## 6. 与原15D后验差分的实际区别及诊断证据

原版 `ΔU_old=inv(U_15D_prev) U_15D_cur` 包含Wheel完整 `K=P Hᵀ/S` 对位置/姿态/Bias的瞬时条件均值修正；它不等同于单纯物理运动积分。新方案仅使用gyro和Wheel速度的时间积分，无15D P、Kalman更新或新置信协方差，避免把该类位置重定位解释为车体位移。

独立CPU重放相同原始IMU/Wheel CDR、135s初始化、外参、测量/available时序和cloud clock：1000个合法帧的U与原logged U逐元素差 **0**；5007条Wheel消息、11300次含回滚的step调用。各消息最终重算的Wheel base位置修正P99 **1.643m**、max **49.968m**，姿态修正max **1.555°**。最大50ms区间旧U位移 **93.713m**（1874.257m/s）；逐事件分解中观测修正向量norm **93.184m**，过程传播速度全前缀max **23.984m/s**，分解残差max **8.9e−16m**。这是后验位置修正污染运动增量的实证，不是车辆真实速度认证。

同时核验“旧保存上一端点”与“当前历史revision上一端点”：位置差max **1.27e−13m**，两种相对增量逐项差 **0**。因此本前缀不能归咎于混用revision，单换同revision端点不足以修复。原版失败对照及全部proposal保留，不改原15D算法，也不把本诊断包装成新预测器已通过。

## 7. 相同Source/Target的配对实验与归因

三LiDAR融合/原Deskew/当前Source矩形/.8m/5000点、真实dense历史Target、120m圆及相对z[-10,30]、原坐标定义、独立O、不使用global final、CUDA NDT原目标/分辨率/q50/GLL3/step/max与既有guard全部不变。新预测器只改变因果运动增量及由此得到的O预测；原版与新版使用独立状态/输出目录，旧名称/数据不覆盖。

必须区分两类证据：①条件配对诊断固定同帧Source、历史真实云/锚点与Target数组，比较两种seed的预测误差和配准结果；可用两种因果预测中心构造的两套ROI做预注册2×2 seed/ROI交叉，明确为条件化分析，不接在线反馈。②完整独立O链回放保持相同Target规则，但预测中心、历史O锚点和KEYFRAME选择可能使实际Target数组改变；逐帧记录中心、Target IDs/hash/点数/有效高斯数/覆盖，单独分析ROI/Target变化，不把所有收益归因于运动模型。

在0105/1013、normal8/right3、高速弯道、517–519故障及历史BB范围比较 Legacy B-Prediction、新KINEMATIC_BK、各自配准后结果及连续漂移。预测前误差与匹配后误差分开，报告0/75/150ms工程参考敏感性、短时50/100/150ms/1s/5s相对运动、拒绝/退化/缺口和累计漂移；不依据参考选Bias、门限、ROI、分支。公共Source仍保留已批准的A0 Deskew地图前史相关性。跨地图自身A0反馈可能改变Deskew Source，须核对实际hash并区分“同原始输入”与“逐字节同Source”，不能混称。

## 8. 计划源码范围、性能和暂停条件

仅在独立研究分支新增 `kinematic_predictor.py`（区间积分/合法样本选择）、`kinematic_contract.json`、`test_kinematic_predictor.py` 和独立 `kinematic_bridge.py`（双轴标签、固定初始化左变换、既有O/Target接口）；公共runner新增显式predictor selector与原样诊断，不替换当前已冻结模块/记录。若必须更改原Source、Target规则、坐标/轴、独立O、guard或CUDA目标，停止并再询问。

计算为每50ms内IMU/Wheel header并集上的少量3×3/SE(3)运算，不做15×15更新，不增GPU匹配次数。按原500ms级合法输入窗口保留索引/held值；实际CPU/P50/P95/P99/max、缓冲/日志成本及完整A0+B周期必须实测，不能先许诺50ms或100Hz通过。缺口、恒定转弯SE(3)解析对照、倾斜3D、time/available迟到、Bias固定、杆臂不重复、零输入/逆向和失败终止均需CPU数值测试；之后才在用户批准范围内运行真实配对和连续实验。

**当前状态：只有合同和原版离线诊断；新预测器尚未实现。须等待用户确认以上具体定义。**


## 附录：已核实的实际源码依据（本机审查路径）

- `production-repository/wheel_velocity/src/wheel_velocity.cpp`：`model`、`wheel_velocity::timer_callback`，Wheel轮位校正、VN角速度来源、base_link、linear.x/linear.y和timer now。
- `research-repository/src/geometry_ieskf/wheel.py`：`WheelObservation.model/correct`，IMU/base杆臂语义、旧guard、完整K与15D注入。
- `research-repository/experiments/phase_bg_far_lio/c1/continuous_inputs.py`：`stream`，原CDR、header/recorder available、IMU轴变换和三扫描release合同。
- `coordinator-repository/experiments/phase_kg_s3/shared/causal_runner.py`：`run`、`at_reference`、`base_pose`，135s原mean/P/零Bias初始化、独立pure U、原相邻时刻查询及B输入；没有把global NDT更新写入pure。
- `research-repository/experiments/phase_kg_s3/track_b/local_odometry.py`：`LocalOdometry.step/_target`，首合法U的去yaw/平移初始化、原ΔU差分/独立O链及当前预测中心的O系真实历史Target裁剪。
