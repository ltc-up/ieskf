# BG-C1共同初始化审计

研究基线799064772cf1af282e36ee789e45932d0e2ecdb3。最新用户授权一次性录制EKF位姿，取代此前“必须证明历史GlobalInit成功”的停止门。本实验是**外部初始位姿条件化定位**，不是独立GNSS-free全局初始化。

CONFIRMED：初始化时刻固定bag相对135s，读取起点130s。使用最后一条header和recorder均不晚于135s的有效kinematic_state消息1185392，frame=map、child=base_link；完整均值、原始header/recorder和消息6×6协方差仅保存在私有initialization.json。均值仅用一次；没有读取后续录制EKF/GNSS/GT更新新状态或选择匹配。

共同T_map_base使用同一既有map0105 native XYZ坐标合同；不经过NED，也不增加PCM变换。现有0.8m目标的身份和原地图关联沿用BG保存记录；不能凭frame字符串认证全球地图/GNSS大地基准。base_link为原后轴参考合同，T_base_IMU来自相同已审计静态TF，禁止回放bag map→base_link动态TF。IMU原点的位置p_IMU=p_base+R*r；静止base速度零时v_IMU=R*(omega_body×r)，不把两个参考点速度直接混用。

CONFIRMED：选帧前约5s因果样本共999条IMU、500条轮速（准确间隔和样本身份见私有记录），轮速为零；比力各轴std约0.017/0.022/0.032m/s²，最大IMU间隔6.11ms。均值比力与初始姿态的重力方向夹角1.8073°，并非完美独立姿态真值。不利用未来样本拟合Bias；FAR保留原生固定bias_init零值，15D保留原零Bias均值，后续各自正常传播/更新。静止样本仅支持初始零base速度的条件，不证明无悬架运动或绝对倾角准确。

## FAR实际初始化

CONFIRMED（官方commit ef20510316ad1f8dbf2e428e4a193f700493eccd）：`state_estimation.hpp::set_initial_state()`要求有效IMU、位置、逐轴姿态以及速度。`initialize_from_measurements=true`时，GICP初始参数只设置前端，不自动设置九状态EKF。因此运行器通过原生测量接口向独立relay通道投递一次授权均值和静止零速度，并发送相应初始化状态；之后只转发FAR自己的GICP测量/状态。共同初值不反复发布、不作为未来独立观测使用。原生fusion_vec在step中清零；首次step可能消费初始化尚存标志，这是原生行为，不人为改变。

KFBase构造器将9×9 P置零；`set_covariance_matricies()`设置R和Q，不把P设为0.01，也不复制录制协方差。native初始P为半正定零阵，是实现事实，不是统计合理性认证。首个实际EKF输出位置与共同均值之差为0；输出stamp在135s后7.364ms。首个pose协方差对角为[0,0,0,3.5208e-8,3.0031e-8,2.9973e-7]，linear-velocity块对角约[9.9983e-5,2.3994e-4,3.9968e-4]。两个可导出块有限且对称；完整位置/姿态/速度交叉块未从ROS输出导出，标UNKNOWN。twist消息中的angular covariance不属于九状态P，不能把其中未明确初始化的极小值解释为角速度置信度。

原生日志明确记录“Initial state of the state estimation has been set”，随后持续输出21079条EKF状态。不能以GICP启动替代这一证据。内部同IMU倾角参考、非完整约束版本、FIR、Q/R、门控及固定Bias设置全部保留；没有接入额外外部姿态。

## 15D实际初始化

使用相同均值时刻与参考点转换。P对角保持研究默认：p各1、v各4、theta各0.0076154355、bg各0.0004、ba各0.25；无历史P/成熟Bias复制。完整初态P和p/v/R/bg/ba私有保存。保留原研究轮速更新、完整惯性模型、原Legacy观测R和500ms回溯语义；未启用Optical、A/C、GMF或新门控。完整输出P逐条检查，不把数值检查当精度认证。

两系统只共享初始位姿均值，不共享状态维度、P、轮速、倾角参考、Bias模型或滤波器假设。结果差异不能全部归因GICP/NDT。

CONFIRMED：全部25295条15D输出的完整15×15 P有限且对称，最大非对称量0，最小特征值5.0020991e-07；不是统计尺度校准或定位精度验证。约180s，15D bg范数约0.00139rad/s、ba范数约0.0481m/s²，来自本次自身历史更新，不是历史成熟Bias复制。

初始化消息零cov占位的实际语义：原生set_input_position/orientation在开启adaptation时会先接收消息cov；随后set_initial_state调用set_covariance_matricies将R及R_adaptive重新设回原生R_init。故不能笼统称零cov永远不被读取，也不能将其解释成共享了历史P；本次未自定义R值或改该原生重置行为。初始化标志可能在首次step再次被消费，已披露为原生初始化语义，而不是额外独立信息。
