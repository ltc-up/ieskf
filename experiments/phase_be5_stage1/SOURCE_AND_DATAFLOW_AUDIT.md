# BE5 Phase 0：实际数据流与状态合同（草稿）

范围：只读审查。源码基线 a03036ca3289b00ab747d13b19059421aff28790；生产源码另属 aa50e35467f89b4963f882d1f5abaace46d0ce70，不能称为同一仓库/二进制。CONFIRMED=直接读到源码/保存记录；HYPOTHESIS=待检验解释或拟议模型；UNKNOWN=缺失证据。下文未来工作均未执行。

## 实际15D合同

CONFIRMED：state.py:4–16中名义状态为(p,v,R,bg,ba)，误差排列[p(3),v(3),theta(3),bg(3),ba(3)]。p/v均在地图坐标表达、参考IMU原点；R将base对齐的body坐标转到map。inject采用R Exp(delta_theta)，右扰动；bg/ba在已旋转到body的轴上，不是原Bosch消息轴。R不是IMU原始安装轴姿态。chart_map用Jr(delta_theta)重设误差协方差，非简单重置姿态三个方差。

CONFIRMED：inertial.py:5–25为保持上一IMU输入的ZOH模型，中点姿态Rm=R Exp((omega-bg)dt/2)，a_map=Rm(force-ba)+g，g沿map-Z向下。p+=v dt+a dt²/2，v+=a dt，R*=Exp((omega-bg)dt)。force合同是比力；若输入已经去重力会重复补偿。代码只加一次g，不能因此证明原始驱动从未去重力。已有test_motion_audit.py:15等测试区分静止比力和零线加速度，但本轮未执行测试。

CONFIRMED：F含p-v、p/v-theta、p/v-bg、p/v-ba和theta-bg耦合，P'=FPFᵀ+GQGᵀ，保留完整15×15；Bias名义值传播不变、协方差含随机游走。Q采用采样保持近似，不是精确连续白噪声离散积分；现有test_ZOH_Q_is_not_exact_continuous_white_noise明确位置噪声项为连续积分的0.75倍。不能把“名义离散模型Jacobian准确”写成“所有物理噪声都精确离散化”。本轮不更改Q/Bias模型。

CONFIRMED：Wheel/ExternalPose采用Joseph形式，再用Jr reset并对称化。ExternalPose通过Cholesky验证输入R和后验P，原子拒绝保留状态/P。对称化只消除数值不对称，不保证SPD；SPD数值通过也不证明统计一致。

## 真实输入表

| 输入 | 保存的实际接口 | 15D用途及适配 | 证据 |
|---|---|---|---|
|Bosch|/sensor/bosch/imu，sensor_msgs/Imu|读取事件omega/force，按T_base_IMU旋转一次到body后传播|CONFIRMED：motion事件字段、run_loop.py:28–43|
|Wheel|/localization/wheel_velocity，TwistWithCovarianceStamped|读取已转换的speed/variance标量，WheelObservation约束base前向速度|CONFIRMED：AR INPUT_AND_MAP_IDENTITY、保存事件；不是直接解析原始EAV26 CAN|
|Kistler|/sensor/kistler/measurement，历史Kistler消息及状态|独立AY/BE光学配置，原轴速度+安装变换+角速度杆臂；XY模式暂时抑制重复wheel更新|CONFIRMED：optical_velocity.py、optical_replay.py；不是所有15D基线默认融合|
|NDT|保存服务请求/结果，6DoF T_map_base|ExternalPose(x,y,z,rx,ry,rz)，通过base位置p-Rr预测，不把base位置当IMU位置|CONFIRMED：run_loop.py:39、external_pose.py|
|GNSS|motion文件有gps/gps2条目，离线参考另存|在线sensor白名单仅imu/wheel/optical/clock/ndt；历史pose只初始化一次|CONFIRMED：存在记录不等于消费；初始pose更早来源是否含GNSS UNKNOWN|
|生产EKFConvertor|GPS/IMU/速度转换ROS链|不是15D离线回放的订阅或适配环节|CONFIRMED：15D无ROS订阅，直接读导出事件；保存wheel本身已是上游转换产物|

CONFIRMED：run_loop实际独立使用CUDA Deskew服务；每个原有分支用自己状态历史产生逐点去畸变和Source，source历史归档与500ms融合窗口不是一个东西。它是离线因果反馈程序，不是部署ROS 15D节点；不应罗列不存在的15D ROS remap。实际ROS launch、话题订阅在原采集/生产节点；本轮未启动任何节点。

CONFIRMED：从实际motion.transforms调用static_transform(base_link,imu_bosch)，杆臂与用户指定历史向量一致，旋转不是单位阵。具体矩阵和设备数值仅存私有source_and_contract_evidence.json。w和f都先乘该旋转，Bias随后在body轴估计。没有发现这段回放再次调用Converter；不能据此认证所有原始驱动轴/杆臂物理校准。Kistler物理测量光斑参考点、残余安装和传输延迟仍UNKNOWN。

CONFIRMED：观测的ref是保存的native scan reference；available由原三雷达arrival决定，程序再叠加实际计算耗时。PoseDelay/ManagedReplay按原header回溯并重传播。IMU/Wheel排序主要按header，不等于完整复刻ROS executor实际callback次序；真实传感器曝光与header偏差、缺失的历史入队时刻不能补成0。

## 生产六状态代码级对照

| 项目 | 生产6D | 当前研究15D | 尚不等价 |
|---|---|---|---|
|状态|X,Y,Yaw,YawBias,Vx,Wz|IMU原点p,v、SO(3)、bg、ba|参考点、维数、Bias物理含义不同|
|预测|平面速度/yaw运动学|3D比力、陀螺、重力积分|15D新增垂向和重力投影风险|
|IMU|Converter/独立姿态链及yaw-rate|原Bosch轴向适配后直接惯性传播|噪声、采样与时序未等价|
|Wheel/Kistler|Converter选择速度源，主滤波平面速度|wheel含杆臂、姿态和bg导数；光学独立实验|旧wheel已转换，不是假设raw CAN|
|LiDAR|XY/Yaw测量，Z/RP独立1D处理|BE使用完整6DoF ExternalPose，全P耦合|不能复用生产平面R作为统计6D R|
|延迟|15级扩展历史，约140–150ms，最多五次处理|约500ms事件账本，回滚原输入并重放|生产不能撤销已融合观测|
|坐标|生产pose检查frame_id但不是自动TF转换|明确p_IMU-Rr及原TF旋转|实际部署地图/参考点仍需追踪|
|高度姿态|不在平面6维耦合状态|全状态耦合传播/观测|直接比较Vz没有生产对应量|
|R来源|发布策略+callback分档+平面子矩阵×5|BE从fitness重建对角6D R|Native助手已有但BE未调用|
|门控|callback范围/延迟/宽松Mahalanobis|匹配有效性、R范围和姿态/Bias域、账本失效|不等同独立NIS门控或错误峰识别|

CONFIRMED：当前aa50 lidar_ndt源码/YAML选择DIRECT7/GLL3，不能套用历史c647 OFF；lidar_ndt和v1的launch都存在，不是部署证明。现场加载组件、参数覆盖及二进制对应性仍UNKNOWN。

CONFIRMED：GNSS沿用现有生产use_gps语义，本轮不新增完整性逻辑。旧GPS1 Converter中直接发布antenna ENU、到base补偿注释未启用；标签map不能证明后轴中心。独立global_init有use_gps_position_prior入口，但此15D运行器不调用它。GNSS默认在线关闭、仅离线评价；不借C偷偷把VectorNav GNSS/INS解重新带入估计。
