# BB速度模块：设计、来源与修改理由

## 边界与来源

基于BA3606769，物理估计器为AY364d246的原始光学XY模式，原版c647ac3作冻结对照。保留Bosch传播、eav24轮速回退、完整15D、500ms延迟历史、CUDA Deskew/map0105/NDT100/原矩形裁剪/原fitnessR。没有局部位姿链、关键帧图、滑窗、相对位姿更新、伪绝对位置或GNSS融合。

独立两扫描配准复用AW e15e7a2的研究协议适配器及small_gicp fa0cfc983417c044b6036f71dfa448038bb9c820（MIT），没有复制第三方整库或生产实现。scan_pair_service.cpp与AW源码字节一致；构建需已有授权依赖。实际实验使用已保存AW二进制，SHA见manifest。服务只使用前两帧自身Deskew后的base参考扫描作Target，从不加载全局地图；每帧共两对50/100ms。全局CUDA-NDT仍是唯一LiDAR位姿观测。

FAR-LIO审计SHA ef20510316ad1f8dbf2e428e4a193f700493eccd，文件lib_cpp/odometry_cpp/modules/velocity_handler/include/velocity_handler/derivative.hpp。真实实现以previous_pose.inverse()*current_pose的Sophus SE3 log/dt返回[平移切向量,角切向量]，首帧返回零、abs(dt)<1μs或>1s重置，100m/s/10rad/s域检查。它的frame参数未使用；负dt未被abs检查排除。BB重用形式但不复制源码，明确正向25–150ms、首帧UNKNOWN而非真实零速、全向量拒绝、实际scan-to-scan而非已融合地图位姿差分。本文不是FAR完整LIO移植。

## 帧、时间与速度

每个真实源云已由当前候选自身15D历史完成三LiDAR CUDA Deskew，表达在同一base参考点的当前参考时刻。GICP内部的T_previous_current将当前云映射到上一参考云。相对初值来自当前候选两个未辅助的15D预测位姿之差；只为配准初值，不当测量。目标只保留两帧，非局部地图。

设phi=Log(R)，J(phi)=I+(1−cosθ)/θ²[phi]×+(θ−sinθ)/θ³[phi]×²，则xi=[J⁻¹t,phi]/dt。使用完整SE3 log，而非t/dt。它在恒定body twist下为相同body轴表示；动态时是区间等效速度，参考时间标为区间中点，不冒充端点瞬时速度。真实加速度、轴旋转与非交换运动形成有限区间近似误差。

光学对照采用完整T_base_sensor：v_sensor=R_base_sensorᵀ(v_base+omega×t_base_sensor)。避免光学测量点与后轴/IMU混用；不靠补零光学Z重建3D。对Kistler原始/滤波与轮速按同一50/100ms区间积分比较，仅离线评价允许端点插值。IMU对照由原生先验记录插值得到，属于条件性诊断，不是独立真值。速度输入本身没有查询GNSS或弯道编号。

## 分离几何、完整性与统计噪声

单对必须converged、candidate_valid、至少100对应且比例≥.5，速度范数≤100m/s、角速≤10rad/s。对应门槛只约束最小支持，不把点数视为独立样本数。平移归一尺度10m/旋转1rad，对small_gicp右扰动[rot,trans]的完整H作变换，再以姿态Schur补分析平移；最弱/最强方向比≥1e−4。非对角耦合保留，但H源于人工局部表面协方差，不生成统计R，不证明正确峰。

50ms与100ms线速度XY还须相互一致。以预先声明的50mm端点分辨率工程假设，固定一致性包络2√2·.05√(1/dt50²+1/dt100²)m/s；这不是已认证的误差界、置信区间或GNSS标定结果。较长区间用于减小差分放大。交叉间隔共享扫描，因此一致性不能作为两份独立证据融合。重复结构可能同时骗过两者，UNKNOWN保持。

冻结扫描完整一圈先发现100ms单对即使收敛且H合格，仍出现约65m/s的来源差异；这是设计交叉间隔检查和不直接新增速度Kalman测量的证据。没有用GNSS结果选择阈值或映射Hessian为R。

## 真正使用方式：下一帧匹配的速度辅助

qualified速度与同一中点历史状态形成world速度差：

dv_world = R_mid*v_pair_base − (v_IMU_mid − R_mid*(omega_pair×lever_base_IMU))。

下一帧只给全局NDT的平移初值增加dv_world_XY·(ref_next−ref_pair_end)，Z/R不变；仅当年龄0–75ms且XY差速范数≤5m/s时使用。5m/s沿用现有速度失配域，75ms限制20Hz只下一帧外推，不延长积分。超域拒绝，不裁剪状态或强行移动滤波后验。每次从自身15D原始预测重新构造初值，绝不累计成新的局部位姿。当前扫描自己的测速结果只供下一帧，不能反过来初始化当前同一个测速。

这是solver-seed velocity auxiliary，不是额外速度Kalman更新，也不是改变IMU状态传播方程。它通过真实NDT重新匹配的结果间接影响15D及下一次Deskew。若所有初值收敛到相同极值，本来就不应期望很大收益。不能声称增加了独立速度信息或完成测速统计标定。原始光学/轮速仍按AY进入15D；地图R、完整H/P/Joseph/reset未改。

为何不直接融合：scan-to-scan共享LiDAR端点、预测Deskew及运动初值，与同一NDT/15D误差相关；small_gicp平面特征值不是传感器噪声，当前既无联合相关R也无真实速度标定。直接给它独立R会再次过度计数。BB没有用放大P/缩小R补偿这一缺失。

## 代码修改及副作用

- lidar_velocity.py：独立SE3 log、传感器点速度、完整耦合几何诊断；单位/变换验证。不会产生伪速度协方差。
- lidar_speed.py：两帧历史、正向时间、两间隔一致性、实际配准、速度辅助和失效退出。主20Hz辅助和10Hz计算消融开关；跳过局部测速不丢任何全局LiDAR帧。额外CPU时间和历史年龄必须实测。
- run_loop/launch：只对AY运行器插入独立服务请求、下一帧seed和私有日志；全局matcher、滤波器和原始源处理字节不变。monitor开关关闭辅助，40帧完整seed/P/NDT/delta与AY完全相同。
- 失效硬化：局部服务broken pipe/非法JSON/100ms无响应即清除hint并退出测速；不终止正常全局NDT。100ms为故障退出上限，不是实时通过门槛。服务挂起终止，后续不重启或伪造速度。指定[20,22)s测速不可用注入，与真实传感器异常分开保存。

P主候选的初版单圈/三圈保留；增加服务故障退出及低频开关后重新执行最终三圈。L低频单圈仅计算成本消融，不声称完成L连续三圈。旧数据、stash、生产默认和main保持不变。

## 有依据的第二候选R：扣除已经发生的速度反馈

P完整运行后检查链路发现，pair在当前NDT更新前计算dv，若下一帧直接使用，可能没有扣除上一NDT已经施加的速度反馈。这不是Kalman矩阵错误，而是速度辅助初值的时序近似。R保存测速结束时的未更新v_end_pred；使用hint时从当前已知历史取同一end时刻最后一个后验v_end_post，令dv_R=dv_P+v_end_pred−v_end_post。仅作一次next-seed外推，所有阈值、点云、几何及滤波更新不变。历史端点缺失就退出，不用最新状态代替。

该操作只扣除明确发生的速度反馈；有限区间加速度、全局姿态修正及shared Deskew仍是近似，并非新的联合统计测量。用独立测试验证0.8m/s已纠正量不会再次作为1m/s提示原样补偿。没有根据GNSS选择倍率。P/L/指定fault结果完整保留，R须重新执行自身单圈/三圈，并独立判断是否取得整体收益。
