# FIXED LAG GRAPH ARCHITECTURE

本报告属于KG-S2有限CPU固定滞后Shadow研究。未修改生产、main、NDT、15D核心、地图或Q/Bias；未运行全圈/三圈。所有精度数值使用既有GNSS/INS及天线到base转换的工程参考，不是独立真值。完整原始输入、状态、P/J/H、轨迹和消息身份留在授权私有卷，公开文件仅含自编源码及脱敏标量。CONFIRMED指所述具体数值/实现事实，不等于定位或统计认证。

## 状态与参考点

**CONFIRMED**：每节点X=Pose3(map←物理IMU)、V=map下IMU原点速度、B=(ba,bg)为物理IMU轴向Bias，保留全部15维。原15D姿态body轴转到真实IMU轴；Source由base转IMU，地图保持map系。不把base后轴速度当成IMU速度。完整初始15×15 P以右误差图一阶变换，保留交叉块；仅每个窗口第一个合法检查点进入prior，后续旧15D状态不作为在线因子。

令T_BI=(R_BI,r_BI)，则R_MI=R_MB R_BI，p_MI=p_MB+R_MB r_BI；Source p_I=R_BIᵀ(p_B−r_BI)。Wheel模型为

\[
h_w=e_x^T[R_{BI}R_{MI}^Tv_{MI}-(R_{BI}(\omega_m-b_g))\times r_{BI}].
\]

只约束纵向；未增加vy=0非完整约束。沿用既有Wheel方差与slip模型；未调Q/Bias。原始Bosch比力直接用于物理IMU原点，map重力向下，没有重复补偿重力或虚构Wheel。

## 时序与因子

**CONFIRMED**：节点位于实际Wheel测量时刻及LiDAR参考时刻，真实IMU分段ZOH预积分，含姿态/速度/位置、Bias修正、协方差与Bias RW边。未把已融合LiDAR增量当作IMU观测。采用0.5s单一固定lag，没有窗口长度搜索。

E0=IMU+Wheel；E1=E0+原生一元Scan-to-Map；E2=E0+分别连接前1/2/3帧的二元VGICP；E3=两类几何；E4=仅E1地图几何加入逐对应Huber。几何不经过NDT最终位姿或Fitness R。地图Target以预先存档的因果预测裁剪，固定120m平面范围及原点相对高度范围；不随后续优化答案挑地图。Source/Deskew固定原路径，因此本轮是冻结前端的后端Shadow，不是真前端反馈闭环。

原生LMExt保持对应更新与cost同步；GTSAM CalculateMarginalFactors在涉及离窗变量的因子子图上做Schur消元，留下LinearContainer边缘先验，删除旧变量。不是直接丢弃历史信息。线性化边缘先验不能恢复被边缘化变量的完整非线性信息，未实现原GLIM全部FEJ/关键帧策略。

每步记录所有活动节点状态、初值、因子ID、H/g、最新完整P、Bias、迭代、边缘化和计算时间。配置maxIterations=8，但原生扩展的0开始计数可产生9个外层步骤，实际次数如实保存；到上限仅代表预算截断，不叫converged。没有通过增加预算追分。

## 因果性与相关性边界

**CONFIRMED**：fault及四个BE子窗口均按冻结Header/原始可用时间处理；只使用当时已经可用且测量时间不晚于节点的运动输入。优化后的旧窗口节点另存retrospective，不能冒充它们当年的在线输出。最新节点输出仍在扫描时刻，不是已经传播到完成时刻的100Hz控制状态。

713另一次经过没有原LiDAR recorder可用时刻及成熟完整15D检查点。仅做明确标记的条件化离线窗口：初始均值来自原因果Seed，初始速度来自过去Wheel及杆臂、零横向初始假设，Bias从模型零初值、P用既有初始配置。它不是恢复的成熟15D，也不是到达时序合格的因果运行；数据释放时间只作离线输入批次下界。

**UNKNOWN**：原Deskew和Target裁剪共享旧地图反馈，Scan-to-Map/Scan-to-Scan共享当前Source，多边还共享历史扫描与地图。未完整建模跨因子/跨帧相关性，未加任意降权来包装一致性。E3中P进一步收缩但精度未稳定改善，不能将这些因子视为独立额外观测。

时间资格补充：运动节点按真实测量时间建立，但本CPU后端在LiDAR节拍将其成批送入优化；它不是实时100Hz Wheel/IMU发布器。原始可用时间用于输入资格检查，程序未以真实ROS clock节拍等待发布，故仅称“因果输入前缀Shadow”，不称在线调度验收。state_age_at_available仅为扫描到记录可用时刻的年龄，不是CPU完成/控制发布年龄。

Wheel可比性补充：新图使用有效传感器数据及相同纵向/杆臂/方差模型，但未将原15D顺序更新的运行时残差拒绝门限复制为图的自动因子选择器。所有E组保持同一Wheel因子；与原15D纯预测的比较因此不是逐运算数值等价移植，不能把微小差异归为新信息收益。
