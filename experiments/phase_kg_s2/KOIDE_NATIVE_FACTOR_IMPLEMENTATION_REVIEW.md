# KOIDE NATIVE FACTOR IMPLEMENTATION REVIEW

本报告属于KG-S2有限CPU固定滞后Shadow研究。未修改生产、main、NDT、15D核心、地图或Q/Bias；未运行全圈/三圈。所有精度数值使用既有GNSS/INS及天线到base转换的工程参考，不是独立真值。完整原始输入、状态、P/J/H、轨迹和消息身份留在授权私有卷，公开文件仅含自编源码及脱敏标量。CONFIRMED指所述具体数值/实现事实，不等于定位或统计认证。

## 实际复现边界

**CONFIRMED**：Koide等ICRA2024将固定地图配准、分别连接历史扫描的配准、IMU预积分和历史状态边缘化放在同一目标中。原论文已知地图定位程序与GLIM不是同一个公开程序；本轮未运行论文完整闭源实现。[Koide原文Sec.III](https://arxiv.org/html/2402.05540v1#S3)、[作者项目页](https://staff.aist.go.jp/k.koide/projects/icra2024_gl/)、[原生VGICP](https://github.com/koide3/gtsam_points/blob/8e94654558dfbe7ddb0341de300daaf17e25ee02/include/gtsam_points/factors/impl/integrated_vgicp_factor_impl.hpp)、[原生LM扩展](https://github.com/koide3/gtsam_points/blob/8e94654558dfbe7ddb0341de300daaf17e25ee02/src/gtsam_points/optimizers/levenberg_marquardt_ext.cpp)。

**CONFIRMED**：本轮构建并实际链接gtsam_points `8e94654558dfbe7ddb0341de300daaf17e25ee02` CPU库，GTSAM4.2。使用原生`IntegratedVGICPFactor`：残差为目标体素高斯均值与变换后Source均值之差，权重为两局部协方差旋转合成的逆；对应在每次linearize更新，LM试探期间使用缓存的对应/权重。Target体素储存平均局部协方差，不是体素散布协方差。native KNN=20、特征值正则(.001,1,1)、体素1m。本轮不是Stage1的.5m公式诊断的等价复现，也不是DIRECT7 NDT目标。

没有把独立配准结果包装成PoseFactor。E1/E3/E4直接把原始几何H/g及源/目标交叉项送入非线性状态优化。Scan-to-Scan最多分别连接前3帧，未拼接Target。

## 必须处理的原生集成细节

**CONFIRMED**：原生`error()`返回ΣrᵀWr，而原生HessianFactor采用0.5二次型。研究适配器仅把非线性cost归一化为0.5ΣrᵀWr，保留原生H/g/交叉项。合成固定对应和固定权重下有限差分吻合；这不声称包含协方差旋转导数的精确Objective Hessian。

**CONFIRMED**：原生LMExt在线性化更新对应后重算当前cost。普通GTSAM BatchFixedLag LM不会保证同样语义，历史单因子会走一步后因旧cost而过早停止。保留`results/`和`verified/`集成对照，正式结论只用`native_verified/`。原生LMExt + GTSAM CalculateMarginalFactors构成正式滚动图；没有修改第三方源码。

## GLIM与Holistic Fusion

GLIM审查固定`1a197c999e4865f5cab7c9cd25474128413ebbca`。[GPU因子构造](https://github.com/koide3/glim/blob/1a197c999e4865f5cab7c9cd25474128413ebbca/src/glim/odometry/odometry_estimation_gpu.cpp)与[IMU链](https://github.com/koide3/glim/blob/1a197c999e4865f5cab7c9cd25474128413ebbca/src/glim/odometry/odometry_estimation_imu.cpp)用于原理对照。OVERLAP关键帧管理不等于固定最近帧；本轮保持固定1–3帧，以隔离因子本身，未声称新验证了OVERLAP优越性。

Holistic Fusion固定`6e2b3a8841d022552f83cfa90149ff2044645723`。[ImuBuffer](https://github.com/leggedrobotics/holistic_fusion/blob/6e2b3a8841d022552f83cfa90149ff2044645723/holistic_fusion/src/lib/ImuBuffer.cpp)可从优化状态以实际IMU与Bias传播至新时间；[GraphManager](https://github.com/leggedrobotics/holistic_fusion/blob/6e2b3a8841d022552f83cfa90149ff2044645723/holistic_fusion/src/lib/GraphManager.cpp)区分预积分缓存与优化。借鉴的是低频优化和高频传播分离；未安装或运行HF，更未声称其默认使用本轮原始点云因子。

**UNKNOWN**：论文GPU性能、原闭源程序全部参数和跨场景鲁棒性没有复现。CPU结果不能提前外推GPU50ms通过。
