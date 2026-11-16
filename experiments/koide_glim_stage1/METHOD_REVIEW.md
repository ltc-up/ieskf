# 方法审查与边界

主要论文：Kenji Koide、Shuji Oishi、Masashi Yokozuka、Atsuhiko Banno，ICRA 2024，*Tightly Coupled Range Inertial Localization on a 3D Prior Map Based on Sliding Window Factor Graph Optimization*。论文将最近N帧VGICP、相邻IMU预积分、每帧到固定地图的配准以及边缘化先验联合优化；不是先独立求若干相对位姿再当独立高斯测量。前三帧分别构成边，而非拼接一个Target。[原文Sec.III](https://arxiv.org/pdf/2402.05540)。[作者项目页](https://staff.aist.go.jp/k.koide/projects/icra2024_gl/)标记代码closed-source；不能声称运行了其完整定位实现。

GLIM是另一个开源框架，不等于上述先验地图定位程序。当前审查固定commit `1a197c999e4865f5cab7c9cd25474128413ebbca`（不是假定2024原始版本）。GPU odometry分别连接最近帧和历史关键帧；窗口外关键帧可以固定姿态构成一元边。[create_factors源码](https://github.com/koide3/glim/blob/1a197c999e4865f5cab7c9cd25474128413ebbca/src/glim/odometry/odometry_estimation_gpu.cpp)。IMU预积分单独生成ImuFactor及Bias时间约束，不是点云残差，也不是Wheel积分。[IMU源码](https://github.com/koide3/glim/blob/1a197c999e4865f5cab7c9cd25474128413ebbca/src/glim/odometry/odometry_estimation_imu.cpp)。

GLIM OVERLAP以当前扫描对关键帧集合的联合占用支持决定是否插入；超容量先移除低重叠，再用对最新帧的支持与对其它关键帧的冗余共同评分。不是只挑最近或最高重叠一帧。默认配置最多15关键帧，本次为等成本诊断冻结3个、1s历史、.7/.01插入/移除参考值，不能视作赛车调优。[配置](https://github.com/koide3/glim/blob/1a197c999e4865f5cab7c9cd25474128413ebbca/config/config_odometry_gpu.json)。

B/C实际执行的是独立CPU公式级D2D因子诊断，不是原生GPU/联合估计：当前点均值与历史体素均值残差，权重为目标平均局部协方差与旋转后的源协方差之和的逆。体素协方差取平均局部协方差，不能换成体素散布并冒称VGICP。[gtsam_points原始因子](https://github.com/koide3/gtsam_points/blob/8e94654558dfbe7ddb0341de300daaf17e25ee02/include/gtsam_points/factors/impl/integrated_vgicp_factor_impl.hpp)、[体素均值实现](https://github.com/koide3/gtsam_points/blob/8e94654558dfbe7ddb0341de300daaf17e25ee02/src/gtsam_points/types/gaussian_voxelmap_cpu.cpp)。

本次冻结：体素.5m、20近邻、局部表面特征值(.001,1,1)，无surface-validation、多分辨率、原生GPU和联合优化。没有验证与Koide原生因子逐数值等价。原始D2D cost、对应支持、12×12联合GN信息及交叉块、梯度保留；单步GN仅候选，不当作优化后位姿。C也只是GLIM式选帧诊断，非GLIM完整性能复现。

D才是用户提出的工程对照：用至多3个真实历史扫描、经因果运动放入最早历史base系，形成局部NDT Target；与B/C严格分开。E提供gyro+Wheel运动诊断及独立列出的原15D短预测，不构建新融合器。
