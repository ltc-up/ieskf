# Phase BE 文献与参考源码审查

审查日期：2026-11-13。范围：设计研究，不运行新定位算法。GNSS方向按用户后续约束收窄为生产参数与坐标链审计；RAIM/FDE、欺骗检测、新完整性状态机及新GNSS融合机制均不纳入本轮建议。本文“适用性/复杂度”若未标论文实测，均为本研究推导，不是作者实验结果。

## 阅读与证据等级

正文已查阅：FAR-LIO III、X-ICP方法、SuperLoc III、COIN-LIO III、scan-matching PF方法、Doucet RBPF、Fang Gaussian-sum章节、GLIM方法、Goblirsch车辆模型；Larsen通过DTU原文浏览读取。Selmanaj可读出版社方法说明，但作者PDF下载403/浏览超时，未完成全文逐式审查。Alspach/Sorenson和Bar-Shalom经典原文访问失败，只记录准确书目，不以其摘要推断实现细节。没有声称所有经典论文已全文复现。

MHT另查阅Reid原文第V节；IMM出版页被JS验证阻挡，其经典书目信息与通用混合思想可核对，但未完成原文逐式审查。

官方代码已保存版本并审查关键函数，见SOURCE_REVIEW.md。错误猜测的SuperLoc/COIN仓库URL已更正，失败访问保留在本地审计；后续仓库不能冒充原论文发表版本。

## 文献卡片

### R1 FAR-LIO

**FAR-LIO: Enabling High-Speed Autonomy through Fast, Accurate, and Robust LiDAR-Inertial Odometry**；Maximilian Leitenstern, Marcel Weinmann, Patrick Haft, Tobias Lasser, Dominik Kulmer, Markus Lienkamp；2026，IROS录用预印本。[原文](https://arxiv.org/html/2606.26010v1) / [代码](https://github.com/TUMFTM/FAR-LIO)。III-A采用CUDA体素哈希、GICP，源点邻域不足时退化为点对点；III-B为九状态运动学EKF、延迟补偿。论文核心输入LiDAR/IMU，不是GNSS融合论文。实际多赛车高速测试有参考价值；不能将作者硬件结果当作本机50ms保证。计算约随点数、候选邻域与迭代增长。借鉴稀疏点处理和时间链计量；不照搬扫描间测速、动态子地图、重复测量平滑或作者允许丢帧的时限策略。

### R2 X-ICP

**X-ICP: Localizability-Aware LiDAR Registration for Robust Localization in Extreme Environments**；Turcan Tuna, Julian Nubert, Yoshua Nava, Shehryar Khattak, Marco Hutter；2022预印本、2024 IEEE T-RO。[原文](https://arxiv.org/html/2211.16335v3) / [作者项目](https://sites.google.com/leggedrobotics.com/x-icp/home) / [后续官方比较实现](https://github.com/leggedrobotics/perfectlyconstrained)。逐对应分析优化主方向贡献，分可定位/部分/不可定位，再约束优化。证明的是退化场景鲁棒性，不是重复护栏多峰可判别性。对应贡献累积约O(N)，小矩阵分解近常数。适合借鉴方向保留；原ICP更新约束不能直接变成NDT观测R，源码后续版本也不是原论文逐位复现。

### R3 SuperLoc

**SuperLoc: The Key to Robust LiDAR-Inertial Localization Lies in Predicting Alignment Risks**；Shibo Zhao, Honghao Zhu, Yuanjun Gao, Beomsoo Kim, Yuheng Qiu, Aaron M. Johnson, Sebastian Scherer；2024预印本、2025 ICRA。[原文](https://arxiv.org/html/2412.02901v2) / [项目](https://superodometry.com/superloc) / [发布代码SuperOdom](https://github.com/superxslam/SuperOdom)。III用点法向/力臂构造方向贡献与标签统计，前置分析仍需要对应/局部几何，并非零成本、完全不依赖初值。主体累积O(N)，邻域搜索另计。适合做匹配前后诊断互补。论文称covariance的标签比例不等于标定误差协方差；其均匀方向假设、阈值和对角化不能直接用于赛车XY-yaw耦合滤波。

### R4 COIN-LIO

**COIN-LIO: Complementary Intensity-Augmented LiDAR Inertial Odometry**；Patrick Pfreundschuh, Helen Oleynikova, Cesar Cadena, Roland Siegwart, Olov Andersson；2023预印本、2024 ICRA。[原文](https://arxiv.org/html/2310.01235v4) / [代码](https://github.com/ethz-asl/COIN-LIO)。几何弱方向指导强度图像patch选择，光度残差与点面/IMU共同进入迭代滤波。强度图像处理与patch评价增加成本。适用性：补充模态原则有价值；当前多雷达强度一致性、地图强度和曝光类漂移未验证。本轮不引入强度前端，也不能称仅靠现有Hessian就实现COIN-LIO。

### R5 GICP

**Generalized-ICP**；Aleksandr Segal, Dirk Hähnel, Sebastian Thrun；2009 RSS。[原文及作者代码入口](https://roboticsproceedings.org/rss05/p21.html)。双边局部协方差决定对应残差度量；有模拟与真实配准实验。每次迭代近邻查询约O(N log M)，小位姿求解另计。适合局部候选与方向几何；局部平面模型不消除错误对应/重复结构，形状协方差不是最终位姿统计R。

### R6 small_gicp

**small_gicp: Efficient and parallel algorithms for point cloud registration**；Kenji Koide；2024 JOSS 9(100),6948。[论文](https://joss.theoj.org/papers/10.21105/joss.06948) / [代码](https://github.com/koide3/small_gicp)。并行预处理、搜索和配准，提供H/b/error/inliers；是CPU并行库，不能称现成CUDA GICP。复杂度同局部GICP并受线程、预处理影响。本机AW已有真实闭环，故值得复用为Shadow第二后端；不重复宣称替换NDT有利，不把库benchmark视为全链预算。

### R7 scan matching PF

**Scan Matching-Based Particle Filter for LIDAR-Only Localization**（预印本题名 *Scan-Matching based Particle Filtering approach for LIDAR-only Localization*）；Naga Venkat Adurthi；2023 Sensors 23(8),4010。[出版原文](https://doi.org/10.3390/s23084010) / [可读预印本](https://arxiv.org/html/2302.06843v1)。预计算似然网格加并行全局/局部匹配改善粒子重采样，KITTI验证。未核实可用官方代码仓库，不填猜测URL。粒子似然约O(PN)，全局匹配额外计；不是20Hz赛车连续跟踪证明。不照搬宽松定位判据/重置策略；可借鉴匹配引导proposal与异步候选。

### R8 RBPF

**Rao-Blackwellised Particle Filtering for Dynamic Bayesian Networks**；Arnaud Doucet, Nando de Freitas, Kevin Murphy, Stuart Russell；2000 UAI（arXiv于2013上传）。[原文](https://arxiv.org/abs/1301.3853) / [作者机构书目](https://research.google/pubs/rao-blackwellised-particle-filtering-for-dynamic-bayesian-networks/)。分解采样变量与可解析条件状态，降低采样维度。原文有机器人定位/建图例；未核实配套官方代码。约O(P×条件滤波成本)，本研究EKF条件化只能叫近似RBPF。适合离散盆地+连续状态，不支持“2粒子必然足够”的断言。

### R9 Gaussian mixture

**Nonlinear Bayesian Estimation: From Kalman Filtering to a Broader Horizon**；Huazhen Fang, Ning Tian, Yebin Wang, MengChu Zhou, Mulugeta A. Haile；2017预印本。[正文VII](https://arxiv.org/html/1712.01406v2)。给出GSF独立更新均值/P/权重的通用框架，无赛车验证、未核实官方代码。K分支滤波约O(Kn³)，候选匹配常为主要成本。适合2～3分支研究，需额外标定似然。经典 **Nonlinear Bayesian estimation using Gaussian sum approximations**，Daniel L. Alspach, Harold W. Sorenson，1972 IEEE TAC 17(4):439–448，[DOI](https://doi.org/10.1109/TAC.1972.1100034)仅书目核实，原文未成功读取。

### R10 MHT

**An algorithm for tracking multiple targets**；Donald B. Reid；1979 IEEE TAC 24(6):843–854。[出版入口](https://ieeexplore.ieee.org/abstract/document/1102177/authors) / [原文](https://www.cse.psu.edu/~rtc12/CSE598G/papers/ReidMHT.pdf)。保持关联历史，后续证据裁剪/合并。官方原版代码未核实；分支树最坏指数增长。迁移的是延后决策思想，不是把赛车定位变为多目标跟踪。限制K可控但会丢模式，不能承诺完备性。

### R11 IMM

**The interacting multiple model algorithm for systems with Markovian switching coefficients**；Henk A. P. Blom, Yaakov Bar-Shalom；1988 IEEE TAC 33(8):780–783。[原文入口](https://ieeexplore.ieee.org/document/1299)。条件模型交互混合，通用复杂度模型间混合O(K²n²)+滤波；未核实作者官方代码。本轮只借鉴未来动力学模式研究；跨地图盆地进行IMM均值混合会制造中间假位姿。不能把运动模型切换和匹配盆地切换等同。

### R12 OOSM与延迟

**Incorporation of Time Delayed Measurements in a Discrete-time Kalman Filter**；Thomas Dall Larsen, Nils Axel Andersen, Ole Ravn, Niels Kjølstad Poulsen；1998 CDC pp3972–3977。[机构原文](https://backend.orbit.dtu.dk/ws/portalfiles/portal/4363506/Larsen.pdf)。比较增广/重算/外推，外推配合适当增益，并非所有条件最优。无核实官方代码。增广内存O((nH)²)，重放约O(H×单步更新)。适合延迟方案选择；不只平移观测却忽略相关性。

**Update with Out-of-Sequence Measurements in Tracking: Exact Solution**；Yaakov Bar-Shalom；2002 IEEE TAES 38(3):769–777，[DOI](https://doi.org/10.1109/TAES.2002.1039398)。原文访问失败，仅书目/出版摘要核实，不据此声称已实现exact OOSM或其适用性证明。

### R13 固定滞后/GPU GICP

**GLIM: 3D Range-Inertial Localization and Mapping with GPU-Accelerated Scan Matching Factors**；Kenji Koide, Masashi Yokozuka, Shuji Oishi, Atsuhiko Banno；2024预印本。[原文](https://arxiv.org/html/2407.10344v1) / [官方代码](https://github.com/koide3/glim)。固定滞后与GPU配准因子，后续子图优化。矩阵稀疏结构/重线性化决定成本，最坏稠密求解O((nH)³)，不能只算6×6求逆。[GTSAM官方固定滞后文档](https://borglab.github.io/gtsam/fixedlagsmoother/)说明窗口边缘化。适合长期技术参照，当前完整后端替换工程量大；GLIM的GPU能力不意味着small_gicp已上GPU。

### R14 车辆侧滑

**Vehicle sideslip estimation: A kinematic based approach**；Donald Selmanaj, Matteo Corno, Giulio Panzani, Sergio M. Savaresi；2017 Control Engineering Practice 67:1–12。[出版社](https://www.sciencedirect.com/science/article/pii/S0967066117301491)。速度/加速度/角速度运动学观测，不需完整轮胎参数；传感偏置、姿态/低激励仍关键。正文获取限制见开头，未确认官方代码。低维观测器O(n³)，适合启发低复杂度vy诊断；不能照搬直行侧滑归零启发式。

**Three-Dimensional Vehicle Dynamics State Estimation for High-Speed Race Cars under varying Signal Quality**；Sven Goblirsch, Marcel Weinmann, Johannes Betz；2024 IROS。[原文](https://arxiv.org/abs/2408.14885) / [代码](https://github.com/TUMFTM/3DVehicleDynamicsStateEstimation)。九状态点质量EKF结合参考角与单轨UKF虚拟速度；真实赛车，包含RTK-GNSS辅助，不是GNSS-free证明。小状态计算较轻，但轮胎/转角/路面假设需本车验证。借鉴姿态-侧滑辨识边界，不搬GNSS质量逻辑或复杂轮胎参数。

## 综合判定

成熟的是多峰概率表示、方向性几何、延迟估计的数学工具；本系统收益尚未被这些论文证明。低成本顺序：观测生命周期/参考点审计 → vy与分雷达方向诊断 → NDT/small_gicp相同输入Shadow → 双分支独立历史。大粒子15D、完整固定滞后图及新GNSS完整性算法暂不实施。作者性能数字与本机实测分栏，不能交叉替代。
