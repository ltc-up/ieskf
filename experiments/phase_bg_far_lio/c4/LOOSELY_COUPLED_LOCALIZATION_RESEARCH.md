# 可借鉴的松耦合机制与概念边界

本轮实际阅读固定源码及官方说明，补充原文检索。没有运行其他框架或复制其滤波器。原X-ICP、SuperLoc、Switchable Constraints全文/源码审查复用[BG-C3记录](../c3/ROBUST_OBSERVATION_LITERATURE.md)，不把缓存实现冒称原论文逐字源码。

| 项目/原文 | 真实机制、所需信息 | 对当前系统的借鉴与风险 |
|---|---|---|
| robot_localization，官方滚动分支固定b1e3731；Moore/Stouch, *A Generalized Extended Kalman Filter Implementation for the Robot Operating System*, IAS2014 | 通用EKF/UKF融合外部pose/twist/IMU；按测量时间队列处理，lagged rollback+测量/控制历史，独立配置current-time预测；map/odom按REP105分工 | 借鉴时间、历史与frame合同，不把其15变量视为Bias ESKF；参数式NIS拒绝并不能解决错峰。P近似依赖输入协方差及相关性，不能自动认证共享信息 |
| Autoware EKF Localizer，官方core 9799c6d | 平面六状态、time-delay增广、pose/twist平滑队列、有界输入及诊断 | core/node分离、有界队列值得采用；不能继承重复平滑/R×5作为真实噪声标定；z/roll/pitch不等价于完整惯性传播 |
| hdl_localization，Koide维护，固定de2dc77 | 先验地图NDT输出pose，IMU驱动16维UKF；可合odom UKF作初值，观测为p+quaternion | 最贴近外部pose松耦合范式；所审`correct`直接送入最终NDT结果，不能照搬成鲁棒门控；两预测相关性、Quaternion统计与固定R需独立评估 |
| FAR-LIO，官方ef205103 | 原生GICP+body速度9状态、FIR/倾角辅助、历史位移补偿迟到pose、R adaptation/floor、ROS persistent缓存 | 运动学、生命周期与时序值得拆开消融；不能复制重复缓存或将缺失P导数当优点。此次横向优势尚无完整机制因果 |
| Xu/Cai/He/Lin/Zhang, *FAST-LIO2: Fast Direct LiDAR-inertial Odometry*, T-RO2022 | 紧耦合直接点到地图配准、迭代误差状态滤波、增量ikd-Tree；高频惯性传播/状态反馈 | 可借鉴真实时间惯性预测和去畸变/地图更新职责；不是外部NDT pose的即插即用后端，不证明同点云外部pose可忽略相关性 |
| Shan/Englot/Meyers/Wang/Ratti/Rus, *LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping*, IROS2020 | 因子图、IMU预积分、LiDAR里程计与可选GPS/闭环；维护高频IMU预测与低频优化 | 可借鉴优化修正后重置/重积分预测前史与发布分工；不能将因子图结果当当前松耦合EKF的协方差/实时性证明 |
| Barrau/Bonnabel, *The invariant extended Kalman filter as a stable observer*, TAC2017，arXiv1410.1465 | Lie群不变误差，对满足结构的系统研究误差自治/局部稳定 | Invariant不是Iterated。可能改善特定误差线性化的一致性，但不能修复错误对应或地图缺失，也不支持直接删除Bias。当前没有实现/实测此变体 |
| Tuna等，*X-ICP: Localizability-Aware LiDAR Registration for Robust Localization in Extreme Environments*, T-RO2024 | 对应级方向可定位性、部分/不可定位约束及受控求解，而非最终Hessian一个数 | 可保留强横向约束，但需NDT对应级重定义；重复结构错误分支仍可能局部充分，约束弱方向也可能压制正确大修正 |
| Zhao等，*SuperLoc: The Key to Robust LiDAR-Inertial Localization Lies in Predicting Alignment Risks*, arXiv2024 | 配准前几何方向风险、不同里程计先验/因子 | 借鉴前置风险与后置曲率互补；不能把标签数当统计R，也不能将自身IMU预测当新独立证据 |
| Sünderhauf/Protzel，*Switchable Constraints for Robust Pose Graph SLAM*, IROS2012 | 对关联添加switch及先验，联合图优化 | 可启发可撤销观测账本；单prior/单pose时仍可能等价残差拒绝，错峰持续平滑可骗过开关，条件P可能遗漏开关不确定性 |

官方原文与代码：

- [robot_localization源码/文档](https://github.com/cra-ros-pkg/robot_localization/tree/b1e373177cc8190f2a424243a73a1023c882140f)、[REP105](https://www.ros.org/reps/rep-0105.html)。
- [Autoware官方说明](https://autowarefoundation.github.io/autoware_core/main/localization/autoware_ekf_localizer/)、[固定源码](https://github.com/autowarefoundation/autoware_core/tree/9799c6da7cdf4cb0bd9d9766e62acb406fd2f73c/localization/autoware_ekf_localizer)。
- [hdl固定源码](https://github.com/koide3/hdl_localization/tree/de2dc7769aa2412876d05ae6161408d3295b6a3c)、[FAR官方](https://github.com/TUMFTM/FAR-LIO/tree/ef20510316ad1f8dbf2e428e4a193f700493eccd)、[FAR原文](https://arxiv.org/abs/2606.26010)。
- [FAST-LIO2原文](https://arxiv.org/abs/2107.06829)、[官方代码](https://github.com/hku-mars/FAST_LIO)。本轮arXiv HTML路径404，保存失败，不声称重新全文核验其全部推导；以官方论文/摘要和仓库说明限定上述机制。
- [LIO-SAM原文](https://arxiv.org/abs/2007.00258)、[官方代码](https://github.com/TixiaoShan/LIO-SAM)。
- [Invariant EKF原文](https://arxiv.org/abs/1410.1465)。稳定性结论有系统结构/局部条件，非全局错峰保证。
- [X-ICP原文](https://arxiv.org/html/2211.16335v4)、[后续相关实现](https://github.com/leggedrobotics/perfectlyconstrained)。
- [SuperLoc原文](https://arxiv.org/html/2412.02901v1)、[官方实现](https://github.com/superxslam/SuperOdom)。
- [Switchable Constraints作者原文](https://nikosuenderhauf.github.io/assets/papers/IROS12-switchableConstraints.pdf)、[Vertigo](https://github.com/OpenSLAM-org/openslam_vertigo)。

## 对当前数据的数学判断

Error-State是围绕名义状态的局部扰动；Iterated是同一观测重线性化求解，必须保持同一prior的信息计数；Invariant是在群作用下选择误差结构。三者互不等同。每100Hz tick把同一LiDAR当新测量也不等于合法的Iterated更新。

外部pose松耦合合同至少包括z、6×6 R图、参考点、观测/可用/决策时间、唯一ID、使用次数、生成该pose的prior/地图来源。协方差R只描述已选分支局部误差，不能替代分支概率。几何信息、经验权重、统计协方差、风险与是否接纳分开记录。

计算上：原状态预测O(n³)的小矩阵常数可控；回滚随历史事件数增长；对应级几何O(N)但邻域搜索/法向可能主导；跨候选需额外配准就必须付完整前端成本。100Hz独立输出只能隔离发布调度，不能消除GPU/输入积压，也不能保证50ms全周期。本轮没有宣称以上论文在A2RL产生新收益。
