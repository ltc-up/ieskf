# 鲁棒观测文献与源码复核

本轮实际检索原文、作者页面及公开源码，并复用本机缓存；没有下载数据集、仓库或依赖。下面“文献机制”和“本项目判断”分开，后者是研究推断，不冒称论文在A2RL验证过。网页访问日期2026-11-16；缓存commit与关键小源码SHA见manifest。

## A. X-ICP

Turcan Tuna、Julian Nubert、Yoshua Nava、Shehryar Khattak、Marco Hutter，**X-ICP: Localizability-Aware LiDAR Registration for Robust Localization in Extreme Environments**，T-RO 40 (2024)，预印本2022。[原文v4](https://arxiv.org/html/2211.16335v4)、[作者项目](https://sites.google.com/leggedrobotics.com/x-icp/home)。

CONFIRMED文献机制：点到平面对应先回LiDAR局部坐标，分平移/旋转分析主方向及各对应贡献，过滤并分类可定位/部分可定位/不可定位；在ICP求解中加入受控约束，退化方向不自由漂移。它不是直接用最终Hessian条件数拒绝整帧。

源码复核使用缓存`perfectlyconstrained` commit `0fbe4175ea205a271f85287abfd8048e5f7dd32a`，其中PointToPlane.cpp包含Equality/Inequality等路径及对应localizability测试。这是后续“Informed, Constrained, Aligned”项目中的相关实现，**不能冒称与原X-ICP论文代码逐字等价**。[官方后续实现](https://github.com/leggedrobotics/perfectlyconstrained)。

本项目判断HYPOTHESIS：可以借鉴每方向证据分离，保留强横向观测，而非整帧拒绝。但当前NDT是体素高斯目标，需重新定义对应/尺度/耦合，不能复制ICP阈值。局部几何充分也可能匹配到重复护栏错误分支；退化方向若存在正确大修正，冻结它也会误拒。已有C1仅聚合Hessian/支持，缺每对应贡献身份。额外对应遍历约O(N)，小矩阵特征分析便宜，但当前完整50ms成本UNKNOWN；不在本轮实现匹配器约束。

## B. SuperLoc

Shibo Zhao、Honghao Zhu、Yuanjun Gao、Beomsoo Kim、Yuheng Qiu、Aaron M. Johnson、Sebastian Scherer，**SuperLoc: The Key to Robust LiDAR-Inertial Localization Lies in Predicting Alignment Risks**，arXiv 2412.02901，2024。[原文v1](https://arxiv.org/html/2412.02901v1)、[官方源码](https://github.com/superxslam/SuperOdom)。

CONFIRMED文献机制：配准前从几何约束方向统计风险/可观测性，联用其他里程计相对pose prior与LiDAR/IMU因子。标签占比不是统计标定协方差；文中还给出退化经验界，不能简化宣传为完全无需判断参数。文献报告45ms/帧来自其CPU/任务设置，不是本系统20Hz认证。

缓存源码commit `f10e65cd50007767b22e4c401689665e20d827d6`，LidarSlam.cpp::FeatureObservabilityAnalysis约583行使用法向、主方向、旋转轴叉积和feature observability分类，约344行累积约束直方图；这比只读最终score提供更细的几何信息。没有构建或运行该项目。

本项目判断HYPOTHESIS：适合研究前置风险与后置Hessian互补；需要原Source/地图邻域及来源标签，现有聚合无法全部补出。先验若就是自身IMU/滤波结果，不能当独立约束重复计权；强动态侧滑或预测误差时也会压制正确大修正。label-count是相对覆盖，不处理多峰正确性、跨帧相关性或绝对R尺度。新增法向/对应搜索可能主导开销，50ms可行性UNKNOWN。

## C. Switchable Constraints / 鲁棒估计

Niko Sünderhauf、Peter Protzel，**Switchable Constraints for Robust Pose Graph SLAM**，IROS2012。[作者原文](https://nikosuenderhauf.github.io/assets/papers/IROS12-switchableConstraints.pdf)、[作者项目](https://nikosuenderhauf.github.io/projects/switchableConstraints/)、[Vertigo源码](https://github.com/OpenSLAM-org/openslam_vertigo)。

CONFIRMED文献机制：给潜在错误闭环边增加开关变量和趋向开启的先验，联合优化pose与图约束；不是对每条大创新直接判错。代表项为‖s·r(x)‖²_R⁻¹+‖1−s‖²_Ξ⁻¹。源码树中edge_se2Switchable有两个pose及第三个switch顶点，edge_switchPrior单独约束switch；本轮网页确认源文件结构与原文目标，未构建Vertigo，未核验所有模板头/当前commit与2012实验逐项等价。

本项目判断HYPOTHESIS：可以启发有限窗口关联风险账本。若只剩一个prior与一个LiDAR候选，开关仍主要由残差决定，可能拒绝正确大修正；稳定错误分支也可能保持开启。不能把原pose-graph后端直接搬成单次ESKF NIS门。条件重加权得到的P不自动包含switch不确定性，关联与跨帧相关性需单独建模。小窗口求解增加状态/重线性化工作量，未经测量不能认为毫秒级。当前不引入因子图或拟合switch参数。

## D. FAR-LIO

**FAR-LIO: Enabling High-Speed Autonomy through Fast, Accurate, and Robust LiDAR-Inertial Odometry**，arXiv2606.26010（2026）。[原文](https://arxiv.org/abs/2606.26010)、[官方源码](https://github.com/TUMFTM/FAR-LIO)。本轮主要使用已核验缓存commit `ef20510316ad1f8dbf2e428e4a193f700493eccd`及C1实际运行，而不按论文名称推断机制。

CONFIRMED源码/运行：body速度九状态、原生倾角参考、低速gyro Bias学习；按自身历史位移补偿迟到pose，pose covariance随延迟放大并floor。ROS received缓存每tick再提交，不能借用其“连续输出”来证明一次信息接入。C1评分段横向R恒.05，位置残差裁剪未触发。详细证据见[BG-C2](../c2/README.md)，本轮没有再次运行。

本项目判断：可借鉴坐标明确的body运动与估计/发布解耦，不复制缓存重复观测。延迟补偿共享自身历史，遗漏相关性问题不能靠固定倍率证明解决；FAR全段50ms已经失败，不能视为免费鲁棒机制。

## E. 现有15D及组合研究判断

CONFIRMED：完整P、Bias、SO(3)reset、迟到测量历史更新和事件重传播已有源码与C1记录；这些是本项目实现，不是文献保证的真实统计正确性。C1 Legacy没有终态撤销账本，保存匹配/Deskew不会随历史prior改变而重算。

共同启示（HYPOTHESIS）：把局部可定位性、预测一致性、有限时序分支证据分开记录，同时保留测量时间/可用时间/决策时间。不能用论文成功案例保证当前重复护栏判别；未来几何或时序候选仍须在历史错峰和正确大修正上独立检验。没有任何一篇文献使未经标定Native或简单NIS门自动成立。
