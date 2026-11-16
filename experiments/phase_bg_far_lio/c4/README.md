# BG-C4：松耦合架构、FAR归因与1013错峰

基线 `5d784066c7dca12407e78d567531618c008e06b6`。本轮完成35次CUDA冻结匹配、原生FAR函数的同条件合成消融、C1保存日志的受限prior恢复，以及实际ROS2合成100Hz发布。没有新真实连续闭环、生产修改、数据集/镜像下载或清理。

**PARTIAL：机制证据增加；完整FAR因果分解、1013下15D污染闭环和真实100Hz/50ms系统验收仍未完成。** 不把数学/接口通过包装成定位成功。

1. [高频架构审查](HIGH_RATE_LOCALIZATION_ARCHITECTURE_REVIEW.md)
2. [FAR横向归因](FAR_LATERAL_ADVANTAGE_CAUSAL_ANALYSIS.md)
3. [1013故障恢复](LOOP44_1013_FAILURE_RECONSTRUCTION.md)
4. [文献与源码](LOOSELY_COUPLED_LOCALIZATION_RESEARCH.md)
5. [最多三种候选](ROBUST_FUSION_CANDIDATES.md)
6. [真实执行结果](CONTROLLED_EXPERIMENT_RESULTS.md)
7. [100Hz原型](100HZ_STATE_ESTIMATION_PROTOTYPE.md)
8. [架构决策](NEXT_ARCHITECTURE_DECISION.md)
9. [结论](FINAL_CONCLUSION.md)
10. [复现与证据边界](REPRODUCE.md)、[存储/Git](STORAGE_AND_GIT_AUDIT.md)

公开文件只含研究代码、标量诊断和身份；完整原始点云、位姿、P/H/K/R和运行记录保留在授权磁盘私有C4目录。工程参考不是独立真值。分类：CONFIRMED=本源码/执行直接支持；HYPOTHESIS=尚无因果消融；UNKNOWN=证据不足；NO-GO=不满足指定资格。
