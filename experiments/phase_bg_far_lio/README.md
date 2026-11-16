# Phase BG — FAR-LIO只读审查交付

研究基线 `0d5ffecbc6cce0b2862d40fb38a6efe29650fed6`。本阶段仅审查已有缓存、论文、环境及1秒bag元数据；未构建、未拉镜像、未回放、未运行FAR-LIO。研究分支保持独立，生产不变。

**定位收益、困难段改善、50ms及连续性：UNKNOWN / NOT_RUN。当前不能直接运行：融合点云丢逐点时间，IMU轴/参考点适配及全局初始化合同待完成。**

- [源码与论文](SOURCE_AND_PAPER_REVIEW.md)
- [输入与地图](A2RL_INPUT_AND_MAP_AUDIT.md)
- [环境、存储及待审批方案](ENVIRONMENT_AND_STORAGE.md)
- [预先冻结的有限运行计划](MINIMAL_RUN_PLAN.md)
- [公平比较](FAIR_COMPARISON.md)
- [失败与未执行结果](RESULTS_AND_FAILURES.md)
- [决策](FINAL_RECOMMENDATION.md)
- [复核](REPRODUCE.md)

CONFIRMED表示所审源码或记录内已核实，不等于部署/物理认证；HYPOTHESIS为待实验解释；UNKNOWN为证据缺失。完整私有时间、TF、路径与原记录不公开。
