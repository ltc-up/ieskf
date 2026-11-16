# Phase BG Native有限测试交付

本阶段按批准补齐固定版本软件依赖、实现独立输入适配、完成Native构建及预先冻结的normal8/right3。**没有下载数据集、bag、地图或Docker镜像；没有修改生产/NDT/15D、原地图或TF；没有全圈/三圈。**

系统对照 **NO-GO**：正常段原始配准有局部收益，交付状态未优于已有15D；右弯出现两次约10m工程参考差的收敛候选。50ms完整周期未通过，且有限测试受到明确IO/时钟调度停顿影响，不可直接判定FAR核心算法劣于NDT。

- [输入合同](INPUT_CONTRACT_RESULTS.md)
- [构建与存储](BUILD_AND_STORAGE.md)
- [精度与公平性](ACCURACY_AND_FAIRNESS.md)
- [性能](PERFORMANCE.md)
- [垂向与连续性](VERTICAL_AND_CONTINUITY.md)
- [失败与机制](FAILURES_AND_MECHANISMS.md)
- [P4与三雷达分布](SPATIAL_DISTRIBUTION.md)
- [最终建议](FINAL_CONCLUSION.md)
- [复现与边界](REPRODUCE.md)

CONFIRMED只指所审代码和保存记录内成立；HYPOTHESIS需单独实验；UNKNOWN不得推断通过。本轮Native指原生编译，不是把NDT Native协方差切入生产。
