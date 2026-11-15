# KITTI BF1只读复盘

审查基线：`49c3aad8d666deea3a9165117d09dd526df5360b`。本次仅分析已经保存的57帧KITTI结果、完整R/K/P候选、耗时记录及49个A2RL配对冻结样本；没有重新匹配、滤波更新、回放、下载或参数修改。输入文件审查前后SHA相同。这里的“只读”指实验输入和算法，新增内容仅为分析脚本、统计及报告。

**结论：维持功能验证CONDITIONAL、50ms实时性NO-GO、生产推进NO-GO。** 不从单一KITTI场景决定生产方案。RACECAR下载被用户明确否决，旧申请归档为备选，不再作为下一阶段。

- [末段MAX_ITERATIONS](MAX_ITERATIONS_ANALYSIS.md)：末段支持减少、纵向弱约束和优化偏移同时存在；种子并没有先漂移数米。第64/65帧已收敛但纵向匹配变差。缺少逐次迭代轨迹，不能确认震荡或唯一根因。
- [Native收益与代价](NATIVE_GAIN_AND_ERROR_ANALYSIS.md)：整体XY改善来自纵向，横向和Yaw代价可见；约88%的XY误差平方改善累积在两组均无NDT更新的最后11帧。不是错误匹配识别能力提高。
- [耗时归因](TIMING_ATTRIBUTION.md)：NDT对齐约占实测总耗时78%；纯IMU积分、纯IO和部分诊断没有独立计时，不能伪造精细拆分。共同到达时刻是反事实实验时钟。
- [A2RL机制对照](A2RL_MECHANISM_COMPARISON.md)：Native方向性和P耦合跨场景不同，小增量不能证明精度；已有错误盆地可有良好Hessian。
- [最小后续实验建议](MINIMAL_NEXT_EXPERIMENTS.md)：仅提出待批准的5帧迭代轨迹与计时补全，不执行、不改优化器。
- [复现及核验](REPRODUCE.md)、[聚合统计](AGGREGATES.json)、[版本清单](manifest.json)。

参考仍为OXTS工程参考；建图参考、接收机导航及IMU测量可能相关。不能视为独立真值或绝对Native标定。0/75/150ms原结果全部保留，本文机制分解明确采用0ms与相同发布状态时刻，不重新挑选时间偏移来优化排序。
