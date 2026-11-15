# 六项结论与停止边界

1. **Fitness是否独立于匹配优化？** 当前实现独立：优化和Native质量先完成，Fitness使用单独raw-point评价分支；未见其反馈到优化或Native covariance。上层Legacy仍依赖Fitness构造R，不能把“独立于优化”扩大为“独立于整个定位系统”。此外PCL分支仍位于align共同try/catch中：若发生CUDA/资源异常，catch会置success=false、converged=false；可选raw索引也影响内存和初始化失败面。因此不是完整故障隔离，本次未注入故障、未验证异常等价。
2. **OFF保持结果吗？** 本次三帧、各三轮ON/OFF均与P2历史等价，所有受检匹配/Native字段数值差0；正常、收敛偏移及MAX_ITERATIONS三类均覆盖。有限样本不证明所有场景永久逐bit一致。
3. **节省多少？** 后两轮配对align节省中位：60帧36.197ms、65帧36.547ms、76帧35.691ms，范围34.816–36.757ms。全3轮数据保留，不隐藏首轮；不报告P95/P99实时通过。
4. **来自计算还是等待？** 移除了完整PCL kernel/D2H/同步阶段，尾部未承接相同量级等待；但PCL计时覆盖CPU等待/GPU执行等，不能精确划出各自份额。CUDA纯kernel事件时间UNKNOWN。
5. **值得做低成本、数值等价Fitness吗？** CONDITIONAL。若Legacy R或必要诊断继续依赖现有定义，约36ms成本值得单独立项；应保持raw-point距离、shell6边界、miss不进入均值分母及计数等定义，并验证边界/稀疏/重复结构、数值容差和完整耗时，不能用NDT score或Native residual伪装数值等价。若未来R不再依赖Fitness，先判断是否需要每帧计算，再决定优化是否值得。**本轮没有实现或批准新Fitness算法。**
6. **未来不靠Fitness构造R时保留什么？** 保留优化退出/收敛、有限位姿、score及支持计数、Native完整矩阵/残差尺度/退化方向、创新与更新生命周期等独立诊断；不能合成未经证实的正确性标签。Fitness可按明确可用性、时间戳和计算频率保留作离线/可选诊断；若少算或异步，不能把旧值当当前观测，也不能继续自动生成Legacy R。具体接入需另行审查批准，Native Raw不能直接替代生产R。

匹配错误依旧存在：65仍收敛偏移，76仍35次MAX_ITERATIONS；关闭Fitness没有修复目标/几何问题。本阶段是性能解耦验证，不是新定位算法或精度认证。生产、Legacy/Native规则、Q/Bias、GLL/收敛/迭代保持原状。当前停止，不启动后续Fitness优化、滤波回放或生产变更。
