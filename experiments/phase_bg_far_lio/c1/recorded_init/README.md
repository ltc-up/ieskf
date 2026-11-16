# BG-C1：一次录制EKF均值条件下的pit连续启动

本目录是用户追加授权后的实际运行交付，取代上一阶段“历史初始化标志缺失导致停止”的执行决定；旧预检及GlobalInit失败证据仍保留。

- [初始化与模型审计](INITIALIZATION_AUDIT.md)
- [时间合同和空间](TIMING_CONTRACT_AND_BUDGET.md)
- [完整评分对照](CONTINUOUS_COMPARISON.md)
- [性能与全部失败](PERFORMANCE_AND_FAILURES.md)
- [决策](FINAL_CONCLUSION.md)
- [复现](REPRODUCE.md)

原始轨迹、输入身份、完整P/候选与源码私有快照位于用户新加卷独立目录，未公开受限原始数据。公开聚合不能替代独立真值认证。

> 后续源码勘误：关于“均值仅用一次”、固定Bias及非完整约束的机制解释，请同时阅读[BG-C2勘误](../../c2/ERRATA_BG_C1.md)。原运行、轨迹、参数与评分未改变。
