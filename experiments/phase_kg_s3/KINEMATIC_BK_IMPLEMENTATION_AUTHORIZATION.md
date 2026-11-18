# KINEMATIC_BK implementation authorization — 2026-11-17

用户已批准 `B_KINEMATIC_PREDICTOR_CONTRACT.md`（SHA256 `969901871464091466a5bd2f09a74a1d838ab584102dbdba73917d6d282f0717`）并授权正式实施、配对诊断及完整连续回放。原合同作为已批准版本保留，不覆盖其文本或旧实验。

三项决定已冻结：一次性固定 `H_init` 继承原首合法扫描的局部轴，此后不从Legacy/global取运动、Bias或Pose校正；首个不可恢复输入缺口终止新分支并报告有效覆盖率与剩余范围；暂不新增滑移、速度跳变或加速度门限，body横向/竖向速度零为明确假设。旧运行 `B-K` 仍是关键帧Target标签，新预测器只称 `KINEMATIC_BK`。

输入调度必须区分 `DELAY`、`IRRECOVERABLE_INPUT_GAP`、`RESOURCE_FAILURE`。原cloud available时输入不合法，只记DELAY；后续真实输入事件触发重试时显式记录新的query available，不能把晚到观测倒填到原释放时刻。严格按扫描顺序提交局部链，保留原Source版本；沿用批准的缓冲/队列资源约束，不新增等待、运动或滑移数值门限。所有真实等待、匹配、重试和排队开销进入延迟/周期记录。

完整原始IMU/Wheel测量时间/可用时间审计可用于区分数据真缺口和晚到，但不能把未来观测值用于更早query available的Seed。若完整输入也无法满足25ms IMU/100ms Wheel held覆盖，终止并保留前缀；若资源无法维持待处理队列，单独报告资源失败，不冒称物理丢包。不得补零、伪造速度、静默重初始化、改Source/Target/ROI/NDT模型或退回全局后验。

总代理独立复核数值测试、初始化和输入因果性、固定Source/Target配对归因、独立O链全程结果及性能；A与Legacy B继续，不等待KINEMATIC_BK。此授权不允许修改生产代码或新增LiDAR融合路径。
