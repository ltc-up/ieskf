# 事件驱动内核与100Hz当前状态输出合同

## 现状：C1并非独立100Hz发布器

CONFIRMED：`c1/run_continuous_15d.py`逐IMU事件进入LegacyComparisonReplay，按header以持有的上一条IMU传播到下一事件；Wheel有自身header、原模型及观测更新；LiDAR以扫描header历史插入并重传播。IMU传播没有降成100Hz。迟到NDT完成后`current()`返回最新已传播状态，并记录一次ndt_completion。

`logstate()`只在初始化、Wheel和NDT完成时执行；没有ROS状态Publisher或独立10ms状态timer。`ClockPump`是/clock，不是状态发布。实际保存25295条：Wheel21080、NDT完成4214、初始化1。其均值约120Hz不能证明均匀100Hz。

保存记录到达间隔P95=11.354ms，max=16.914ms；状态header最大间隔12.084ms。135s至345.806s虚拟10ms网格共21081槽：259槽没有新记录、4376槽多条记录，258对相邻槽复用同一已保存快照。首槽没有已保存初始化状态，不能用稍后结果回填。

只读因果查找（仅选到达时间≤tick的记录）显示：可用21080槽中保存状态年龄P95=6.462ms、max=15.934ms；最新已交付IMU header与保存状态header差绝对P95=4.808ms、max=12.135ms。这是**日志可见状态的诊断**，内核在未记录的IMU事件上还会前进，不能把日志滞后当成内核实际滞后。真实timer调度、同步开销和100Hz完整性UNKNOWN。

## 新合同设计（未实施）

采用事件驱动估计内核+独立10ms输出。内核唯一写者处理原IMU、Wheel、clock及NDT事件；发布侧只读取原子不可变快照，不调用观测更新。每个NDT唯一ID只对应一条信息记录；发布timer绝不重新融合旧NDT。

输出包至少含：sequence、scheduled_tick、actual_publish_time、state_time、state_age、latest_available_IMU_time、last_NDT_observation/decision_time、revision_id、有效性及原因、位置/姿态/速度、完整15×15 P。明确state_ref=IMU、p/v在map、SO(3)右误差；如另提供base位姿/速度，须具名且用现有杆臂/角速度及Jacobian变换，不把IMU速度或P冒称base参考点。

优先输出最新已传播快照，header如实标状态时间；不能只把旧状态stamp改成tick。若要求预测至tick，可在**副本**上使用已到达且时间合格的真实held IMU做有界短时预测，同时传播P。该预测不是新增IMU观测，不反馈内核；上限建议不超过一个10ms周期且不超过既有IMU有效年龄，具体数值须预注册测试，当前不改变任何阈值。超界输出STALE/INVALID并记录原因，不伪造测量、跳过输入或静默追赶。

历史修正期间不要让输出线程抢锁等待整个重传播：内核事务完成后原子替换mean/P/time/revision；一个输出包不得拼接旧均值与新P。事务前包可明确标旧revision与年龄；完成后下一tick读新revision。此举不保证地图位姿无跳变；控制连续性与map/odom设计必须单独证明，不用低通掩盖修正。

## 发布侧数值等价与验收方案

纯接口候选：同事件序列、有/无发布器，逐事件内核mean/P/账本hash相同；发布副本不污染core，重复tick不增观测次数。若启用副本短预测，比较内核仍应相同，输出需对照同held-IMU模型的参考计算。检查乱序、NDT事务跨tick、IMU中断、首次初始化、过期、重复ID及P对称/SPD。所有时间使用整数ns且单调调度，保留bag可用时刻与真实处理/发布时刻。

验收不是平均频率：报告每10ms槽输出/缺失/迟到/重复、state_age P50/P95/P99/max、deadline overrun、revision一致性、最新可用IMU差。并测发布复制/序列化/队列开销及NDT全周期50ms；不把C1的28.888ms历史判定max当新接口保证。新实现及实际调度运行尚未执行，当前100Hz部署结论UNKNOWN。
