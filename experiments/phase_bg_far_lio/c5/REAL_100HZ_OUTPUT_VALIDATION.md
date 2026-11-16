# 真实输入负载下100Hz只读发布

CONFIRMED：复用C4 SnapshotStore；事件按原IMU/Wheel/NDT时间推进，独立10ms线程发布不可变mean/P快照，不重复融合LiDAR，不改状态时间戳。ROS2本机隔离域实际发布/订阅完整状态、P、revision、真实state_time/age及地图/运动双健康状态。输出位置参考点明确为IMU；base位置需使用同一快照姿态/杆臂转换。无虚构未来IMU预测。

39501预期/实际/唯一收到时隙，缺失0、重复0、倒退0、>10ms晚时隙0。501条UNINITIALIZED对应初始化前及边界，没有伪装有效；39000条初始化后状态均未STALE。state_age P50/P95/P99/max = 3.070/6.312/8.086/12.795ms；调度迟到=.052/.343/.634/2.452ms。

复制mean/P P95/max=.0629/1.1656ms；JSON序列化=.1020/.4586ms；publish API=.0216/.1631ms。DDS精确内核传输成本UNKNOWN；loopback收包记录包括序列化、排队和调度，不能减法视为纯DDS。不是车辆控制链端到端测量。

39000条完整P均有限、对称，最小特征值4.9984e-7、最大不对称0；原子锁覆盖mean/P/revision及新鲜度元数据。state_time相对最新已处理IMU时间可能前进0～14.318ms（Wheel/scan事件按既有held输入推进），不能声称所有状态恰好等于最新IMU采样；age也不等于IMU年龄。两字段分别保留。

地图健康396个时隙为MAP_UPDATE_STALE，但运动状态仍新鲜；38589近期更新、516尚无接纳。地图分支正确性始终UNKNOWN，不能因CURRENT就声称地图可靠。

确认数值等价：真实末段检查点至525s的3714事件，关闭发布器并以原ExternalPose替代TraceObserver，逐事件mean/P位级哈希与实际运行相同，结果码相同。每步PSD/有限检查通过。另有33项预运行测试。此为冻结事件内核等价，不是全程第二次匹配闭环，也不验证发布器不存在任何性能影响。

CONDITIONAL：本机研究负载下100Hz当前状态与低年龄合同通过；NO-GO：据此直接授予生产DDS/控制安全资格、100Hz LiDAR更新或观测统计一致性。
