# 高频定位架构：实际源码对照

CONFIRMED：源码固定为生产aa50e354、FAR ef205103、TAM common 4b19b608、本研究5d784066。新增官方小源码总279159B，commit及文件SHA见[source_inventory](source_inventory.json)。仅审查，不构建替换这些系统。Autoware新版本与本地生产不是同一个版本，不能混称当前部署。

| 系统 | 预测、融合、输出的实际顺序 | 迟到/无观测 | 可借鉴与限制 |
|---|---|---|---|
| 生产A2RL六状态 | `EKFLocalizer::timerCallback`：检查激活→实测dt→`predictWithDelay`→pose队列→twist队列→高度/姿态及输出。100Hz配置，不是完整3D IMU积分 | 有限增广历史15步；`pop_increment_age`最多五次；新pose拒绝不删除历史队列 | 接口、诊断、激活合同可保留；不能把历史重复平滑当独立信息或复制到15D |
| FAR原生9状态 | `function_queue_callback` watchdog→`model_update_callback`。每tick再次补偿所有received缓存→set输入→`step`预测/校正→输出/debug→FIFO压入当前状态。IMU回调更新handler，模型TS=.01 | 自身FIFO位移补偿，不是历史P回滚；超FIFO失败设ERROR；watchdog影响有效mask；输出时间now−配置平均输入延迟 | body速度/输入管理值得研究。received不消费导致旧LiDAR重复信息，不能当合理100Hz策略；debug与实际校正不在同一时间点 |
| 本研究15D | IMU真实header/dt传播；Wheel/pose按事件接入；迟到pose恢复检查点并重放，C1窗口500ms；Joseph/reset | 没有NDT时IMU继续；过旧事件按历史合同拒绝。C1输出随事件/处理完成，不是独立均匀10ms | 保留真实事件、完整P及回滚；补独立发布、版本、健康状态，不强制降低IMU频率 |
| robot_localization | 固定发布周期调用`integrateMeasurements`，优先队列按传感器时间处理；`FilterBase::processMeasurement`仅正dt预测再correct | `smooth_lagged_data`可回退状态与测量队列重放；`sensor_timeout`触发纯预测；`predict_to_current_time`是独立选项 | 时间排序、超时、历史测量/控制恢复值得借鉴；其15个状态是Euler/速度/角速度/加速度，**不是本项目Bias ESKF** |
| 新Autoware core | Node timer→`EKFLocalizer::update_step(t_curr)`→drain异步输入→激活/初始化→dt预测/历史→pose/twist aged队列→发布 | 新队列有size/age与丢弃诊断；仍保留aged重复平滑 | 清楚的core/node分离、有界输入；没有替我们解决重复信息或3D重力/Bias问题 |
| hdl_localization | 16维UKF名义向量含p/v/quaternion/bg/ba；IMU timestamp预测，冷却期跳过；NDT以预测为初值，7维pose校正 | 另有odom UKF初值融合；检查的`PoseEstimator::correct`中NDT最终结果直接用于UKF，不存在可证明的全局错峰保护 | 是实际松耦合外部pose案例，不是15D ESKF数值等价替代；固定观测噪声与共享预测相关性仍需审查 |

官方固定源码：[robot_localization](https://github.com/cra-ros-pkg/robot_localization/tree/b1e373177cc8190f2a424243a73a1023c882140f)、[Autoware](https://github.com/autowarefoundation/autoware_core/tree/9799c6da7cdf4cb0bd9d9766e62acb406fd2f73c/localization/autoware_ekf_localizer)、[FAR](https://github.com/TUMFTM/FAR-LIO/tree/ef20510316ad1f8dbf2e428e4a193f700493eccd)、[hdl](https://github.com/koide3/hdl_localization/tree/de2dc7769aa2412876d05ae6161408d3295b6a3c)。

## 三种可行架构

| 选择 | 时间/信息合同 | P/迟到处理 | 控制与代价 |
|---|---|---|---|
| A 固定100Hz全流程 | 必须将每段IMU积分恰好一次，不能每tick重复最新样本/旧LiDAR | 队列按测量时间消费；不得以tick数充当实际dt | 实现简单但容易把GPU/IO阻塞传给输出；适合本来就是平面输入模型的工程，非本研究默认 |
| B 事件估计+独立100Hz输出 | IMU真实dt，Wheel/pose唯一ID；发布最新已传播状态及真实状态时间/年龄 | 完整事务完成后原子提交mean/P/版本；回滚中不发布中间历史状态 | 本轮实现此最小候选；输出不改轨迹。快照复制、序列化、调度需分别计时；无输入必须STALE |
| C 连续局部惯性状态+地图修正 | 高频local/odom与低频map约束分工，不能把两个相关状态视为独立观测 | 必须维护关联或Schmidt/cross-covariance，历史更新与map修正版本一致 | 更适合控制连续性研究，但本轮未实现完整分离滤波，不能靠低通或换TF隐藏跳变 |

推荐B先验证，再独立审查C。输出header必须等于状态真实时间，另给publish/scheduled/state_age；需要短时预测时只在副本用因果ZOH输入、传播P并设最大跨度，不能假造IMU。本轮不做这种副本预测，最新IMU年龄直接可见。

map→odom→base关系采用同一时刻T_MB=T_MO T_OB。改变T_MO不保证使用map路径的控制误差连续；控制参考路径与状态必须在同一版本变换。坐标变换要对完整P作Jacobian映射，不能把IMU原点P当base P发布。本轮只输出IMU原点研究包，不发布控制TF或伪装生产消息。

UNKNOWN：实时部署timer、控制器路径版本与故障策略尚未实车验证。源码提供能力不等于部署或实验每次执行了该能力。

## 生产TF与姿态/高度的额外核对

CONFIRMED：本地生产timer从z/roll/pitch三个独立1D filter取值，与平面状态拼接`getCurrentPose`，发布map坐标Odometry、child=base_link；它不是完整15D耦合3D预测。`timerTFCallback`函数虽存在，但本版本构造中的TF timer创建被注释，不能仅因函数存在声称部署正在发布该TF。实际运行TF树/控制器订阅合同仍需运行时证据。当前源码不能据此认定已实现map→odom→base连续分工。
