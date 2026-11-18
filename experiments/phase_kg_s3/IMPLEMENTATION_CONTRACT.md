# KG-S3 最终实施合同（2026-11-17）

权威顺序：用户本轮《技术方案审查意见与实施批准条件》>已批准PHASE_KG_S3_TECHNICAL_DESIGN_REVIEW.md>附件《双子代理并行研发与总代理独立验收》。附件中的A/B编号按已批准设计统一为A0 Legacy、A1方向性R、A2 Score MAP；B-P纯IMU/Wheel、B0上一帧、B-M滚动、B-K关键帧。附件的“残差”不能被解释为允许Native最佳体素目标替换A2。研究实现与Shadow完整验证已批准，生产替换和跨线融合未批准。

## A：冻结算法

原CUDA NDT实际多邻域高斯Score、DIRECT7、q50对应、解析g/H及GLL3机制不变；分辨率1.5、原Align步长0.2、max35、步长ε .01、相对目标ε .001。先静态/数值核对Score符号、尺度、邻域求和、RxRyRz图、导数一致性；不一致必须报告提问，不静默修模型。Score温度/尺度冻结，不按评价参考或每帧Fitness调节。

A2目标F(x)=1/2 (x⊟x-)ᵀ(P-)⁻¹(x⊟x-)−βS(T(x))，15D Prior为LiDAR更新前完整mean/P。几何只直接作用位姿，完整P交叉项影响速度/Bias。原Align仅作未融合初值。必须保存原Prior候选、原Align候选、分别从两者出发在同一Prior/P/β下的MAP精炼结果；按同一总目标及事前冻结有效性规则决策，参考不参与。GLL试探检验含Prior总目标；Prior每轮不更新，不先Pose融合再几何更新。原Align+双起点精炼全成本计20Hz周期。禁止擅改0.2、尺度、关联。

A1为最终Pose+完整6×6几何R一次更新；保留交叉项与正确图/杆臂映射，不用Fitness决定新R，不用Prior总H生成R再融Prior。A2直接原Score MAP，不冒称A1。旧Native最佳体素白化残差和Raw协方差仅独立对照。

区分MAP mean、条件几何信息、统计标定P。不得将LM阻尼H逆当可信P。允许预先冻结并明确标记的条件协方差模型用于研究Shadow自身反馈，保存fullP/交叉项/状态增量/异常；统计未校准不得声称生产一致性。数值阻尼与观测信息分开。β、条件P模型的数学定义须在首次实数据实验前冻结并接受总代理审核；若设计尚无法确定或需更换核心模型，先向用户提问。

500ms回滚保留mean/P/Bias/held-IMU/唯一观测账本和Source/Target/先验版本。相同扫描只提交一个LiDAR事务；历史重新线性化从未包含该观测的checkpoint开始。C5实际LegacyComparisonReplay没有Managed终态账本，不混称。100Hz线程只读已提交快照，预测留在事件核，不等待GPU，不改旧状态时间戳。

## B：冻结算法

同一不可变当前Source进入全局与局部匹配；原三雷达融合/Deskew/Source矩形/0.8m/5000点保持。历史Target保存原Deskew后、Source矩形裁剪前真实融合云；不套Source矩形或5000上限。在O系按当前更新前因果预测位置执行120m圆、相对z[-10,30]m，保留正确倾斜轴变换，0.8m采样；记录frame/point IDs/Deskew历史/位姿与Target版本。Target不能使用当前最终global NDT结果。

B0=上一帧真实扫描Target+独立IMU/Wheel预测U+连续Pose链O。ΔU由U相邻参考时刻状态差给出；O预测=上一O·ΔU，成功累计局部Z，失败累计ΔU并标退化。失败后仍用真实上一帧（带预测锚标志），不得偷偷改last accepted Target。局部结果主要更新Pose链，不声称LiDAR对v/Bias/fullP联合估计；U不接global/A/B LiDAR后验。纯预测是独立对照。公共Source保留历史地图Deskew相关性（用户已确认），需记录/评价；不得称完全独立输入。

B须完成B0之外的滚动多帧Target、overlap关键帧管理对照、GLIM表面验证/多分辨率审查及可行实现，保留各自方案名称/历史年龄/统计相关性。CUDA NDT优先；若稀疏性或接口迫使更换目标、Target定义或局部状态，暂停受影响模块提问，不擅换GICP。GLIM式对照不冒充B0，A/B先不融合。

## 公共运行合同

完整130–525s输入前史，135s一次合法录制均值初始化，P/Bias原配置，禁止用成熟存档后验维持新闭环。0105与1013显式地图身份，仅换地图，原raw input IDs/可用时间/外参一致。每个自身反馈方案的后续Seed/Deskew来自自己状态，不能把冻结前端叫闭环。B共用选定A0对照的相同Source，其局部U/O不受global后验重置；A2 Source另条件不得混统计。

713s若有完整合法前史则从合法起点单独连续执行；无法恢复立即提问，不用旧五帧结果替代。覆盖normal8/right3、直道高速弯道、517–519故障、正确大修正、错误盆地/持续/恢复、BB异常。未自然复现旧错误须如实报告，不按参考造错误或挑结果。

区分LiDAR header/point time/Deskew ref、IMU/Wheel测量与实际可用、完成、状态、发布时刻。GNSS/INS仅离线工程参考，报告0/75/150ms及杆臂/轴/地理基准限制，不用它选参数、分支或关联。

统一输出XY RMSE、纵横向/Yaw、P95/P99/max、错误分支持续/恢复、P及状态健康、支持/退化、50ms完整周期P50/P95/P99/max、CPU/GPU/RSS/VRAM、100Hz状态年龄/连续性。保留所有超时/失败/异常候选及原始完整P，不以SPD/收敛/P缩小宣告正确。A/B先各自隔离性能，GPU独占锁分时，不共享同一源码工作区。

## 存储、安全与组织

持久实验根只用/authorized/research_archive/phase_kg_s3/{track_A_geometry_fusion,track_B_local_lidar_odometry,shared_evaluation,reports}。总新增数据≤5GiB（A/B各2GiB+公共1GiB），预取128MiB、日志1024项、RSS≤8GiB/实验、VRAM暂6GiB，超过先提问。不复制原bag/地图，不删历史，不自动回退系统盘存实验大数据。归档只读期间仅在会话目录保留轻量源码/测试/合同，不跑持久实数据回放；后续恢复RW再归档。

研究基线d941720d015dd0826dd6f3c87183b773df7c13db，生产aa50e354…只读。研究分支codex/kg-s3-a、codex/kg-s3-b独立clone（只读卷期间不能写worktree注册元数据）。源码原始生产副本仅在private ignored目录作研究构建，不公开提交。任何core变更不得发生在生产仓库。研究公开代码/脱敏报告可提交研究分支；不推main/不公开原始受限材料，用户范围高于旧AGENTS发布惯例。

总代理协调合同、输入/时间、GPU队列、统一评价，并独立审源码与实际输出，不接受代理自报PASS。A交付六报告：A_IMPLEMENTATION/A_GEOMETRIC_R/A_ITERATED_15D/A_CONTINUOUS_RESULTS/A_WRONG_BASIN_ANALYSIS/A_REALTIME；B交付B_IMPLEMENTATION_AUDIT/B_SOURCE_TARGET_CONTRACT/B_LOCAL_ODOMETRY/B_GLIM_COMPARISON/B_CONTINUOUS_RESULTS/B_REALTIME。总交付A_B_CONTRACT_AUDIT/A_B_COMPARATIVE_RESULTS/NEXT_FUSION_ARCHITECTURE/FINAL_CONCLUSION/REPRODUCE/STORAGE_AND_GIT_AUDIT，分别给A/B/未来联合GO/PARTIAL/NO-GO。

执行前核验HEAD/dirty、挂载RW/容量、原bag/map身份、CUDA与二进制依赖、时间历史；普通开发/数值测试不反复申请，影响算法定义/因果合法性/核心状态/高风险系统/版本冲突/资源范围时提问。GPU可在沙箱外正常访问，无驱动重装需求；当前findmnt仍ro，已请求用户确认恢复RW。没有得到许可前不force挂载/修复文件系统。

## 用户追加批准：A2研究Hessian窄修

仅允许A2独立研究评估接口将pitch–pitch二阶项中的+sin(pitch)z改为−sin(pitch)z，生产CUDA、原Align、A0和Legacy保持原样。Score/多邻域DIRECT7/梯度/GLL及0.2不变；须用多个非零姿态、多个pitch、不同点云及固定关联对全部21个独立H元素作梯度有限差分。保存原/修Score、g、H、求解方向、候选位姿同输入对照。发现其它实质H误差、修正改变Score/g或无法隔离原Align，立即暂停相关修改并提问，不扩大修复。H数值通过不代表统计信息或可信R/P。通过后继续完整研究，无需重批KG-S3。

## 恢复后的环境事实

主机新加卷[AUTHORIZED_VOLUME_DEVICE] UUID[LOCAL_VOLUME_ID_REDACTED]正常rw，sandbox对未列入writable_roots目录显示ro；本任务通过经审查的require_escalated执行归档写入/CUDA，不再误称磁盘未恢复。无需重装驱动；RTX5070Ti Laptop 12227MiB正常。原研究库d941720工作区干净。环境清单已核验1013缓存997680点、0105缓存908013点，bag recorder覆盖713s，后者仍需完整合法前史运行，不能仅覆盖metadata即称验证完成。

## 用户追加批准：β与小角度数值域
A2唯一固定β=1，原Score sum不按Fitness/map/参考/场景缩放，不做β搜索。记录原S/valid pairs/Prior二次型/两项F贡献/几何Prior梯度及方向/四候选/fullP统计标记；几何主导或错盆地也不临时改β。
原角吸附与g保持；|任一Euler|<1e-4或线搜索跨吸附边界的A2点/路径标NUMERICAL_DOMAIN_INVALID，不用其g/H。可继续其它合法候选，无合法几何更新保留纯15D预测，无Legacy回退。按轴、帧比例、连续时长/state变化、0105/1013汇总，域外帧纳入全段统计。频繁触发则报架构限制，连续化另批。与pitchH错误分开归档。
