# GNSS可选融合：只沿用生产EKF

文件名沿用交付要求；本轮不设计GNSS完整性算法。FIX8不保证可信，但不新增RAIM/FDE、欺骗检测、独立完整性状态机或自动启用条件。GNSS是否入生产EKF仅由既有use_gps语义控制。离线评价不具有在线授权。

## 已核实的开关和流程

| 环节 | 源码行为 | 研究约束 |
|---|---|---|
| EKF use_gps | declare默认0；0禁用、1启用、2既有AUTO | 不改变值域或语义 |
| AUTO | planner轨迹/区域条件决定既有抑制 | 保留，不能另加FIX/在线触发 |
| 参数/服务 | 参数回调及SetSensorModes可更新use_gps | BE不得自行调用启用 |
| GNSS回调 | finite/协方差范围检查后检查use_gps，再入共同pose队列 | 不绕过入口 |
| converter | GPS1原始LLA→ENU，姿态依attitude_type；非FIX8设置大方差 | 描述既有行为，不当完整性保证 |
| EKF观测 | y=[x,y,yaw]，H选择X/Y/YAW；沿用pose R和平滑/历史更新 | 不是只融合GNSS XY，也不新增紧耦合 |

YAML没有显式use_gps项；默认launch加载该YAML但源码默认0，运行期服务/参数可覆盖。未读取现场ros2 param dump，故不能宣称历史bag实际GNSS关闭。GNSS-off重放必须记录最终参数与动态事件，并从开始即关闭；切换后已在共同队列的旧观测可能继续处理。

## ant1→后轴中心核验结果：未通过

旧EKF实际默认接线：原始GPS1 → ekf_converter::gps1_callback → pose_from_gps → in_pose_with_covariance → callbackPoseWithCovariance → pose_queue → measurementUpdatePose。

converter的position.x/y/z直接赋gps_enu；旧横向/纵向杆臂扣除代码是注释。measurementUpdatePose直接构造XY/yaw观测，frame不符仅告警，没有TF点转换。LLA→ENU只换坐标表达，不换物理参考点。raw驱动用positionGpsLla填GPS1，而不是INS estimated位置；因此不能假设接收机INS杆臂已应用于该GPS1坐标。除非取得设备寄存器/驱动语义的确凿反证，该链按天线观测看待，不能标为base_link后轴中心已完成。

另一条vn_sensor_msgs::pub_gnss_pose路径有lookupTransform(vectornav_ant1,pose_child_frame_id)，将平移经姿态旋转后加到ENU。但所查YAML pose_child_frame_id=vectornav_ant1，意味着自身TF平移为零；而且该话题不是所查EKF默认输入。它将姿态方差设很大，直接remap还会触发现有全对角方差上限拒绝，不能建议“换个话题即可”。base_observer某模式输出侧有ant1到后轴转换，也不能修复EKF入队前的参考点。

Xacro源码注释明确base_link为后轴中心地面投影；现场是否加载该TF、map原点/轴向是否与PCD/NDT一致、姿态是否同步仍需同一部署证据；本轮只确认源码路径，不发布外参值。ENU和map同名不够。GNSS-on比较必须先通过参考点资格检查；当前NO-GO直接宣称有效GNSS-on基准。

## 正确的几何契约（用于审计，不实施新融合）

约定T_AB将B坐标变为A，l_BA是后轴B指向天线A、在B表达：

p_MB = p_MA − R_MB l_BA。

若TF查得t_AB（B原点在A表达），则p_MB=p_MA+R_MA t_AB，不能把两种方向混用。投影原点转换、姿态外参、杆臂各只能执行一次，且使用观测时刻姿态。小角度误差下J=[I, R[l]×]（符号随扰动定义），P_B需包含位置、姿态、杆臂不确定性与交叉项；不擅自改变当前R。检查四种yaw和非零roll/pitch、静止旋转、不重复补偿、时间错位；本轮未执行数值标定或改TF。

## GNSS关闭的数据防火墙

默认实验use_gps=0，保存参数/服务轨迹；候选生成、权重、分支删除/恢复、初始化seed、Q选择、航向/速度不得读离线GNSS标签。global_init另有use_gps_position_prior=true，与EKF开关独立：若用其输出seed，即使use_gps=0也不能称GNSS-free。GNSS-free重放沿用冻结原初值并记录其来源，禁用研究流程中的GNSS候选先验；不修改生产默认。进一步检查INS attitude/速度是否内部GNSS辅助，若无法排除，称“GNSS位置观测关闭”，不声称所有上游信号GNSS独立。

## 恢复与退出

本轮只观察生产现有开关机制下的恢复/退出，不增加自动重接入。断联无新观测，预测继续；恢复观测是否入队仍由原开关及检查决定。use_gps=0时恢复也不得入状态。use_gps=1/2下现有逻辑并不证明跳变安全，必须测量且报告限制；不能以新低通、重置、协方差补丁掩盖问题。若需要修复参考点或恢复机制，另立经授权工程任务。

默认attitude_type=3时converter把最新NDT orientation填入GPS Pose。这使GPS航向与LiDAR明显相关；即便位置参考点修正，也不能把该Pose当独立GNSS航向证据。当前参数语义仍保持不变。
