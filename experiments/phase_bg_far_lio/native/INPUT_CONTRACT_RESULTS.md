# 输入、地图和初始化合同

CONFIRMED：TAM__common固定gitlink及传递子模块按上级commit获取；IMU转换仅复制数值/协方差，没有frame_id→TF旋转、杆臂或重力处理。只将其当传感器适配器会错。

独立研究模块 input_adapter.py：三雷达XYZ应用一次既有静态变换；仅融合、不运动补偿；保留intensity和来源ID；header取最后真实点时刻，点时间重新表达为相对秒。原始head+点时保持不变，绝不统一+50ms或伪造均匀时间。

- 298组（169 normal8＋129 right3，含各40组预热）全部通过；894个原云身份沿用既有SHA清单，不复制点云。
- 全部静态TF与原bag、motion和bindings一致；最大点时往返误差2ns；所有融合header与原native_reference完全一致。
- 14项自动化测试通过：三源缺失/重复、无效时间/字段、大小端、非零旋转/杆臂、重力保留、因果区间、杆臂Jacobian中心差分、缺失协方差和字节日志。此为数值/合同测试，不是精度认证。

IMU在base参考点的比力为 f_B=R_BI f_I−α_B×r_BI−ω_B×(ω_B×r_BI)，ω_B=R_BI ω_I，α仅用当前/前一个陀螺样本。保留重力，不再次扣除g；首个缺前值样本跳过，不能伪造零角加速度。六条原bag消息与历史motion记录omega/force逐项差为0；同一非单位TF只使用一次。

normal8/right3因果差分杆臂修正P95分别4.698/4.885m/s²，max9.916/7.765m/s²。差分噪声、振动/非刚体安装和加计/陀螺交叉相关未统计标定，不能宣称物理校准已认证。原gyro/force covariance[0]为−1，原始消息元数据标记不可用：研究模块不将其旋转成负矩阵，也不伪造有效协方差。ROS输入保留原不可用标记；官方单IMU路径对协方差clamp后归一化，因此该未知权重不改变单源名义控制值，但其忽略不可用标记的语义是保留的集成风险，不是统计置信度证据。Q保持原生配置。

地图：原0.8m Target已确认SHA 0895ad26af296aeba68b55350f7b9b70cd6b397be551fc78af75794410192968，908,013点。仅序列化为XYZ PCD，10,896,299字节，逐点完全一致；PCD SHA 836fed31eede50ecab902a2ae58a26ab05383bd94e45315342db3df6638b870b。已有4m占用体素41,982。没有测试帧建图、地图重采样或全局旋转/平移。PCL警告缺intensity，但FAR该路径只取XYZ，两个节点均加载908,013点并完成初始化；没有填入伪造几何。观测到GPU进程约1540MiB，并非严格峰值；主机完整HWM UNKNOWN。没有直接加载67M原始PCD。

初始化：仅每窗口首个预热扫描一次，使用保存的15D **pre-NDT prior**；核对p_base=p_IMU−Rr与旧seed完全一致。该prior的时间由旧运行器强制等于该scan参考时刻。后续仅自身EKF TF反馈，无每帧GT/历史轨迹seed。两个窗口独立初始化，不代表全圈连续运行；初始化祖先是否含GNSS UNKNOWN，不称无GNSS初始化。地图数值坐标合同成立，部署ENU/地理原点及独立垂向基准未认证。

运行图：独立ROS域，发布白名单仅融合云、适配IMU、clock和必要静态TF。原bag动态map→base/其他定位/VectorNav/GNSS均未发布。StateEstimation实际订阅仅上述IMU、LiDAR及其status、clock、参数事件和原生NH wheel空话题；没有提供wheel，也没有伪造速度。官方内部reference_angle_measurement=true保留，是同一IMU/速度派生倾角，不是独立AHRS/C融合。优化器、Q/R、Bias设置和enable项逐项与官方配置相同。

CONFIRMED：两窗口运行内map/num_voxel均保持41,982，map/points_added、points_removed和node/map_update均为0，未在线改图。但内部保留点数分别585,085与584,980，相差105；同一PCD输入不等于两次GPU体素插入产生逐点完全相同的内部表示。并行选点/顺序是HYPOTHESIS，原因未经新实验确认；不可称bitwise确定性地图复现。

进一步沿StateEstimation::set_input_acceleration/angular_velocity核查：其3D分支是输入异常检查、赋值和原生FIR，没有再次TF旋转或杆臂平移；未发现重复参考点转换。IMU轴/时间的接口一致性成立不等于设备物理标定、安装刚性和时延已独立认证。
