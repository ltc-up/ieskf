# A2RL输入与地图合同

CONFIRMED：原bag只读SQLite检查一个1秒窗口，没有启动ROS图或回放。原bag约70.76GB，复用历史身份，不重新全量哈希。三原始PointCloud2话题均在，选定bag不含`/sensor/lidar_fusion/points`；不能以单雷达替代融合。`cloud_registered`等不是合适原始输入。

|检查|现有证据|结论|
|---|---|---|
|点字段|XYZ、FLOAT32 timestamp、intensity；20字节/点|CONFIRMED：时间是秒偏移，不是绝对秒或整数ns|
|频率/点时|约20Hz；抽样时间约−25至+20ms|CONFIRMED仅此短样本，不认证全包同步|
|Bosch|1秒198条，dt中位约5.009ms，比力模中位约9.760m/s²|CONFIRMED带重力比力，非已扣重力加速度；不能仅此判断高速轴符号|
|Kistler IMU|246条，dt中位约4.007ms，比力模约9.824m/s²|CONFIRMED：不是Kistler光学速度，姿态标记不可用；不无故双IMU重复融合|
|到达延迟|原始云首批约75–82ms，IMU约亚毫秒|CONFIRMED窗口值；后续须区分header、点时和真实可用时刻|
|TF|bag动态TF含map→base_link、map→car_link|CONFIRMED：回放这些将污染FAR外部初值，必须阻断|

## 仅关闭上游deskew为什么不够

CONFIRMED：只读生产副本`lidar_fusion_undistorition`中common.h定义LidarPointType=pcl::PointXYZI；processLidarDataNoUndistortion读原云后静态变换并拼接，丢timestamp。LidarSyncCallback两条路径又受undistort_to_frame_end影响，可能仍+50ms。此源码副本部署版本UNKNOWN，不修改它。现有历史Deskew输出也不得再次由FAR重复deskew。

HYPOTHESIS：最小研究适配器应对三原始云仅应用现有静态外参、保留点身份/真实时间、不运动补偿，统一为base坐标PointCloud2。若融合header为h，则每点offset=(h_i−h)/1e9+原offset；不得给不同雷达统一伪造时间。应逐点验证h+offset与原点绝对时间一致、三雷达完整保留、只旋转一次。这里只提出合同，未编写/运行融合器。

## IMU、TF、参考点

CONFIRMED：既有15D链路直接适配Bosch，并通过非单位静态旋转到body，状态在IMU原点。FAR ROS Imu回调直接调用类型转换并送加速度/角速度，未在该回调见按frame_id查询TF。TAM__common未初始化，类型转换内部是否含特殊处理UNKNOWN，完成依赖只读核验前不可直接remap原IMU。须核实base/IMU原点的旋转、角加速度/向心杆臂项、重力约定；只转换一次。

仅白名单发布必要LiDAR/IMU静态外参，不能照搬完整tf_static。隔离ROS domain，禁止bag /tf、GNSS、VectorNav姿态、原定位pose及参考轨迹进入FAR图；自身EKF独占反馈TF。旧GNSS converter ant1到后轴补偿存在已知未修问题；离线参考不是后轴绝对真值，保留0/75/150ms及杆臂不确定性说明。

## 地图

CONFIRMED（历史身份）：map_0105_voxel_fix.pcd，67,307,313点，原SHA `8561cd2254ed35164134a3146e89dfcd403643193ff11c84db93fde2b77ad52a`。本轮仅读头，1,076,921,104字节、XYZ+scalar_intensity。Phase AR的0.8m VoxelGrid导出908,013 XYZ，目标SHA `0895ad26af296aeba68b55350f7b9b70cd6b397be551fc78af75794410192968`；导出不含刚体变换。不能误用map_1013enu或另一个normal地图。

FAR直接加载该PCD XYZ原则可行；intensity非核心几何，但字段不匹配须记录。67M点全量CPU/GPU加载的峰值内存及体素容量UNKNOWN，禁止直接运行撞内存。若需用原NDT已有0.8m目标生成PCD，属于未来独立适配，需保留逐点身份与只格式转换合同，不许改地图几何或测试帧建图。

UNKNOWN：当前部署地图ENU原点、生产PCM变换与FAR初始姿态是否完全一致。历史离线数值地图身份不等于部署地理合同。不能凭PCD VIEWPOINT=identity假定ENU，也无需凭空生成origin.yml。
