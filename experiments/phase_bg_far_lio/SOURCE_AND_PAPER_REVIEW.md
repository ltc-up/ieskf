# 源码与论文审查

CONFIRMED：官方及既有缓存commit为 `ef20510316ad1f8dbf2e428e4a193f700493eccd`，缓存工作区干净；TAM__common gitlink为 `4b19b608bdd8b6d6c60747e3fcfa1288399c3186`，本机子模块未初始化。缓存没有重新克隆。

论文：Maximilian Leitenstern、Marcel Weinmann、Patrick Haft、Tobias Lasser、Dominik Kulmer、Markus Lienkamp，2026，*FAR-LIO: Enabling High-Speed Autonomy through Fast, Accurate, and Robust LiDAR-Inertial Odometry*；阅读全文III–IV及消融。作者结果含三雷达拼接赛车数据，不能因一个PointCloud2入口推断其只适用单雷达。[原文](https://arxiv.org/html/2606.26010v1)。

论文III-A：CUDA voxel hashmap、27体素搜索、局部点协方差、sparsity-aware GICP、动态对应距离及自适应局部地图密度。论文III-B：九状态位置/姿态/body速度、IMU控制输入、静止校准Bias、LiDAR位姿及速度、IMU/速度导出的倾角参考；延迟观测向当前状态外推并多周期分摊。它不是本项目15D ESKF，也不是原始输入回滚重传播。作者速度/精度/耗时结论仅是其环境证据，不构成本机先验地图结果。

|问题|已审源码证据与边界|
|---|---|
|先验地图|`node_base.hpp:355–370`在update_map=false时，`.pcd`直接加载；非PCD字符串作为地图service名字。不是文档所说的目录输入；不读取origin.yml。|
|地图坐标|`utils.hpp:255–280`复制PCD XYZ，无地理原点平移/旋转。必须让原地图与EKF初始全局位姿在同一数值坐标。|
|初始化|`tam_odometry.hpp:65–142`读自身EKF TF；默认wait_tf=false允许首次无TF时进入配置初始位姿回退。wait_tf=true加仅LiDAR测量可能形成启动互等，禁止盲用。`set_initial_state()`需要IMU和位置/姿态/速度测量；缺测不得伪造。|
|额外姿态|默认reference_angle_measurement=true；ref_orientation_handler用同一IMU加速度、角速度及状态速度差分产生倾角，含动态补偿及asin限幅。不能称为独立姿态源。|
|GNSS|默认配置仅LiDAR观测+IMU输入，未配置GNSS；通用节点有可配置测量入口，最终运行白名单/有效参数仍需审计。无GNSS并不证明初始pose祖先无GNSS。|
|非完整约束|compose选`ekf_3d_nh`，组件明确为VehicleModel::NonHolonomic。需记录实际wheel/vehicle-model有效条件，不能把NH后缀当普通15D；不能仅凭NH后缀断言没有wheel时仍强制vy=0。|
|点时|utils接受t/time/timestamp：UINT32纳秒偏移、FLOAT32秒偏移、FLOAT64绝对秒。原A2RL FLOAT32秒可兼容；XYZI融合输出不满足自身deskew合同。|
|去畸变|论文描述速度线性回归；当前实现polynom_undistortion使用自身TF姿态历史拟合，并限制点时间到可用历史范围，需测实际clamp/skip；不能用论文描述替代当前实现。|

默认配置仍是在线建图，必须显式改为研究配置update_map=false，禁止偷偷退回在线建图。默认max_iter=500、max_time=200ms、frame_outdated=100ms、max_time_diff=50ms不保证20Hz。不是要求将NDT35迭代套给GICP；先保留原生算法参数并如实报告预算失败。

[固定版本源码](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/ros_packages/tam_odometry/include/tam_odometry/node_base.hpp)、[配置](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/config/far-lio.yml)、[后端](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/state_estimation.hpp)、[官方文档](https://tumftm.github.io/FAR-LIO/)。
