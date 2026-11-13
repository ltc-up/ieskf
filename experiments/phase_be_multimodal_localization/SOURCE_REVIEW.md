# 参考源码审查与证据等级

2026-11-13保存官方仓库只读快照。仅审查下述路径，不声称构建/测试过这些参考系统，也没有将其代码复制进生产或本研究算法。首次错误URL及下载失败在私有审计保留。

| 仓库 | 固定commit | 审查路径/发现 |
|---|---|---|
| [TUMFTM/FAR-LIO](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/ros_packages/tam_state_estimation_node/include/tam_state_estimation_node/node.hpp) | `ef20510316ad1f8dbf2e428e4a193f700493eccd` | model_update_callback遍历测量通道、延迟补偿后输入位置/姿态/速度，附带延迟方差缩放；config/far-lio.yml实际输入为LiDAR odom与IMU。通用多通道能力不等于该配置启用GNSS。 |
| [koide3/small_gicp](https://github.com/koide3/small_gicp/blob/fa0cfc983417c044b6036f71dfa448038bb9c820/include/small_gicp/factors/gicp_factor.hpp) | `fa0cfc983417c044b6036f71dfa448038bb9c820` | linearize查询近邻，用两侧协方差和T构造度量，再累积H/b/e；旋转在前平移在后、局部Jacobian。不是位姿统计协方差。 |
| [ethz-asl/COIN-LIO](https://github.com/ethz-asl/COIN-LIO/blob/76729cc4feb3649cbd79d28f82d9f62a2c82889b/src/feature_manager.cpp) | `76729cc4feb3649cbd79d28f82d9f62a2c82889b` | 方向评分通过投影Jacobian和图像patch梯度，为各弱几何方向选择互补feature；不是单一Hessian逆赋R。 |
| [leggedrobotics/perfectlyconstrained](https://github.com/leggedrobotics/perfectlyconstrained/blob/0fbe4175ea205a271f85287abfd8048e5f7dd32a/libpointmatcher/pointmatcher/ICP.cpp) | `0fbe4175ea205a271f85287abfd8048e5f7dd32a` | 含X-ICP类localizability与多类约束/重映射；README明确后续比较实现含X-ICP检测，不把整个仓库称原X-ICP发布版本。部分localizability测试文件为空，不能把文件存在当测试覆盖。 |
| [TUMFTM/3DVehicleDynamicsStateEstimation](https://github.com/TUMFTM/3DVehicleDynamicsStateEstimation/blob/14ba6285d1cb8d0cd847a69af4e1981015d48b8e/README.md) | `14ba6285d1cb8d0cd847a69af4e1981015d48b8e` | 明确点质量EKF+单轨UKF速度架构，与FAR-LIO论文九状态的关联可核实；本轮未编译UKF，不迁移GNSS质量适应。 |
| [superxslam/SuperOdom](https://github.com/superxslam/SuperOdom/blob/f10e65cd50007767b22e4c401689665e20d827d6/super_odometry/src/LidarProcess/LidarSlam.cpp) | `f10e65cd50007767b22e4c401689665e20d827d6` | FeatureObservabilityAnalysis根据PCA法向/主方向、旋转轴叉乘和点积统计feature贡献；具体实现含配置阈值，不采用论文宣传中的绝对免调参作为生产承诺。 |

## 生产证据定位（私有源代码不发布）

以生产HEAD aa50e35467f89b4963f882d1f5abaace46d0ce70及当前工作文件SHA256为准，未改变既有未跟踪目录。

- ekf_localizer/src/ekf_localizer.cpp:73–74 默认use_gps；492–504 开关/现有AUTO/入队；928及954 参数与服务；291–301 五次队列处理。
- ekf_localizer/src/measurement.cpp:poseMeasurementMatrix与twistMeasurementMatrix：XY/yaw与vx/wz；poseMeasurementCovariance：R乘平滑数。
- ekf_localizer/src/state_transition.cpp:predictNextState：仅vx、yaw+yaw_bias，未使用vy。
- ekf_converter/src/ekf_converter.cpp:248–334：GPS1投影/姿态/协方差，317–321杆臂注释；212默认attitude_type=3，此时GPS姿态复用NDT orientation，不能算独立GNSS航向。
- ekf_localizer/launch/ekf_localizer_fly_eagle.launch.xml:8,36–38默认GPS converter话题接线；没有证据认定现场必然与此相同。
- ekf_localizer/src/ekf_module.cpp:measurementUpdatePose 直接读XY/yaw，frame不一致只告警。
- a2rl_vehicle/vectornav/src/vn_sensor_msgs.cc:48–56 TF查找；367–393 投影/旋转平移/姿态大方差；YAML:34设置目标ant1。
- a2rl_vehicle/vectornav/src/vectornav.cc:1035 positionGpsLla与1154 positionEstimatedLla不同。
- a2rl_launch/urdf/a2rl_race.urdf.xacro:7明确base_link为后轴中心地面投影；这是源码定义，现场加载TF仍待核实。
- base_observer/src/base_observer.cpp:341输出侧有参考点转换，不能代替入EKF前转换。

本地私有证据保存完整源快照与文件哈希、磁盘核验、Git状态和文献访问结果。公共报告不发布真实路径/地图/bag/轨迹/外参/参数payload；这不影响用户在本机查阅完整证据。
