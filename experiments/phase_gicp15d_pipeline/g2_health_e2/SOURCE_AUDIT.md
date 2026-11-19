# G2-E2 源码核对

固定源码版本：FAR-LIO `ef20510316ad1f8dbf2e428e4a193f700493eccd`；Autoware Core `e05daabe1524b57b24d330f39470c77b4d6b6995`。G2-E 报告称 FAR-LIO 公开仓库只有 README，这一判断错误；指定头文件确实在 `lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/` 下。本阶段按这些实际文件纠正。

| 机制 | FAR-LIO / Autoware 源码 | G2-E 当前状态 | G2-E2 处理 |
|---|---|---|---|
| 位置、姿态、速度、IMU 独立有效位 | FAR-LIO `state_machine.hpp` 分 POS、ORIENTATION、VEL、IMU；Autoware 分 pose/twist 缺测计数 | 只有 GICP 位姿状态与速度更新原因散落在日志中 | opt-in 独立 IMU、Wheel vx、Kistler vx/vy 更新年龄、缺测帧数、综合状态；完整 6D GICP 仍为一个位姿来源 |
| 观测门控 | FAR-LIO `ekf.hpp` 有 box/Mahalanobis 残差处理；Autoware pose/twist 分别做门控与延迟界限检查 | 收敛/候选检查、6D SO(3) NIS `HPHᵀ+R`、5 m/5°、历史回滚 | 保留现有 6D 门控；不照搬 Autoware 只用状态协方差 `P_y` 的 gate 或默认门限 |
| 有效位变化后的自适应 R | FAR-LIO `base.hpp` 对位置、姿态分别设置上限并逐步衰减；速度有独立配置/有效位 | G2-E 曾试单帧/长失效后整体 R×4/×2/×2；故障注入的恢复误差反而增加 | 显式 E2 恢复模式保留基础 LiDAR R；独立记录传感器有效性，不按未经校准的常数放大 R。原 G2-E 恢复模式保留作 A/B |
| 纵横向位置不确定性 | Autoware `ekf_localizer.cpp` 从 XY 协方差输出误差椭圆及横向大小 | 15D P 已输出，但未在车体方向诊断 | 在观测 Header 的后验 P 上记录 3σ 主轴、纵向、横向半径 |
| 错误/失效后的恢复 | FAR-LIO 的 ERROR/STALE 可锁存，需显式软件重置；Autoware pose/twist 缺测持续报警 | G2-E pose LOST/RECOVERING 会自动恢复；IMU/速度无独立诊断 | 离线研究保持自动重新接纳通过 6D 门控的观测；不照搬 FAR-LIO 的车辆安全停机锁存，因为本回放没有上层安全控制接口 |

源码： [FAR-LIO base.hpp](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/kalman_filter/base.hpp)、[ekf.hpp](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/kalman_filter/ekf.hpp)、[state_machine.hpp](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/submodules/state_machine.hpp)、[Autoware EKF](https://github.com/autowarefoundation/autoware_core/blob/e05daabe1524b57b24d330f39470c77b4d6b6995/localization/autoware_ekf_localizer/src/ekf_localizer.cpp)。
