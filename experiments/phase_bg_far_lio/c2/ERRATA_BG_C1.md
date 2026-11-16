# C1机制说明勘误（保留原始运行与评分）

本勘误不改写C1轨迹、输入、版本、参数、精度和性能结果；纠正进一步源码追踪发现的机制解释。适用于C1 recorded_init/INITIALIZATION_AUDIT.md及其manifest中相关描述。

1. **一次发布不等于一次融合。**C1应用仅发布一次录制初始均值，且未读取后续录制EKF/GNSS作为在线更新，这一输入事实保持。原文以核心fusion_vec清零解释不重复使用不充分：FAR ROS层measurement.received没有在消费后复位，会每tick重新提交缓存，直到新GICP覆盖；初始化缓存同样可能重复进入。不能继续声称原生只在首次step可能再消费一次。manifest的recorded_mean_used_once只证明应用取用/发布一次，不是统计独立融合次数。
2. **固定Bias应限定范围。**保留原生bias_init=0属实，但低速时IMU Handler持续学习gyro均值，驾驶阶段冻结；加计Bias保留配置初值。不能将整个Pit至驾驶运行描述为全程固定六轴Bias。
3. **非完整约束仅编译/配置存在。**C1没有给FAR发布wheel，不应暗示零vy轮速分支实际参与融合。model/vy debug是未馈入的vehicle model值，不是状态vy。
4. **不要把两类协方差混同。**初始化占位零cov不是GICP输出：4209条实际GICP pose covariance为固定非零对角；其后FAR floor/延迟/状态适配才形成R。

这些发现改变因果解释与公平性判断，不改变原有限条件化定位实验确实完成、FAR精度组合改善及严格全段50ms仍NO-GO的事实。没有偷偷重跑或修复算法。
