# BE5：最小实施提案（等待用户批准，未改代码）

## 决策

HYPOTHESIS/建议：先批准“冻结快照诊断+Native Raw全矩阵公平对照准备+A模型数值验证”，不立即批准A强融合、C融合或Native-Calibrated生产替换。当前C仅Shadow；原始Q/Bias、匹配器和生产保持不变。B高度约束、GMF扩支、GICP替代、全局重定位停止本轮研究。

CONFIRMED：现有代码已经有右误差注入/重置、ExternalPose全P更新、pose_uncertainty.Native图转换、ManagedReplay延迟账本；不应重写一套滤波器。BE主循环未调用Native转换助手，helper含Fitness隐藏条件。优先暴露真实R和proposal/commit关系，而不是先堆新状态。

## 未来文件/函数清单

| 未来文件/函数 | 拟议最小改动 | 开关/参数 | 风险与测试 |
|---|---|---|---|
|新建geometry_ieskf/vertical_velocity.py|实现A1/A2 h和H，先只返回诊断|a_mode=off/world_normal_shadow/body_normal_shadow；finite sigma/contact source tag|坡/roll/非零r/bg中心差分；不得清零v|
|新建geometry_ieskf/attitude_reference.py|S²/SO(3)合同、来源/半球/残差诊断|c_mode=off/shadow；source_id、time、covariance provenance|只读参考不得重复Bosch/自Deskew；不默认ON|
|既有pose_uncertainty.py:native_pose_R|拆开engineering_R(f)与Native合法性；明确坐标/图；保留M C Mᵀ|r_mode=legacy/native_raw/native_calibrated；scale默认不启用|测试无Fitness仍可有效转换、交叉项和图不符拒绝|
|pose_uncertainty.py:source_floor_R|不用于BE5；不要把其对角化当Native保护|无新增使用|构造强交叉项证明不可误用|
|新建BE5冻结快照driver|只消费旧prior/观测/实际R和身份，输出proposal/commit分离|experiment_id、baseline SHA、限帧数、输出预算|E0读回已有R；不启动Service/匹配|
|ManagedReplay.key/step（仅未来软ON获批后）|为A/C增加唯一ID/时间/生命周期和原始参考payload，并回放|默认OFF；schedule独立冻结|拒绝/过期/重复ID/rollback及P一致|
|ExternalPose.correct|优先复用现有fullR API，不改其生产外延|独立R选择在调用方完成|不复制生产分档；残差图契合|
|新建tests/test_be5_*.py|数学与时序测试、GNSS-off、配置组合与空间限制|不读取全部真实云|数值测试≠精度验证|

所有改动只能在现有研究仓库、用户指定研究环境内实施；本阶段不创建新实验分支/完整副本。实际实现时是否继续现有研究分支须由用户明确方案，不擅自新建分支。原生产EKFConvertor/lidar_ndt/TF/YAML/6D Q/R文件不在改动列表。

## 必须先解决的合同

CONFIRMED：Legacy基线应保存实际BE R，不是重构一个不同的原发布R。Native Raw用已保存native_internal；kappa_at_float_pose另有float重查询，不可混作内部同一次scale。A1/A2的姿态Jacobian不同。C数据源可能切换，缺失逐帧记录不能假定独立。实际TF旋转非单位，body Bias/omega必须同轴。

HYPOTHESIS：只有独立标定条件成立才允许Native Calibrated。第一版最多先做预注册单标量尺度条件研究，不为了最低RMSE调6轴权重。full6D R的frame/参数图转换要先通过，再讨论尺度。原生产ROS发布图风险仅记录，不在本轮修补。

## 测试与可审查交付

未来批准后提交小补丁、独立配置、合成Jacobian/更新/生命周期测试、至多60帧冻结诊断及≤12s短窗口计划。先交E0与各单因素提案、全R/P、增益贡献和失败原因；有证据再申请短窗口软ON和组合，最后才申请全量因果反馈。

验收：模型Jacobian、全矩阵传播、重复/延迟/retraction与GNSS-off隔离均可核验；物理独立性、精度、20Hz另列UNKNOWN/失败，不以测试全绿替代。保留全部负面结果，不增大门限或剪P让结果过关。

## 当前明确NO-GO

C同IMU/未知来源强融合；map-Vz硬零；Roll/Pitch固定零；Native×Fitness；对角化Native；用曲率识别全局分支；用GNSS参考在线选R/门控；在旧错误观测上增加独立NIS来包装可靠性；用A/C掩盖输入轴/时间/重力合同错误；生产6D迁移、优化器/线搜索/Q/Bias调参。
