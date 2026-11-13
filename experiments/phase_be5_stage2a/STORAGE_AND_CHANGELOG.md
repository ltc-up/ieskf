# 代码与存储边界

研究基线30de6fa，继续既有研究分支，不创建仓库/分支副本。

修改：pose_uncertainty.py:native_pose_R（解耦Fitness、全矩阵frame/图验证）。
新增：vertical_velocity.py（A1/A2纯模型）、attitude_reference.py（S²/来源诊断）、shadow_candidate.py（纯候选）、参数化测试、冻结合同/诊断/统计脚本及报告。未改变InertialPredictor、Wheel、ExternalPose、ManagedReplay或生产策略。

所有真实state/P/R及输入/TF均不发布；实际55案例私有保存，公开仅去标识汇总/合成测试。完整源码Diff在Git及私有code_diff.patch中，无生产源码复制。

新增数据约12MB量级（包括保留的初版和时间合同规范案例），具体逻辑/分配字节见私有storage_verification.json；远低于500MB。复用所有旧原始输入，生成原生点云0、复制bag/地图0、全量SHA重算0。只对新小结果、源码及七个相关原BE结果做身份检查，不重新扫描/哈希整个历史数据集。

七个相关BE原frames、timing、summary、metrics、CSV、ledger/source_version与原历史SHA一致；30个受保护审查源码未变，仅批准的Native助手改变。原Q/Bias/域参数与基线配置逐值一致，生产HEAD/status/stash保持；不会用研究结果替代生产性能。

结果只保存到指定卷独立Stage2A目录；各路径见本地STORAGE_PATHS.md。所有初失败、阻塞、旧54及补充1批均保留，不覆盖旧结果。Stage2A后停止，下一阶段另行批准。

计量快照：逻辑11744716字节（11.745MB），分配11960320字节；另保留5MB最终Git/元数据裕量，总上界16.745MB<500MB。此为新Stage产物计量，不包含既有大型历史数据。
