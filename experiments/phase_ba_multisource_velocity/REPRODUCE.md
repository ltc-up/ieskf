# BA复现

需要获得授权的本地loop44旧eav24原始包、map0105、私有传感器绑定、原始三LiDAR及已有CUDA运行服务。公开聚合统计不能重构这些输入。Python、NumPy、现有ROS2提取环境和CUDA服务版本沿用AZ/AR；没有新外部数据集。不要修改原数据或复用已有输出目录。

在本研究worktree根目录设置 `PRIVATE` 为独立持久输出目录，`RAW_OPTICAL` 为AY已有包含raw_topics及events的光学原始提取JSON。完整输入SHA见manifest。data链接必须由本地授权资产提供，包含AR的bindingslap、AS的bindings_multilap、loop44运动JSON、frozen配置和CUDA服务。不能用历史最终NDT位姿代替真实服务。

```bash
export PYTHONPATH=src
export OPENBLAS_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1
export IESKF_CONFIG=data/history/phase_v/config/frozen.json
python3 -m unittest discover -s tests
python3 experiments/phase_ba_multisource_velocity/prepare.py --input "$RAW_OPTICAL" --output "$PRIVATE/filtered.json" --audit "$PRIVATE/frontend_audit.json"
python3 experiments/phase_ba_multisource_velocity/frontend_dynamics.py --root "$PRIVATE"
python3 experiments/phase_ba_multisource_velocity/prepare_joint.py --input "$RAW_OPTICAL" --bindings data/phase_ar_loop44_map0105/bindingslap.json --output "$PRIVATE/joint.json" --audit "$PRIVATE/joint_audit.json"
python3 experiments/phase_ba_multisource_velocity/prepare_blocks.py --input "$RAW_OPTICAL" --noise-audit "$PRIVATE/joint_audit.json" --output "$PRIVATE/blocks.json" --audit "$PRIVATE/blocks_audit.json"
python3 experiments/phase_ba_multisource_velocity/launch.py --root "$PRIVATE" --name F_lap --mode F
python3 experiments/phase_ba_multisource_velocity/launch.py --root "$PRIVATE" --name B_lap --mode B
python3 experiments/phase_ba_multisource_velocity/launch.py --root "$PRIVATE" --name B_multilap --mode B --multilap
python3 experiments/phase_ba_multisource_velocity/launch.py --root "$PRIVATE" --name J_lap --mode J --optical "$PRIVATE/joint.json"
python3 experiments/phase_ba_multisource_velocity/launch.py --root "$PRIVATE" --name J_multilap --mode J --optical "$PRIVATE/joint.json" --multilap
python3 experiments/phase_ba_multisource_velocity/launch.py --root "$PRIVATE" --name C_lap --mode F --optical "$PRIVATE/blocks.json"
python3 experiments/phase_ba_multisource_velocity/launch.py --root "$PRIVATE" --name W_lap --mode W
python3 experiments/phase_ba_multisource_velocity/launch.py --root "$PRIVATE" --name B_guard_profile --mode B --profile
python3 experiments/phase_az_velocity_fusion/evaluate_run.py --root "$PRIVATE" --name B_lap
python3 experiments/phase_az_velocity_fusion/evaluate_run.py --root "$PRIVATE" --name B_multilap --multilap
python3 experiments/phase_ay_localization_solution/verify_frozen.py
```

F/B使用同一filtered.json；B仅在自身IMU减速条件采用滤波X。J需要先运行prepare_joint；C需要prepare_blocks；各脚本help给出实际参数。其余每个运行同样执行evaluate_run，三圈加multilap。launch保留逐帧事务、完整P、预测/NDT/后验和stdout/stderr；私有文件不提交。原版/AY/AZ沿用冻结真实运行，仅最终B保护插桩重跑验证正常全部1468帧seed/P/NDT/K/delta最大差0。前端复现输出SHA完全一致。

故障复现先对原始光学提取运行fault_input，再对所得raw_fault.json运行prepare生成filtered_fault.json，随后B模式显式传入该文件。固定首圈偏移20–22s丢失、35–35.5s原始X加15m/s；与真实故障分开报告。未用平滑后的数据伪造原始故障。指定gross异常被原始创新保护拒绝，不代表任意缓慢偏置可检测。

modality_audit、auxiliary_check、radar_feasibility和brake_command_audit需要原始ROS2环境及私有绑定，各自help提供参数。制动指令解码在当前安装消息版本上预期失败：历史92字节载荷与当前schema不匹配，无嵌入定义。保留异常日志，不猜字段。额外IMU和雷达只诊断，不进入候选滤波。

causal_analysis使用固定AS段1/3/5/8/9前3s后.5s范围；aggregate生成公开结果。GNSS仅在evaluate/分析使用，0/75/150ms固定敏感性不进入运行器。完整单圈1468、连续三圈4134；F/J/C40是独立启动排错，不冒充全圈。F/C/W不做三圈的范围和原因见RESULTS；B最终保护三圈未重新执行，正常单圈逐帧一致性不扩大成已执行三圈。

性能：profile为离线实际周期墙钟；原始光学前端在因果约束下提前生成，独立计时，未处于同一在线ROS2 executor。既有运行器历史完成时间不能解释为真实调度截止期。复现者需保留原始实际时钟，不调大缓存或删除慢帧来宣布实时合格。
