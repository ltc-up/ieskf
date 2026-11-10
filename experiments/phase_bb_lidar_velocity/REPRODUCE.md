# BB复现与输入边界

## 条件

父提交BA3606769，AY364d246为光学XY估计器，原版c647ac3/BA最佳B只作冻结对照。使用已有loop44旧eav24、map0105、三LiDAR原始绑定和CUDA Deskew/NDT100服务。没有新数据集、GNSS融合或生产修改。真实物理输入、源云、配置与外参不在公开仓库；必须另外取得授权。

在worktree提供既有授权data链接：AR bindingslap/完整原始Source绑定/Target/runtime、AS bindings_multilap、AO loop44运动JSON、历史frozen配置、AT参考绑定和AW已哈希参考缓存。PRIVATE指定全新的用户授权持久目录；OPTICAL指定AY已有raw optical.json，FILTERED指定BA filtered.json（只离线对照，不进入BB候选）。脚本保留旧输出，不得复用目录覆盖历史。

small_gicp源码来自已有MIT依赖fa0cfc9。实际运行复用AW私有gicp_service，源码与BB scan_pair_service.cpp字节相同。提供可独立重建的build_pair.sh，需C++17/Eigen3/TBB/nlohmann。BB另行构建了新二进制并用9个固定50/100ms真实扫描对验证与原服务位姿/Hessian数值一致，详见聚合结果。新二进制未冒称用于完整三圈。

```bash
export PYTHONPATH=src
export OPENBLAS_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1
export IESKF_CONFIG=data/history/phase_v/config/frozen.json
python3 -m unittest discover -s tests
# SMALL_GICP_INCLUDE指向已有授权upstream/include，输出必须不存在。
export BB_PAIR_BINARY="$PRIVATE/scan_pair_service_rebuilt"
sh experiments/phase_bb_lidar_velocity/build_pair.sh
# launch默认已有AW服务；需要换成重建服务时显式核对launch的BB_MATCHER绑定，不能静默换版本。
python3 experiments/phase_bb_lidar_velocity/diagnose_pairs.py --transactions data/phase_ar_loop44_map0105/fullpose100_lap/transactions.json --meta data/phase_ar_loop44_map0105/fullpose100_lap/deskew_meta_stream.jsonl --matcher data/phase_aw_native_gicp_fusion/private/runtime/gicp_service --out "$PRIVATE/frozen_pairs.jsonl" --limit 1468
python3 experiments/phase_bb_lidar_velocity/analyze_velocity.py --pairs "$PRIVATE/frozen_pairs.jsonl" --optical "$OPTICAL" --out "$PRIVATE/frozen_velocity.json"
python3 experiments/phase_bb_lidar_velocity/peak_checks.py --root "$PRIVATE" --matcher data/phase_aw_native_gicp_fusion/private/runtime/gicp_service
python3 experiments/phase_bb_lidar_velocity/launch.py --root "$PRIVATE" --name M40 --optical "$OPTICAL" --monitor --limit 40
python3 experiments/phase_bb_lidar_velocity/launch.py --root "$PRIVATE" --name P40 --optical "$OPTICAL" --limit 40
python3 experiments/phase_bb_lidar_velocity/launch.py --root "$PRIVATE" --name P_lap --optical "$OPTICAL" --profile
python3 experiments/phase_bb_lidar_velocity/launch.py --root "$PRIVATE" --name P_final_multilap --optical "$OPTICAL" --multilap --profile
python3 experiments/phase_bb_lidar_velocity/launch.py --root "$PRIVATE" --name L_lap --optical "$OPTICAL" --stride 2 --profile
python3 experiments/phase_bb_lidar_velocity/launch.py --root "$PRIVATE" --name P_fault --optical "$OPTICAL" --drop --profile
python3 experiments/phase_bb_lidar_velocity/launch.py --root "$PRIVATE" --name R40 --optical "$OPTICAL" --feedback-aware --limit 40
python3 experiments/phase_bb_lidar_velocity/launch.py --root "$PRIVATE" --name R_lap --optical "$OPTICAL" --feedback-aware --profile
python3 experiments/phase_bb_lidar_velocity/launch.py --root "$PRIVATE" --name R_multilap --optical "$OPTICAL" --feedback-aware --multilap --profile
python3 experiments/phase_az_velocity_fusion/evaluate_run.py --root "$PRIVATE" --name R_lap
python3 experiments/phase_az_velocity_fusion/evaluate_run.py --root "$PRIVATE" --name R_multilap --multilap
python3 experiments/phase_bb_lidar_velocity/source_study.py --root "$PRIVATE" --optical "$OPTICAL" --filtered "$FILTERED"
python3 experiments/phase_ay_localization_solution/verify_frozen.py
```

其余每个完整run同样执行evaluate_run，三圈加multilap；固定1197公共mask与三圈扩展第一圈1204样本严格分开。0/75/150ms沿用原评价，仅离线参考插值，不反向优化时间。P_final_multilap是故障硬化后的三圈；P_multilap早期结果保留。final_prefix_parity记录P_final前1468帧与P单圈seed/P/NDT/K/delta逐项一致。R没有修改Q/R或任何GNSS输入，只有扣除既有速度反馈的hint运输。

源码中私有服务位置只是既有data相对绑定，实际二进制SHA见manifest。每帧Deskew云、scan-to-scan输入/输出、interval twist、源时刻、资格、几何、辅助量、全局NDT、15D/P、时延和stdout/stderr在PRIVATE保留。公开只导出统计、哈希及不可重构图。

指定速度输出不可用注入是首帧参考后[20,22)s，不删除原始全局LiDAR、不伪造速度、不修改NDT。服务故障自动退出另有单元测试；不能把指定注入扩大为任意错误峰检测通过。L只做1468帧低频成本消融，没有L三圈。

聚合：aggregate.py接受root、ay-root、ba-root和out；需所有run评价、fixed velocity、source_study、peak_checks、final_prefix_parity及rebuilt_pairs结果。final_prefix_parity/重建配准对应公开manifest中的私有诊断身份。完整速度时序图写入PRIVATE，公开图仅分段误差、来源差异和时延统计。所有错误和未认证条件见FAILURES。
