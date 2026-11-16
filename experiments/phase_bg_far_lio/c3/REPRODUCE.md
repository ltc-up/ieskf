# BG-C3复核与数据分层

公开preregistration.json固定297例/三个倍率，不据评分选R。r_scale_counterfactual.py读取现有C1与BE5保存记录，只在副本上执行ExternalPose候选计算，禁止调用predict/replay/matcher；所有候选committed=false，原文件只读。candidate_full_matrices.jsonl是私有原始结果，含完整prior/P、R/S/K/H、15维δ、Joseph/reset P、状态候选及原观测身份。未复制任何点云。

实际执行第一次解析失败（0候选）；第二次891候选及身份检查已完成，但最终摘要遇numpy bool序列化失败。完整矩阵保存有效，随后summarize_saved_candidates.py直接读取它恢复汇总，没有重算候选。失败记录analysis_attempts.json保留。已修复通用序列化，未来新输出位置可完整运行，但不能覆盖现有结果。

复核步骤（变量均指已授权既有路径，输出必须在新加卷）：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 validate_candidates.py "$C3_ROOT" "$BE_ROOT"
```

重建汇总（只读已有候选，不重新做Kalman计算）：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 summarize_saved_candidates.py "$C3_ROOT" "$BE_ROOT" "$RESEARCH_DATA"
```

如需重算891候选，必须使用新的空输出目录及相同preregistration，不删除已有文件；本轮没有额外重算。公开聚合没有位置/TF/轨迹，缺私有授权记录时不能完整逐帧复核。历史小型原统计在旧只读归档中仍存在，本轮只读取它们并在新加卷保存新派生，没有把home作为工作或输出目录。

100Hz audit仅按已保存到达时间查找当时已到达快照；未调用预测、未发布ROS、未把后续状态补到过去。首槽缺失保留；state_age与最新IMU差不能当成未记录内核状态的实际延迟。

输入身份复用C1/C2及BE5 manifest，历史小型统计另记录SHA；不重新哈希原bag/大结果。代码SHA及公开/私有体积见manifest和STORAGE_AND_GIT_AUDIT.md。未执行MINIMAL_CLOSED_LOOP_AB_REPORT对应实验，因此该报告文件不存在。数学检查通过不等于精度、统计一致性、100Hz或50ms验收通过。
