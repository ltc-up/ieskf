# 复现与运行边界

本次实际运行版本以manifest中的baseline、各脚本SHA、binary SHA和私有selection/config为准。公开脚本及聚合不足以还原受限原始点云/轨迹；完整重算需要原授权输入，不能声称公开仓库独立全复现。

所有命令从研究仓库执行，`C4_PRIVATE`必须显式指向已挂载新加卷C4目录；TMPDIR、ROS_LOG_DIR、MPLCONFIGDIR设为该磁盘子目录；`PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1`。禁止自动回退home/tmp。

## 无新真实匹配的复核

```bash
python3 experiments/phase_bg_far_lio/c4/analyze_frozen.py "$C4_PRIVATE" "$AUTHORIZED_OLD_ARCHIVE/analysis"
python3 experiments/phase_bg_far_lio/c4/recover_far_prior.py "$C4_PRIVATE" "$C1_PRIVATE" "$AUTHORIZED_REFERENCE_NPZ" "$AUTHORIZED_EVALUATION_NPZ"
python3 -m unittest discover -s experiments/phase_bg_far_lio/c4 -p test_output_contract.py -v
```

第一条读保存35结果和旧CSV，第二条仅做代数/离线评价，均不运行匹配/滤波。默认路径参数由调用者提供；公开脚本不将工程参考送入在线算法。

## 已执行但不要自动重跑的计算

1. `frozen_diagnostic.py prepare`先冻结选择，再把所需XYZ精确转换到研究matcher内存协议；保存Source/Target/Seed SHA、点数、坐标合同、旧final身份。
2. `run`只运行selection内35条，结果存在则拒绝覆盖；DIRECT7/GLL3/q50/max35等见私有requests。不能把此命令加入自动测试或默认CI，因为会消耗新的真实配准次数。
3. 原生FAR probe：复用已构建Native工作区编译参数，完整argv保存在私有`runtime/native_probe_compile.json`，无需重拉依赖；编译源`native_ekf_probe.cpp`，默认调用原生预测/更新，trace只是复制矩阵。
4. 两次ROS合成：source已有Jazzy，隔离domain217/localhost，运行`test_high_rate_ros.py`并显式给新输出目录；实际最终产物在私有prototype_v2，原prototype保留。没有bag或NDT，也不发布TF。

`analyze_frozen`的L=10是条件XY/world-yaw一阶拉回，不是统计协方差。`recover_far_prior`只允许高于yaw-R floor且FIFO舍入检查通过的时隙；其他UNKNOWN，不用邻近结果填充。

参数化公开脚本还要求私有`input_time_contract.json`，含已核验bag/旧profiling原点；原绝对时间不随公开报告发布。修改前执行源码快照保留，参数化后的分析重算结果一致；没有重新配准。

完整私有目录映射、输入大小、原文件身份、software snapshots、旧结果引用、编译日志、失败尝试及逐事件输出全部保留；未恢复/复制整份bag或全地图。未来全图1013运行需要独立初始化/资源合同，不能由这份有限冻结复现命令隐式启动。
