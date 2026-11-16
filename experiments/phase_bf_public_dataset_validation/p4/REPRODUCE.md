# 复现记录与证据限制

批准基线3c1c45e7f3accb7a29008642c192b84d88e2675d。原P3二进制SHA7baf2be005c00db07d1963a21fc16c4b9d14c2a89ef18a1ff34dce8ea38adcff、共享库0c229305a733c467a5a63019489768f4ea0b626f449cff23c56b707ccb0ed54c；全部P2请求、Source/Target身份保持原样。

已执行记录，不能自动再次运行：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 python3 experiments/phase_bf_public_dataset_validation/p4/run_ablation.py --root "$BF_ROOT" --bd "$BD_ROOT" --out "$NEW_PRIMARY"
# 原首次触发遗漏FULL，记录保留；获授权范围内补原预置两种子：
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 python3 experiments/phase_bf_public_dataset_validation/p4/run_ablation.py --root "$BF_ROOT" --bd "$BD_ROOT" --out "$NEW_STABILITY" --stage-b-supplement "$PRIMARY"
```

实际primary32+supplement40=72，不再执行任何匹配。原运行器快照及trigger=false记录保留；新决策纳入FULL后触发，未读参考决定分支。采样器与随机种子未调优。

仅分析已保存结果与测试：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 experiments/phase_bf_public_dataset_validation/p4/test_sampling.py
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 experiments/phase_bf_public_dataset_validation/p4/analyze.py --root "$BF_ROOT" --out "$NEW_ANALYSIS"
```

完整private ablation_01/stability_01保留请求、结果、Native矩阵、原文件身份、采样索引npz、临时输入SHA、timing及失败/决策记录。临时bin会被后续组覆盖，因此复现某组必须从原Source按保存索引重建并核对SHA，不能直接重放指向current_source.bin的旧请求。FULL永远指向原文件，不从采样缓存恢复。

6项采样测试通过；15份空间排序及严格嵌套性、70份采样子集字节SHA、72次Matcher实际点数、FULL64/66原结果等价性、前后原输入SHA全部核验。无新PCD、地图、下载或15D更新。新增结果低于50MB；测试/预算/提交状态见本地P4_AUDIT。图只展示基础种子，完整3种子结果仍保留。
