# 复现与空间约束

本轮五次匹配已执行，下面命令仅为记录；完成后不得自动再次执行。

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 python3 experiments/phase_bf_public_dataset_validation/p2/run_five.py --root "$BF_ROOT" --bd "$BD_ROOT" --out "$NEW_PRIVATE_RUN"
```

需要另行批准才能重新执行。BF_ROOT为已批准工作卷BF根，BD_ROOT为原研究Matcher根；本轮没有新数据/点云/库副本。函数硬编码帧60/64/65/66/76，输入哈希/配置和Source字节不一致则在匹配前停止；等价性失败则停止后续调用，无自动重试。代码没有导入15D滤波或回放。

仅既有trace分析（不匹配）：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 experiments/phase_bf_public_dataset_validation/p2/analyze_five.py --run "$PRIVATE_RUN" --root "$BF_ROOT" --out "$NEW_PRIVATE_ANALYSIS"
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 experiments/phase_bf_public_dataset_validation/p2/test_contracts.py
```

输入原身份、编译命令、新外层哈希、loaded-library映射、5请求/响应及等价性表在本地five_01。完整位姿、Newton/梯度重建、H谱、每步条件在analysis_02；analysis_01是保留的首次分析失败前缀。公开RESULTS/ITERATION_METRICS只有允许的去标识诊断数值，完整复算仍需授权私有输入及原研究库/头文件；不重新分发地图/原轨迹/原生产源码。

新增总量限制10MB，实际约2MB以内，包括小型外层二进制、119条trace、失败前缀和分析图；完整原库29MB为既有只读依赖，不复制也不计作新结果。公共仓库只保存小脚本、报告、诊断图及标量聚合。TMPDIR指向新工作目录，未将编译临时产物、点云或缓存放home/T7。精确最终大小及存储检查见本地P2_AUDIT.json。
