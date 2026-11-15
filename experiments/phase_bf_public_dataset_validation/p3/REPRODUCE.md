# 已执行命令与复现边界

18次匹配已完成。以下只作为复现记录，**不得据此自动再运行**：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 experiments/phase_bf_public_dataset_validation/p3/run_pairs.py --root "$BF_ROOT" --bd "$BD_ROOT" --out "$NEW_PRIVATE_RUN"
```

BF_ROOT为原指定工作卷BF目录，BD_ROOT为历史Matcher目录，输出必须是该卷新目录。复用P2请求和已验证Source/Target，运行前检查哈希与q50/GLL3/35次等配置；3帧×ON/OFF×3轮，差异即停，无自动重试。原库不改，研究外层默认ON，OFF缺失Fitness为null，无Legacy R。

只统计已有结果或测试，不调用CUDA匹配：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 experiments/phase_bf_public_dataset_validation/p3/analyze_pairs.py --receipt "$PRIVATE_RUN/execution_receipt.json" --out "$NEW_ANALYSIS"
P3_RECEIPT="$PRIVATE_RUN/execution_receipt.json" PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 experiments/phase_bf_public_dataset_validation/p3/test_contracts.py
```

6项测试通过：固定范围、补丁合同拒绝、统计、18次完成与无状态/R提交、OFF缺失语义和18份Native SPD、历史等价性与计时加和。SPD不是尺度正确性证明。运行结果无超容差失败；完整18请求/响应、Native矩阵和身份在私有pairs_01，公共仅保存去标识统计、脚本/配置和结论。无新点云、下载或回放，所有临时编译输出留工作卷，新增MB级；本地P3_AUDIT含实际总量/commit/存储。
