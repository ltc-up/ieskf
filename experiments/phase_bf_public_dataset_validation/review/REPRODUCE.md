# 只读复盘复现

运行于既有研究仓库，无需网络、CUDA服务或滤波回放。使用已有NumPy/SciPy环境：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 python3 experiments/phase_bf_public_dataset_validation/review/analyze_existing.py --bf-root "$BF_ROOT" --be-cases "$BE_CASES" --out "$NEW_REVIEW_OUTPUT"
```

BF_ROOT为已有BF存储根；BE_CASES是BE5 Stage2A verified_55/cases.jsonl。输出必须是工作磁盘的新小目录，不覆盖旧结果。脚本只读已存结果/OXTS参考、计算线性投影和K*r，不调用matcher、滤波更新或回放。完整本地输出路径见本次交付；公开仓库不包含原始输入、完整轨迹或生产源码。

实际检查：共同available epochs一致；每帧计时分解和等于完整记录；投影K*r等于保存增量；所读输入运行前后SHA相同。303/47等历史测试不是本次重跑的测试。本次算术检查通过只证明记录及分析一致，不证明真实精度或统计校准。

新增文件包含分析代码、文档、聚合标量及私有小型细节，预期远小于10MB。原结果、地图、点云、配置未改。当前停止，不运行后续提案。
