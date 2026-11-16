# 只读复核、身份及存储

本次只读原C1 JSONL和既有工程参考NPZ；不读取新传感器数据，不执行ROS、匹配或滤波。公开聚合只用于结论审核；没有原始授权输入时不能完整逐帧复算。私有原文件、轨迹、完整P/原始配置保留原位，既有C1 manifest仍是输入身份主记录。本轮仅哈希小源码/配置，不哈希bag或完整大结果。

公开路径使用别名：REPOSITORY为现有新加卷研究副本；C1_ROOT为既有BG C1数据根；BE_ROOT为既有BE数据根；RESEARCH_DATA为既有BD research_data；NEW_AUDIT_OUTPUT必须是新加卷不存在的新目录。用户本机完整路径记录在私有STORAGE_LOCAL.md，未将地图、轨迹、外参或设备路径发布到manifest。

为避免覆盖，本轮分析脚本要求输出新目录。以已授权数据路径作为变量，在新加卷选择一个不存在的输出目录；不得用本机home或/tmp：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 analyze_existing.py --root "$C1_ROOT" --out "$NEW_AUDIT_OUTPUT" --be "$BE_ROOT" --data "$RESEARCH_DATA"
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 temporal_existing.py "$C1_ROOT" "$NEW_AUDIT_OUTPUT" "$BE_ROOT" "$RESEARCH_DATA"
PYTHONDONTWRITEBYTECODE=1 python3 validate_saved_analysis.py
```

变量BE_ROOT指已有phase_be_e0_e4目录；RESEARCH_DATA指已有BD inputs/research_data。上述最后一项只检查公开聚合、脚本导入及代数结果，不是定位回放。前两项重算描述统计前需确认空输出位置，不能覆盖已有报告证据。

统计约定：stats的p95为绝对值95分位，mean为有符号均值。FAR debug按接收模拟时间分窗；15D误差按原冻结帧与横向轴。constant_received_delay_run_lengths是缓存复用指纹旁证，不是唯一消息ID融合计数。15D完成状态按顺序与match配对、时间差断言验证；代数重构仅使用保存的已接纳增量。

环境：新加卷NTFS3，审查前约332GiB可用；无T7新增写入，无原始数据复制/删除，无大型缓存。沙箱只读挂载视图与授权写入视图不同，所有新增写入均通过新加卷授权路径。总预算≤20MiB，实际见storage_and_checks.json；预计本轮远小于1MiB。将来任何新运行按提案另报时间/次数/空间预算，不能据剩余空间自动启动。

C1既有失败保留；本轮没有新增算法运行失败。分析过程中一条只读查找用了不存在的文件名，随后定位真实模块；一条未运行的分析草稿语法已在执行前修正。两项不产生定位结果，不构成隐藏重跑。

未做新的定位数值回归、Jacobian数值实验或新精度验证。仅保存记录恒等式/字段合同检查通过不能解释为系统定位成功。
