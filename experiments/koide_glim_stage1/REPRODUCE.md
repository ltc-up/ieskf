# 复核与复现

公开代码只含独立研究诊断；完整数据需授权。私有根有plan.json（事前12例）、references/identity.json（GLIM/gtsam_points固定commit及文件SHA）、results.jsonl（含初次失败）、be_holdout_qualified/plan及结果、analysis_complete、21项有效及6项无效原15D补充记录、测试/日志。精确epoch、原始消息身份、位姿、全H/P及外参不公开。

1. 挂载检查通过后，在独立新目录运行prepare.py PRIVATE_ROOT EXISTING_ROOT。拒绝覆盖计划。不得复制/重建原地图或用参考选样本。
2. compare.py PRIVATE_ROOT按冻结合同执行；原正常6例holdout合同修正及独立补做计划见qualification_repair.json。复算应在准备阶段即显式标记original_holdout_fit=true，不能隐藏第一次失败。
3. summarize.py PRIVATE_ROOT EXISTING_ROOT只能在比较后运行，使用既有离线GNSS/INS/天线转换，输出analysis_complete；保留有效性缺口，不插补当真值。
4. existing_15d_prediction.py仅复用实际状态/P，20ms锚点年龄保护。713s应UNKNOWN，不能从525s状态作长传播并称短期预测。原错误文件禁止作为有效结果；以existing_15d_prediction_validated.json和验证失败说明为准。
5. pytest -q -p no:cacheprovider experiments/koide_glim_stage1/test_factors.py，basetemp指向研究盘。6项测试包括全部12个联合Jacobian列中心差分、体素协方差均值、规范自由度、负坐标格网、因果运动、过旧/未来检查点拒绝。

PYTHONDONTWRITEBYTECODE=1、OPENBLAS_NUM_THREADS=1，TMPDIR/XDG_CACHE_HOME/CUDA_CACHE_PATH都显式指向研究卷。新目录设置资源上限，不对已有输出自动重跑/覆盖。首次冷启动与稳态分开记录。第三方实现是外部依赖，不拷入公开库。

当前输出足以审查固定案例残差、矩阵、候选及失败；缺失历史帧不能凭报告精确补造，原生GPU/联合定位结果本来未运行。任何第二阶段需重新批准。
