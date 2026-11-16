# 复现与证据边界

输入只读复用。私有根保存config.frozen.json（事前）、paths.json、preflight_map_geometry.json、timing_contract、realtime_contract、logs及eskf15_1013。精确epoch、原始消息ID/外参/轨迹/P不公开。提供这些输入必须具有原始数据授权。

1. 基线3ab45456，按input_contract.py核验identity和10s realtime。确认RW挂载与资源上限，禁止自动回退系统盘。
2. 原唯一运行命令：source ROS Jazzy环境，PYTHONPATH包含repo/src，PYTHONDONTWRITEBYTECODE=1、OPENBLAS_NUM_THREADS=1，TMPDIR/cache/ROS_LOG_DIR全部设置到私有研究盘；运行 `python3 experiments/phase_bg_far_lio/c5/run_continuous.py PRIVATE_ROOT`。运行器拒绝覆盖已有eskf15_1013。此步骤为归档说明，未经新授权不要重新执行。
3. 离线分析：`python3 experiments/phase_bg_far_lio/c5/analyze_completed.py PRIVATE_ROOT`。要求私有analysis_inputs.json明确historical_0105路径、原地图工程CSV、C1参考npz及tangent。输出analysis，拒绝覆盖；先前已有分析结果直接复核，不自动重跑。
4. 测试：pytest -q -p no:cacheprovider experiments/phase_bg_far_lio/c5/test_trace_equivalence.py experiments/phase_bg_far_lio/c4/test_output_contract.py；原33项完整命令及输出在私有logs/pre_run_tests.log。末段checkpoint仅可加载自己可信pickle。

Source/Target/Seed逐帧以身份/生成配方保留，没有逐帧大型点云，需从原bag及合法前史重新生成才能完全前端复算；保存统计/P可直接审查。FAR真实完整P不可从现有边际量恢复，不能复现不存在的真实A/B。
