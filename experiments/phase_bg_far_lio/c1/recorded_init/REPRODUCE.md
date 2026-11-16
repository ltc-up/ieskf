# 复现与身份

本次实际基线799064772cf1af282e36ee789e45932d0e2ecdb3，独立研究分支；代码没有修改生产或滤波/NDT/GICP数学实现。完整原始结果/时间身份/源配置与SHA在用户BG/c1私有目录；公开只有方法代码与非重构聚合，复现需要获授权的既有loop44、map0105及本地原生服务，不包含其载荷。

停止状态：两套实际连续运行各一次已完成。本文件给复现说明，**不是自动执行下一次运行的授权**。不得在现有目录重跑覆盖；所有运行器使用mkdir(exist_ok=False)保护正式输出。重做定位需后续批准并用新输出根。

## 环境

- ROS2 Jazzy，既有FAR native install，官方源码ef20510316ad1f8dbf2e428e4a193f700493eccd；全部依赖复用，无下载/镜像。
- shell先source ROS与FAR install；如需要repo/src，追加到已有PYTHONPATH，不能覆盖ROS路径。
- TMPDIR、XDG_CACHE_HOME、CUDA_CACHE_PATH、ROS_LOG_DIR全部指向批准磁盘，PYTHONDONTWRITEBYTECODE=1、OPENBLAS_NUM_THREADS=1。
- 使用隔离ROS domain：预检177、FAR178、15D179，localhost only/cyclonedds。只发布白名单三雷达融合、适配IMU、静态外参、clock及一次性授权均值；FAR测量relay之后只转发自身GICP。

## 实际执行顺序（所有脚本在上级c1目录）

1. `prepare_common_init.py --root ROOT --bag BAG --motion MOTION --extrinsics EXTRINSICS`：固定135s，只取一个同时满足header/recorder因果性的有效pose；冻结均值、参考点、原消息cov及因果静止统计，cov不进入新P。
2. `continuous_inputs.py --root ROOT --bag BAG --audit AUDIT --extrinsics EXTRINSICS`：全段身份/三雷达同步/IMU合同，无ROS定位。
3. `realtime_contract.py`同上参数：10秒无定位ROS发布与回执、独立clock、异步日志。全段内容检查与10秒调度检查性质不同，均需通过。逐点时间相对扫描末时刻，误差上界按原适配器检查。
4. `run_continuous_far.py`同上并加`--source FAR_SOURCE`：4秒图发现/地图准备后按真实recorder节奏交付130～345.806s，135s一次共享均值和零base速度；135s前只暖IMU，无LiDAR定位。等待原生EKF成功，不把前端成功代替状态成功。
5. FAR结束后，`run_continuous_15d.py`同上加`--paths BE_PATHS --target-sha 0895ad26af296aeba68b55350f7b9b70cd6b397be551fc78af75794410192968`。使用原研究P/Q/R/Bias/NDT配置、原500ms回放，异步matcher期间持续原输入传播；只维护当前scratch，不产生逐帧点云档案。不能并行运行两套来宣称本次性能等价。
6. 两套结束后`analyze_continuous.py --root ROOT --be BE --data DATA`；`supplemental_audit.py ROOT BE DATA`仅离线读取工程参考和历史记录。0/75/150ms全部保留，原1392/1393及评分索引不改。
7. `python3 -m pytest -q -p no:cacheprovider --basetemp ROOT/unit_test_tmp experiments/phase_bg_far_lio/c1/test_continuous_contract.py experiments/phase_bg_far_lio/test_input_adapter.py`：18项合成合同测试通过，不是定位精度测试。

匹配参数DIRECT7、GLL3、q50、step .2、transformation epsilon .01、relative objective epsilon .001、max iterations35沿用原研究路径。地图与静态外参不改。Native协方差/A/C/GMF不在本实验启用范围；原15D BE Legacy映射保留，无Fitness OFF替代。

## 证据保存

私有：recorded_common_init、timing_contract、realtime_contract、far_recorded_init、eskf15_recorded_init、continuous_analysis、artifact_sha256.json。旧GlobalInit尝试、15D环境启动失败、离线序列化失败单独保留，不伪称没有失败。

全段输入身份摘要、参数/二进制/研究源码身份与新产物SHA可复核。原70GB bag/原地图没有复制或重哈希。公开metrics仅是派生指标，不等同于保留私有完整P/轨迹，也不认证独立绝对真值。
