# REPRODUCE

本报告属于KG-S2有限CPU固定滞后Shadow研究。未修改生产、main、NDT、15D核心、地图或Q/Bias；未运行全圈/三圈。所有精度数值使用既有GNSS/INS及天线到base转换的工程参考，不是独立真值。完整原始输入、状态、P/J/H、轨迹和消息身份留在授权私有卷，公开文件仅含自编源码及脱敏标量。CONFIRMED指所述具体数值/实现事实，不等于定位或统计认证。

## 身份与先决条件

研究baseline d418fa2c5c996ec859e310eb2321d8abfc47fe6c；原生依赖commit见manifest。需要获得授权的私有输入/外参/历史C5/BE身份清单，公开仓库不包含地图、bag、源点云、轨迹、原始P或设备参数。不能仅凭公开报告精确复算全部实数据。

所有输出、TMPDIR、ROS_LOG_DIR和构建缓存指向已挂载研究盘；先检查lsblk/findmnt/df及Git。不能恢复已清理重复点云，也不能下载新数据或变动生产。

1. 只读检查已有环境，使用已安装GTSAM4.2、Eigen/TBB/Boost、ROS Jazzy。gtsam_points CPU配置BUILD_WITH_CUDA=OFF、OpenMP ON，只编译2路，不重建GTSAM。
2. 按私有路径映射运行recover_inputs.py（需原生Deskew服务及ROS类型）。它会核对CDR/状态/Seed/Source身份并在失败时停止；已有inputs应直接复用，禁止覆盖。
3. CMake配置native目录，传入POINTS_SOURCE/POINTS_BUILD；构建kg_s2、kg_diag、kg_tests。运行kg_tests OUTPUT，确认原生数值/边缘化测试。
4. 冻结series.json、0.5s lag、1m voxel、knn20、Huberδ2.795和所有初值。执行run_matrix.py PRIVATE_ROOT NEW_OUTPUT_NAME；不得覆盖原native_verified。713必须使用独立的条件化离线配置及资格说明，不冒充成熟15D连续前史。
5. prepare_diagnostics.py生成同帧原生诊断计划；kg_diag CONFIG PLAN OUTPUT。conditional_R.py仅生成候选，禁止提交。原始拒绝帧缺R应保留原因，不能自动补Legacy R。
6. 优化结束后才能执行evaluate.py ROOT EXISTING_ROOT OUTPUT_NAME；之后aggregate.py生成脱敏统计。GNSS/INS代码不被优化程序导入。
7. 检查P有限/对称/PSD、原生因子缓存深拷贝数值等价、真实迭代预算与状态时间。应用私有TRACE_SCHEMA_ERRATA：旧H_gradient是η，数学梯度为−η；权威P使用显式Pose/V/Bias顺序。

实际运行分三批：初始Batch LM、只读隔离Batch控制、最终原生LMExt。只有native_verified是主结论；其它及两次clone诊断失败保留。源码勘误只修字段语义，原执行源码SHA快照与二进制SHA另存。多线程数值回归容差1e-8，不保证位级重放。

本轮未执行：GPU几何、完整相关性建模/尺度标定、全圈/三圈、真实前端反馈闭环、100Hz控制接入、生产替换。
