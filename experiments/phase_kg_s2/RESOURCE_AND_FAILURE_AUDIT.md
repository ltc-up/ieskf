# RESOURCE AND FAILURE AUDIT

本报告属于KG-S2有限CPU固定滞后Shadow研究。未修改生产、main、NDT、15D核心、地图或Q/Bias；未运行全圈/三圈。所有精度数值使用既有GNSS/INS及天线到base转换的工程参考，不是独立真值。完整原始输入、状态、P/J/H、轨迹和消息身份留在授权私有卷，公开文件仅含自编源码及脱敏标量。CONFIRMED指所述具体数值/实现事实，不等于定位或统计认证。

**CONFIRMED**：原始bag、全局地图和历史结果只读复用。新增原始重建仅100帧必要Source及小控制窗口，没有复制整包/全地图，没有恢复重复点云，没有下载数据集或Docker。CPU依赖复用本机GTSAM，仅下载小型官方gtsam_points源码及Holistic源码元数据。

存储/源码/二进制/输入SHA清单在私有卷。资源计划为新增6GiB、软件下载128MiB、最多2路编译；实际占用及盘剩余量见私有STORAGE_AND_GIT_AUDIT，远低于预算。原生几何CPU不使用VRAM；Deskew阶段GPU精确瞬时峰值UNKNOWN。

保留失败：普通Batch LM缓存cost不兼容；首版诊断缺自定义因子clone；统计JSON的numpy标量序列化错误；7个候选缺实际Legacy R；所有达到LM预算的窗口；S2S及鲁棒方案的负面结果。部分初版脚本异常未持久化独立stdout，但原失败前缀、日志和工具审计仍在；没有将其计作成功定位。

真实100Hz/车辆端到端未执行。713原始到达时序及成熟P缺失，所以其结果仅条件化离线；没有伪造传感器、真值或到达时间。
