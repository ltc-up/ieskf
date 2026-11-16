# BF1结论 — KITTI结束，等待RACECAR审批

- CONDITIONAL：CUDA-NDT可读取适配后的KITTI点云；77LiDAR/846OXTS合同核验完成，后57帧真实CUDA匹配和15D因果反馈完成。前20帧参考辅助地图与测试严格分离，但不是独立地图/真值精度认证。
- CONDITIONAL：完整NativeRaw矩阵转换/SPD及同prior R/K/P可用，57帧交叉项保留；本场景XY参考RMSE .220625→0.196802m，横向P95及Yaw退化。不能认定Native全面更好或替换Legacy。
- UNKNOWN：绝对协方差尺度、完整NEES、跨场景鲁棒性、已收敛错误分支识别、高速banking可靠性。小NIS和小修正不证明正确；Native仍不乘Fitness，Fitness仅对B0构R/B1独立诊断。关闭Fitness的GPU收益未测。
- NO-GO：50ms完整实现周期验收，B0/B1公共到达控制超时27/57和21/57；未执行精确Deskew/ROS控制端到端，不把GPU匹配时间冒充全周期。
- UNKNOWN：A2高速接触假设及垂向真值；本轮仅A残差/H Shadow，参考点是已知Velodyne点而非假定后轴。未见历史多m/s Vz实际跳变，但无独立垂向精度证明。C未融合。
- NO-GO：凭此短场景推进生产配置/新算法或Native标定。保持原Q/Bias、生产6D、TF/YAML和NDT配置不变。

只下载获批三包814833599字节，无历史清理、无其他KITTI/RACECARpayload。工作根现有全部分配约1.077GiB，包含BF0论文及失败准备，低于8GiB增量预算。BF1交付时曾提交[RACECAR历史申请](RACECAR_DOWNLOAD_REQUEST.md)；用户后续明确不批准，现已归档且不再推进。

## BF1只读复盘后的状态

[复盘](review/README.md)维持50ms NO-GO。Native主要改善纵向，约88%的XY误差平方收益累积在拒绝区间，不能作为错误匹配识别证据。用户明确不批准RACECAR下载，旧申请归档，后续高速仅考虑已有A2RL数据。当前停止，不自行执行后续小实验。

## BF1-P2完成后的补充

[五帧证据](p2/README.md)：64/65满足终止但持续向更偏参考位置提高score；66/76没有GLL回退或可证实震荡，直接增加迭代缺乏依据。PCL Fitness墙钟35.47–36.53ms，为本轮align主要成本。仍不构成20Hz或生产资格，未运行Fitness关闭对照，未修改算法/参数，完成5次即停止。

## BF1-P3补充

[三帧ON/OFF](p3/README.md)18次输出与P2保持等价；OFF显式缺失Fitness，不构造Legacy R。稳态align节省约35–37ms，未观测到向尾部同步转移同量级耗时，但纯GPU/等待占比UNKNOWN。不构成完整20Hz通过，也不允许Native替换生产R。完成即停止。

## P4补充

[五帧72次Source消融](p4/README.md)显示采样分布能改变局部优化结果，点数更多不保证正确；空间子集收敛但正常帧可变差。原型排序73–76ms，不能在线移植。保持生产/Native尺度/优化器原状，完成后停止。
