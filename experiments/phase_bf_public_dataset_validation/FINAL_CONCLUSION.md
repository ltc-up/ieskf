# BF1结论 — KITTI结束，等待RACECAR审批

- CONDITIONAL：CUDA-NDT可读取适配后的KITTI点云；77LiDAR/846OXTS合同核验完成，后57帧真实CUDA匹配和15D因果反馈完成。前20帧参考辅助地图与测试严格分离，但不是独立地图/真值精度认证。
- CONDITIONAL：完整NativeRaw矩阵转换/SPD及同prior R/K/P可用，57帧交叉项保留；本场景XY参考RMSE .220625→0.196802m，横向P95及Yaw退化。不能认定Native全面更好或替换Legacy。
- UNKNOWN：绝对协方差尺度、完整NEES、跨场景鲁棒性、已收敛错误分支识别、高速banking可靠性。小NIS和小修正不证明正确；Native仍不乘Fitness，Fitness仅对B0构R/B1独立诊断。关闭Fitness的GPU收益未测。
- NO-GO：50ms完整实现周期验收，B0/B1公共到达控制超时27/57和21/57；未执行精确Deskew/ROS控制端到端，不把GPU匹配时间冒充全周期。
- UNKNOWN：A2高速接触假设及垂向真值；本轮仅A残差/H Shadow，参考点是已知Velodyne点而非假定后轴。未见历史多m/s Vz实际跳变，但无独立垂向精度证明。C未融合。
- NO-GO：凭此短场景推进生产配置/新算法或Native标定。保持原Q/Bias、生产6D、TF/YAML和NDT配置不变。

只下载获批三包814833599字节，无历史清理、无其他KITTI/RACECARpayload。工作根现有全部分配约1.077GiB，包含BF0论文及失败准备，低于8GiB增量预算。下一步仅提交[RACECAR独立下载申请](RACECAR_DOWNLOAD_REQUEST.md)，停等批准。
