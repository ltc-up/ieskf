# Phase BF：BF1及只读复盘完成；RACECAR不批准

基线143ce47。仅下载获批的KITTI0002三包，固定前20帧建图、后57帧测试，两组均完成。Native有局部XY收益，但横向及Yaw退化；50ms预算未通过。所有结果仅属参考辅助功能验证，不构成独立精度或生产认证。未执行A/C融合，未修改生产代码、Q/Bias或NDT配置。

[完整KITTI报告](BF1_KITTI_REPORT.md)、[原始聚合统计](BF1_RESULTS.json)、[协方差对照](COVARIANCE_COMPARISON.md)、[性能](PERFORMANCE_COMPARISON.md)、[测试](TEST_RESULTS.md)、[失败与限制](BF1_FAILURES_AND_LIMITATIONS.md)、[RACECAR下载申请](RACECAR_DOWNLOAD_REQUEST.md)。

BF0原始记录可从143ce47查看，原manifest另存BF0_MANIFEST.json。SiPS全文访问缺口仍明确保留。RACECAR原始数据尚未下载，用户不批准下载，旧申请仅作备选归档。

最新：[KITTI只读复盘](review/README.md)。未开展新匹配、回放或算法修改。后续最小实验仅提案，当前停止。

最新：[BF1-P2五帧诊断完成](p2/README.md)。恰好5次匹配，0次回放/状态提交。0次GLL回退；PCL Fitness墙钟为主要耗时线索。当前停止，未批准下一轮对照。

最新：[BF1-P3 Fitness ON/OFF有限验证](p3/README.md)已完成，18次匹配结果等价，节省约35–37ms；无R构造/回放。当前停止，不修改生产Fitness。
