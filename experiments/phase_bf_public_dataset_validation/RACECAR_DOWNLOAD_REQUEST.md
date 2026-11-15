# RACECAR历史下载申请 — 用户不批准，归档备选

仅一个官方场次：LVMS S3 / M-SOLO-FAST-100-140（MIT-PITT-RW，名义100–140mph）。KITTI BF1已完成，见[结果](BF1_KITTI_REPORT.md)。不因KITTI结果更换场次；不下载E场次、nuScenes或第二个场景。

| 文件及完整官方地址 | 当前HEAD大小 |
|---|---:|
| [M-SOLO-FAST-100-140.db3](https://racecar-dataset.s3.us-west-2.amazonaws.com/RACECAR-ROS2/S3/M-SOLO-FAST-100-140/M-SOLO-FAST-100-140.db3) |20711575552字节|
| [metadata.yaml](https://racecar-dataset.s3.us-west-2.amazonaws.com/RACECAR-ROS2/S3/M-SOLO-FAST-100-140/metadata.yaml) |16132字节|
| 合计 |20711591684字节（20.712GB，约19.289GiB）|

CONFIRMED：官方元数据约900秒、5533818消息，未压缩SQLite3单文件，三路LiDAR及约125Hzrawimux，约20HzBESTPOS。无需解压，不再保存全量转换bag。单SQLite包含本次不需要的雷达/相机info，不能按话题选择下载文件的一部分作为完整bag。可复用BF0官方元数据/参考源码；源bag从未下载。

计划仍选新加卷单盘方案A，不使用T7。实际完整私有路径见本地STORAGE_PATHS审计，公开文档以WORK_ROOT表示BF根目录：未来raw/RACECAR/S3/M-SOLO-FAST-100-140/保存原bag，derived/racecar保存小索引，maps/racecar保存唯一地图，results/racecar保存状态/指标，tmp保存临时缓存。当前新加卷可用352339312640字节（328.14GiB），T7可用166216073216字节（154.80GiB）；两盘容量不相加。

增量峰值预留64GiB：原bag约19.289GiB，索引/有限转换8GiB，地图4GiB，完整必要状态/R/K/P及统计8GiB，临时4GiB，元数据0.25GiB，其余安全余量。下载后原bag只读保留；不复制每帧点云，每组共享Source/map；不自动删除下载文件或历史成果。使用64GiB后新加卷预计仍有264.14GiB。无需历史清理，也不申请清理KITTI；KITTI归档继续保留。

研究价值：相同Legacy/NativeRaw管线测试更高速直道、弯道及banking，A仅Shadow、C禁融合。初步计划首个完整圈建图、后续圈测试，同一场次且不逐圈初始化；必须先核实真实圈界/地图覆盖并冻结，地图/测试不混用。没有独立地图时仍只称参考辅助功能验证。

风险与先行停止条件：rawimux真实来源/单位/轴向/比力合同未验证；官方RAWIMU示例存在加速度赋值错误，不复制该实现。多个INS姿态话题为空；BESTPOS在天线相位中心，必须核实IMU/天线/车体外参和时钟，不能拿天线位置当后轴真值。原始点云逐点时间、帧ID和RTK完整性未知。合同失败则停止对应15D/精度实验，不借GT速度、位姿或C观测补齐。无GNSS在线更新；接收机内部修正相关性单独披露。50ms预算尚不通过，不承诺高速生产GO。

约束：不新增算法、不改NDT/优化器/Q/Bias/门限，不调Native尺度。开始下载前重新核对挂载UUID、只读写状态、空闲空间、文件HEAD和64GiB预算；若大小/路径发生重大变化或不足则暂停重新申请。源许可CC BY-NC4.0须遵守，不公开再分发原bag。

**状态更新：用户明确不批准RACECAR下载。公开数据验证仅保留一个KITTI场景；高速验证后续只考虑已有A2RL数据包。本页历史大小及存储估计仅作备选存档，未重新查询，不构成执行计划或待办申请。**
