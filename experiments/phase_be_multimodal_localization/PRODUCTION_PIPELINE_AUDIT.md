# 生产与历史审计

2026-11-13核实研究远程main=05b73c0aae6b9f5440f8497477dfe9909ebfe9e1；c647ac3为原始研究对照，不能称当前main。BE独立分支自05b73c0开始。生产仓库当前ndt_test，HEAD aa50e35467f89b4963f882d1f5abaace46d0ce70，有既存fusion_vendor未跟踪内容。没有修改生产、main、Q/R、stash或历史分支。源码与YAML值不是现场运行参数快照。

## 实际生产模型

state_index/state_transition定义[X,Y,YAW,YAWB,VX,WZ]。X/Y按vx与yaw+yaw_bias传播，yaw按wz传播；没有vy。measurement.cpp的Twist矩阵仅提取vx,wz。converter可以输出Kistler vx/vy，但vy不进入该六状态模型，且低速轮速回退明确vy=0。yaw_bias是既有安装/航向校正语义，不可重命名为侧滑角。独立Z/roll/pitch滤波与研究15D必须分别评价。

## 观测与延迟

lidar_ndt的context.stamp取输入LiDAR header，输出Pose沿用该时间；具体扫描起点/终点语义和各雷达融合/deskew时间需元数据核实，不能由stamp字段名推断。EKF callback入队，timer读取固定初始队列长度；AgedObjectQueue pop先增加age并按age<5重排，再执行观测更新。失败也耗费一次，已有观测不会因新一帧未入队消失。poseMeasurementCovariance乘smoothing_steps。该做法不能当五个独立证据。

YAML predict100Hz、extend15，历史约140～150ms；实际累计dt决定期限。updateWithDelay使用增广历史和交叉P，非AY500ms chronological rollback/replay。超过窗口返回失败，不能改时间戳挤回窗口。关闭全部位置源的既有服务逻辑可清队列，单独关闭GPS不等价于撤销已排队GPS；本轮不改此语义，默认从开始即use_gps=0，避免切换残留混入GNSS-off实验。

## GNSS入口（详细见GNSS报告）

use_gps声明默认0，1启用，2为已有planner区域AUTO；动态参数及SetSensorModes能改变其值。不是FIX8自动开关。默认launch把in_pose_with_covariance连到converter的pose_from_gps。converter读取GPS1原始LLA→ENU→标map，杆臂补偿被注释；EKF直接读XY/yaw，不执行ant1→base_link。新VectorNav Pose是另一话题且默认配置目标仍ant1；不能凭它有TF代码就认为旧EKF链已补偿。

## 历史证据边界

复核BD报告及原始汇总：收敛/正定曲率无法排除12632错误盆地；KD尚无合格闭环；正常段8有弱横向几何但有效观测；纯NDT-off长期漂移不可接受。30帧15D反事实保持真实回溯，但拒绝后未重新运行matcher，不能当生产闭环安全证明。BD严格50ms最大延迟不合格，见预算报告。

AW已有small_gicp真闭环，AX点级直接融合恶化的历史结论不支持直接替换；本轮仅采纳其既有资格限制，不重做实验。历史75ms GNSS只是工程参考，0/150ms可改变排序。未经独立标签核验的“正确/错误”称代理标签或未定，不输出假精确接受率。

## 待核实而非已证明

现场launch覆盖/服务更改、安装版本与源HEAD一致性、raw GPS参考点硬件语义、真实基准map原点、姿态时间同步、Kistler参考点/轴向/单位、各雷达point来源标记、控制器实际消费frame。GNSS坐标存在明确源码缺口，不能签发“已是后轴中心”。证据保存在本机私有审计，不发布生产源码/静态外参。

## 原始历史文件再核验

AW最终annotated summary明确区分actual_small_gicp/actual_CUDA_Deskew与actual_CUDA：GICP本身CPU，单圈1468、三圈4134均处理完成。早期pre_backend_metadata_fix的evaluation残留actual_CUDA=true不能继续引用为CUDA-GICP证据。AX A_lap处理1468；B_lap只处理531，是失败/不完整结果，不能当完整单圈。用户提供的0.778m历史数值本轮不重新计算为BE成绩；更细排序须对齐原evaluation的时间/覆盖/参考点。私有审计已记录这些文件SHA256和关键非重构元数据。
