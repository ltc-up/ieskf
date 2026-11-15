# 身份、冻结合同与历史等价性

研究HEAD为批准基线6e43ecb0a53470f229e7d0882ccb4e7950ee14ec，当前分支research/phase-be-e0-e4-shadow。工作区开始干净、stash保留。没有创建仓库副本或分支。主机验证新加卷NTFS3 rw、约328GiB空闲；沙箱视图ro未作为真实挂载错误。T7及系统盘未用于实验输出。

|身份|SHA256|
|---|---|
|BF1实际Matcher外层|8ae76d7ece996cf89f498fba2519ba4c73d76b8f64beb69ffd5884a243ea1c08|
|实际加载原动态库|0c229305a733c467a5a63019489768f4ea0b626f449cff23c56b707ccb0ed54c|
|外层研究模板|9fd499e21ce43cdbfba3422f5621fe0a4666eb723cb97e07c34bfac5d1a3e72e|
|冻结Target|293300a932d4ad7cb9cb6d7225d48dcd6a78b68f78c3a45b504f4146f28e3f2d|

另一个旧同名私有Matcher哈希不同，已排除。实际/proc进程映射核实加载原库，不仅检查磁盘文件名。原库未重新编译、修改或复制。CMake历史源目录记录指向审查源码；编译到源码逐行可重构合同仍非新证明，但本次数学执行复用相同二进制库，且逐帧结果验证消除了误用另一库的混杂。

只为导出已有`pcl_fit_ms`、Source/Target读取和服务准备阶段编译小型研究外层。原模板哈希先验证，5处唯一锚点替换只增加计时变量/导出字段。优化、配置构造、Source、Seed、ROI和align/evaluateFinalQuality调用顺序不变；计时和trace可能改变墙钟，因此不宣称实时性能等价。新外层二进制/头文件/编译命令/loaded-library身份保存在本地清单。默认不运行诊断脚本即不影响任何已有执行路径。

五份Source使用BF1既有缓存，逐点字节与原始KITTI点云按原外参转换所得float32完全一致；校验只在内存，不输出新点云。Target、map/test20/57划分、坐标合同、每帧原B0 Seed/时间戳、q50及ROI完全复用。现有Source/map仅数MB校验，不全盘重新哈希。质量配置仍启用PCL-like Fitness、shell6，Native额外查询与历史一致。

逐帧对比：最终位姿、内部参数、Fitness、score、Native完整6×6、最终梯度/Hessian、最后步长及相对变化，均通过预设容差；退出、次数、Source/支持数量及Seed float完全一致。实际数值最大绝对差60/64为4.44e−16、65为1.11e−15、66/76为0。不是通过放宽容差掩盖复现差异；详细atol/rtol保存在逐帧equivalence记录。

现有trace未直接输出位姿/完整梯度向量。完整6DoF前后位姿由最终内部double参数逆向累加119条已记录applied_delta重建；RxRyRz图重建与Seed旋转最大差<8.9e−8，最终平移转float32与输出逐项一致。完整Newton方向/梯度向量依据保存H、raw_norm及应用方向按原SVD解合同重建，区分可能的上升方向反转，并校验梯度范数相对误差最大4.14e−13。此次全部H满秩，方向没有反转；这些字段明确为**重建值**而非直接导出的向量。原始trace仍完整保留。未导出的候选关联身份及每个CUDA阶段不能靠重建假称已测。

每步Hessian以`−Hscore`表示局部信息；原始6维m/rad谱仅检查数值有限性、秩/符号，不解释物理条件数。方向性使用完整RxRyRz到世界扰动变换后`s,d,L*yaw`的交叉项，L=5/10/20；固定其他自由度的投影不等于边缘化统计信息，也不作为Native R或拒绝规则。

Native质量仍沿用原构造默认值：目标体素协方差特征值比下限0.01、原生信息特征值下限1e−6、covariance_scale=1、outlier_ratio=0.55、每体素最少6点；这些为审查模板/头文件值，未新增运行时改写，残差尺度计算仍在同一原库中。完整Native矩阵与历史一致是本次直接等价证据，不将此称统计标定。
