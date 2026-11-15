# 冻结身份与Fitness缺失合同

当前分支research/phase-be-e0-e4-shadow，启动HEAD等于批准基线，工作区干净。主机实际新加卷NTFS3 rw，约328GiB可用；不使用系统盘/T7。复用P2已经核验的三份Source、Target、原B0 Seed、ROI、时间戳和全部参数，直接读取P2请求，不用OXTS重新生成Seed。未创建、复制或转换点云。输入前后SHA一致。

原Matcher动态库SHA256仍`0c229305a733c467a5a63019489768f4ea0b626f449cff23c56b707ccb0ed54c`，研究外层模板仍`9fd499e21ce43cdbfba3422f5621fe0a4666eb723cb97e07c34bfac5d1a3e72e`。新外层通过原有`enable_pcl_fit_score`控制已有分支，不修改或重新编译该库。两驻留进程分别固定ON/OFF，每次串行调用且轮换先后顺序；GPU库路径以实际进程maps核对。

保持DIRECT7、GLL3、q50、35次、step_size0.2、transformation_epsilon0.01、relative_objective_epsilon0.001，Native尺度/关联/质量不变。trace关闭，两模式都加相同前/后device同步屏障，仅用于诊断。ON仍执行原完整PCL Fitness；OFF不启动其kernel，也不构建仅供Fitness使用的raw-point索引，但相同Gaussian地图保持。原库已将优化及Native最终质量置于Fitness分支之前，可靠性/有效性判断也不依赖Fitness。上述是正常执行的数学依赖关系。PCL分支与align共享异常处理，CUDA错误可导致success/converged改变，raw索引也有资源耦合；不具备故障隔离。本轮未做故障注入。此为当前审查实现，不代表所有生产上层接入逻辑。

开关仅在本次研究外层暴露为P3_FITNESS，未设置默认ON，非法取值拒绝。OFF输出`fitness_available=false`、`fitness=null`、`fitness_complete=null`、`p2_pcl_fit_ms=null`；不把原库未计算的默认infinity/0计时传成有效Fitness。`legacy_R_generated=false`，没有导入Legacy/Native R生成器、滤波器或回放模块。

18份输出逐项比较P2历史结果及相同轮ON/OFF：最终位姿、内部参数、NDT score、完整Native6×6、最终梯度/Hessian、步长/相对变化、迭代/收敛/退出、候选有效、Source/支持数量、额外Native内点/邻居与残差尺度，最大数值差为0。ON Fitness也与P2一致。所有18个Native矩阵SPD，测试不是尺度校准。若出现超过1e−9 atol/rtol差异，运行器会立即停止，无自动重试；本次没有触发。

只编译小型研究外层，默认生产路径完全不受影响。完整配置、外层源码/二进制哈希、编译命令和前后身份在本地identity清单，公开不包含生产原始源码、地图或原轨迹。
