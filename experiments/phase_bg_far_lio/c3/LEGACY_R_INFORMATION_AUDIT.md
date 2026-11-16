# Legacy R与单次LiDAR信息预算

CONFIRMED：`src/geometry_ieskf/ndt_pose_observation.py::engineering_R(fitness,smoothing_steps=5)`：非有限、负值、f>15拒绝；f下限1e−5。f<3时XY=.05f、yaw=.00025f；f≥3时XY=1.5f、yaw=.6f，再乘smoothing_steps。

C1调用六维ExternalPose，取上述XY值扩展XYZ、yaw值扩展三个body旋转创新分量。因此：

\[
R_5=5\operatorname{diag}(a_f,a_f,a_f,b_f,b_f,b_f),
\quad(a_f,b_f)=\begin{cases}(.05f,.00025f)&f<3\\(1.5f,.6f)&f\ge3.\end{cases}
\]

C1全6DoF不是生产平面XY/Yaw模型。完整P与H通过杆臂及右SO(3)创新耦合位置、速度、姿态、Bias，即使Legacy R本身对角也不能省略P交叉项。K=PHᵀ(HPHᵀ+R)⁻¹，姿态校正后Joseph/reset，不能按f独立推算横向增益。

×5来自生产五次尝试平滑队列的历史预算。在理想同变量、相同线性化且全部成功时，五次5R的总信息约等于一次R；真实延迟/拒绝/历史块不同，不能自动等价。C1每信息组一次接入，旧观测重传播是从不含其后验的检查点重算，不能算再吸收一次独立信息。其×5没有新统计标定依据；同时也没有依据直接除5。

本轮固定R_m=(m/5)R_saved，m=5/2/1；所有prior/P/位姿/时间/ID/形状相同，未调用新的engineering_R分支选择R，也未使用Native或FAR R_init。×5与297个原更新K最大差2.5313×10⁻¹⁴，δ最大差2.2204×10⁻¹⁶；BE49个保存后验P最大差8.8818×10⁻¹⁶。

CONFIRMED：right3横向位置增益投影中位×5/.10298、×2/.22282、×1/.36411；normal8为.09764/.21190/.34790。降低R确实增加本组位置修正能力，但非标量线性倍增；姿态/速度/Bias也同时改变。全部891候选P有限/SPD，最小特征值5.00237×10⁻⁷；这不是统计一致性或物理Bias可观测性证明。

原Legacy×5是需要保留回滚的工程基线，不是默认正确答案。×2/×1只是信息尺度消融，不称标定R、自适应R或已验证生产配置。量化“多少横向不足可归因×5”只能给同prior即时差值，不能把该差值解释成完整因果轨迹中R的贡献比例。
