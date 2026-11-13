# Native转换最小修复与坐标合同

## 修改范围

CONFIRMED：只修改研究pose_uncertainty.py的native_pose_R：
移除engineering_R(fitness)调用；验证输入pose/C、显式可选固定T_map_pcm；保留完整M C Mᵀ及交叉项，Cholesky检查SPD，无对角化/Native×Fitness。Legacy函数、Q/Bias、滤波器/生产策略和优化器未变。新增的pose/frame一致性检查是测量合同验证，不是调生产创新门限。

直接从git基线加载旧函数作合成前后对照：

| 合成Fitness输入 | 旧Native助手 | 修复后 |
|---|---|---|
|正常0.2|可转换|可转换，原合法矩阵保持数值一致|
|字段缺失|KeyError|可转换|
|100|Legacy范围异常|Native有效时可转换|
|NaN|Legacy范围异常|Native有效时可转换|

这些仅证明解除无关Fitness条件，不表示NaN Fitness对应的实际匹配正确。Legacy对0、低分、高分、边界、超范围、NaN的结果/异常与旧函数逐值保持，正常矩阵输出bit-identical。

## Jacobian与单位

输入为PCM-from-base的[t, a,b,c]，R=Rx(a)Ry(b)Rz(c)。B=[RzᵀRyᵀe_x,Rzᵀe_y,e_z]映射到body-right旋转。固定PCM→map旋转A使平移噪声乘A，body-right噪声不因左乘A而变；创新phi=Log(R_priorᵀ A R)的噪声Jacobian为Jr(phi)^−1 B。完整M包含相应平移和旋转块，不删任何交叉项；位置仍在base，ExternalPose的p_IMU−Rr处理杆臂，避免重复转换。

平移方差m²、旋转创新方差rad²、交叉项m·rad；不能直接把内部Euler角方差贴成ROS fixed-axis或ESKF右误差。原6×6混单位谱只作数值SPD核验，不作为物理退化得分。

合成包含非单位PCM→map旋转及平移、非零RP和相关SPD C。完整图Jacobian直接FD最大误差3.93585983e-10，全协方差FD差异5.50035018e-11。另验证yaw图、不对称、不定/奇异、非有限、位置图不符、非刚体frame失败，无静默回退。

## 实数据合同范围

CONFIRMED（保存BE数值合同）：只针对controlled2 matched离线链。捕获的五个BE源文件SHA与源码一致；base seed明确p_IMU−Rr；Deskew使用base历史，common/native变换为I；直接将map0105 target与seed送给私有matcher，并直接保存align参数/最终pose/Native C，没有调用生产PCM→ENU发布适配器。

每个49 BE案例再检查时间、prior→base seed、seed_float、参数RxRyRz与final矩阵的旋转/平移一致性，全部通过，才显式传I做Native Raw。这不是“默认I所以认为对”，是该保存程序的共同数值map坐标。未知合同的单元测试强制失败；旧Z缺Native位姿/C合同及12632缺配对prior/P，均停止对应对照。

UNKNOWN：地图测量/大地基准、现场lidar_ndt或v1加载组件、生产PCM/ENU适配与服务二进制精确源码重建等价性。此证书不能推广到部署或其他记录。若后续需这些范围，必须另确认，不能借本测试默认通过。

## 统计限制

Native Raw未作尺度标定。原I/N、截断residual_scale、miss及best-voxel模型不变，不以Fitness补乘。已保存kappa_at_float_pose是另一次float位姿查询，不拿它除回内部double参数的C。小C仍不证明正确分支；Native Raw通过数值合同不是生产R替换资格。

最终driver还在任何候选前绑定证书allowed_run、Target身份与捕获的五个源码SHA；坏运行/Target/源码绑定的负例均停止。这一元数据绑定补强未重新计算55案例的数值候选。

非单位T_map_pcm的调用方还必须同时用T_map_pcm @ T_PCM_base转换观测均值；native_pose_R只返回协方差，不会替调用方修改mean。当前BE合同为I，因此相同最终位姿严格保持不变。合成FD已在同一变换后的mean图计算。
