# BE5：垂向失效机制与证据层级（草稿）

## 已有证据，不能改写为新实验

CONFIRMED（历史原报告复核）：phase_z/Z0与Z1中的4.351277、6.268822m/s是同一冻结预测的Y优化候选累计delta_v_map_Z，分别来自固定初始关联解和后续关联子问题；不是已提交状态跳变、单轮更新或车辆真实Vz。私有Z0_INPUT_AND_STATE_CONTRACT保留身份，公共报告不分发轨迹/精确时间。

CONFIRMED（AA既有记录）：14.70s旧窗口最大raw dvz proposal=4.809311m/s，最大实际提交=1.081766m/s，最大carried |vz|=2.213607m/s。大proposal被拒绝；226次历史迭代上限部分接受不是已证明优化成功。该窗口不是BE全圈，也不是新的BE5指标。

CONFIRMED（AA）：首次几何更新将sigma_z从1.077427缩至0.015979m，而sigma_vz仍1.858844m/s；约50ms后Corr(z,vz)=0.985528，P(vz,z)/P(z,z)=19.417368/s。0.250287m附近Z提案可伴随约4.81m/s速度提案。完整6维交叉项仍参与，标量比值不是完整因果归因。

CONFIRMED（AJ）：即使只观测XY/Yaw，全P可产生较大Z提案并让本来很小的XY修正整帧拒绝；旧wheel也可间接产生米级Z更新。AJ最终Pzz=2265.3371、P_vzvz=35.90573、Corr≈0.957707，只是该旧配置局部证据，不是BE运行末端P。

CONFIRMED：BE-E0～E4交付仍为NO-GO；没有证明方向抑制或GMF解决了上述风险。归档保留的frames包含prior/update/state/P，但本阶段没有扫描全部帧重算BE最大Vz；因此“BE同样发生4–6m/s实际跳变”为UNKNOWN/不允许的推断。

## 数学机制

CONFIRMED（模型）：r=z-h(x)，K=PHᵀ(HPHᵀ+R)^−1，delta_vz=e_vzᵀKr。完整P使位置、姿态或wheel创新都能影响vz，非对角项不是应被删除的bug。

HYPOTHESIS（机制待量化）：预测产生p-v、姿态-速度及Bias耦合，早期过强/未标定几何R使位置P迅速收缩而速度仍弱约束，后续创新通过条件增益放大。姿态误差通过重力/比力投影积分，同时缺少真实垂向速度观测，可能共同造成漂移。关联变动、错峰、地图高度与时间误差也能制造创新，不能只归罪于Q或R。

CONFIRMED（可设计归因）：冻结每次实际prior、原残差和K，保存每行K_vz,j r_j、sum与delta_vz一致；wheel进一步按P_vz,v / P_vz,theta / P_vz,bg等块分解。对延迟更新要区分scan-time proposal、commit后的历史状态、传播到发布时刻的状态；rollback后账本修订也须跟踪，不能直接拿两个不同时刻v相减。

UNKNOWN：实际Z/垂向速度真值、地图垂向基准、VectorNav姿态/高度独立性和时间标定。GNSS工程XY参考不足以给出Z/Vz的绝对准确性或完整NEES。

## 基础问题优先级

CONFIRMED：代码的右误差、完整P、Joseph/reset和一次轴向旋转清楚；实际TF旋转非单位阵。已有数值回归支持离散方程导数，未证明真实安装/传感器模型完美。
HYPOTHESIS：已知ZOH噪声近似、未标定Legacy R、输入时间与参考误差可能影响统计尺度；本轮不顺手调Q/Bias或修改安装参数。
UNKNOWN：原始IMU是否在驱动处做过重力补偿的完整现场证据、光学实际测量点/残余延迟、部署Native发布合同。需要日志/源码身份闭合，不以新约束掩盖。

## 验证边界

HYPOTHESIS（未来计划）：先在已有冻结prior/观测里解释增益和候选/提交差异，再比较A/C/R单因素；不清零Vz、不裁剪P交叉项、不直接锁定姿态，不使用方案B高度因子。A不能修复错误位姿峰，C不能自动提供高度可观性，Native局部小R不能证明正确分支。数值稳定、真实误差、观测保留和预算分别验收。

CONFIRMED（坐标区别）：P中theta是body右误差，不直接等于Euler Roll/Pitch。未来若报告P(vz,roll/pitch)或Euler姿态误差，需要先用当前R的局部Euler Jacobian变换交叉项；不能只给P[5,6:8]换名。
