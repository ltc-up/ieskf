# 9D与15D车辆状态模型审查

## 实际模型合同

CONFIRMED：FAR官方缓存commit `ef20510316ad1f8dbf2e428e4a193f700493eccd`。C1使用原生三维九状态，不是平面三状态，也不是固定Bias的等价15D。源码入口和版本见[SOURCE_EVIDENCE](SOURCE_EVIDENCE.md)。

| 项目 | FAR九状态 | C1研究15D |
|---|---|---|
| 状态 | map位置、Euler φ/θ/ψ、body速度vx/vy/vz | map下IMU位置/速度、SO(3)、body陀螺/加计Bias |
| 参考点 | 适配后的base车辆状态 | IMU状态，观测转换为base |
| 离散预测 | 固定TS=0.01s，前向Euler | 实际事件dt、ZOH惯性输入、中点姿态积分 |
| Bias | 独立IMU Handler低速学习陀螺均值；加计初值固定 | 六维Bias随机游走，完整交叉P |
| Wheel | 订阅接口存在，但本次无轮速输入 | 原标量纵向Wheel更新及杆臂 |
| 倾角 | 自身速度导数+同IMU生成RP参考并融合 | 本次无A/C倾角额外软约束 |
| 更新P | (I−KH)Ppred，残差可裁剪 | Joseph、右误差注入及reset |

FAR名义方程，p/q/r为body角速度，f为已处理比力：

\[
\dot p_M=R_{MB}v_B,\quad \dot\eta=E(\phi,\theta)\omega_B,
\quad\dot v_B=f_B-\omega_B\times v_B+R_{MB}^{T}g_M.
\]

展开横向：\(\dot v_y=f_y-rv_x+pv_z-g\sin\phi\cos\theta\)。因此Yaw率与纵向速度显式耦合，**并未假设vy=0**。世界系15D没有显式−r vx项，因为速度在map积分；两者在同参考点、正确坐标转换及连续时间理想条件下可等价，不能把不同表达形式当成额外传感信息。

15D实际中点模型：\(\bar R=R\exp((\omega_m-b_g)dt/2)\)，\(a_M=\bar R(f_m-b_a)+g_M\)，\(p^+=p+vdt+a_Mdt^2/2\)，\(v^+=v+a_Mdt\)，\(R^+=R\exp((\omega_m-b_g)dt)\)。右扰动\(R\exp(\delta\theta)\)。连续块结构有\(F_{pv}=I,F_{v\theta}=-R[f-b_a]_\times,F_{v b_a}=-R,F_{\theta b_g}=-I\)；实际离散F还保留中点旋转对bg的位置/速度耦合。完整公式以`inertial.py::model`为准，未改动。

CONFIRMED：15D \(P^+=FPF^T+GQG^T\)，白噪声密度按dt离散，Bias RW按dt增长；配置gyro/accel密度0.005/0.1，Bias RW 0.0001/0.001。FAR以\(A=I+TS A_c,B=TS B_c,Q_d=BQ B^T\)传播，配置六输入Q=[0.00035,0.0003,0.003,1,2.4,4]；这些是各实现内部的输入噪声参数，**不能与15D连续噪声密度逐数值等同**。FAR初始P为原生零矩阵，15D按原研究初始标准差生成完整P；C1未复制历史成熟P。

## FAR线性化发现

CONFIRMED（源码代数，不是新数值实验）：`ekf_3d.hpp`名义速度含重力随φ/θ变化，但A的速度行、姿态列全为零。应存在的局部导数包括：

\[
\partial_\theta\dot v_x=g\cos\theta,\quad
\partial_\phi\dot v_y=-g\cos\phi\cos\theta,\quad
\partial_\theta\dot v_y=g\sin\phi\sin\theta,
\]
\[
\partial_\phi\dot v_z=g\sin\phi\cos\theta,\quad
\partial_\theta\dot v_z=g\cos\phi\sin\theta.
\]

同文件`dx_pos_y_wrt_theta`的vy项写为sinφ cosθ vy，与名义y方程导数相比缺sinψ。HYPOTHESIS：会影响P传播与增益；UNKNOWN：在本次精度差异中的大小。未修改官方代码，也未执行新Jacobian实验。更简单模型并不自动意味着线性化更一致。

## Bias与可观测性

CONFIRMED：FAR并非始终固定Bias。`update_imu_offsets`在估计速度范数<0.4m/s时开启IMU Handler陀螺样本均值学习，加计保持配置初值；不估计Bias协方差。Pit中陀螺Bias从零改变，normal8/right3中固定，加计始终零。本次保存debug可证明该机制执行。FIR保留，但同一IMU回调分设accel/gyro时均调用Handler，不能简单把5tap解释成200Hz的固定10ms延迟；实际有效频响/相位贡献UNKNOWN。

15D保存状态统计：

| 指标 | normal8 | right3 |
|---|---:|---:|
| ‖bg‖范围(rad/s) | .001911–.002445 | .001601–.002184 |
| ‖ba‖范围(m/s²) | .05623–.06181 | .05572–.06058 |
| 首尾bg向量变化范数(rad/s) | .000524 | .000601 |
| 首尾ba向量变化范数(m/s²) | .00696 | .01891 |
| 相邻保存状态ba最大变化(m/s²) | .000359 | .000708 |
| 最大tilt–ba归一化P相关系数绝对值 | .01585 | .01196 |

Pit最大tilt–ba相关系数约.9734，符合静止时倾角/横向加计Bias难分辨的理论风险。行驶中相关系数下降、σ收缩只说明当前模型的信息分配，**不证明物理Bias可观测性已建立**。没有独立Bias真值、完整可观测Gramian/模型失配检验或NEES；不能把right3 ba与误差相关称为异常漂移因果证明。保存K表明NDT确实修正Bias，但修正可能吸收地图、时间、姿态及关联误差。

HYPOTHESIS：低速估计后固定陀螺Bias，短时可能避免弱可观测Bias被错误LiDAR牵引；长时间无LiDAR、温漂或动态Bias时则可能积累误差并低估P。15D的Bias状态有物理意义，但存在弱可观测工况；**当前证据未证明它们导致数值失稳**。C1全部保存15D P有限/对称且正定，不等于统计一致。

## 轮速、侧滑、倾角

CONFIRMED：FAR非完整模型由wheel输入触发生成[vx,0,0]，其wheelspeed状态默认STALE；C1没有向FAR发布wheel，故该分支并未提供零横向速度观测。`model/vy_mps=0`是未馈入的vehicle_model_velocity，不是实际EKF vy。保存Odometry实际body vy在right3为0.729–1.841m/s，不能断言纯侧滑真值，因为还受姿态/融合误差影响。

15D Wheel是\(e_x^T(R^Tv_I-(\omega-b_g)\times r)\)，包括姿态/bg Jacobian；只约束纵向，不锁vy。Optical未开启。两框架运动输入不同，现有比较不是状态维度消融。

FAR RP参考由同IMU及自身速度差分计算：roll输入含\(-\dot v_y+f_y-rv_x+pv_z\)，pitch含\(\dot v_x-f_x-rv_y+qv_z\)，反正弦前裁剪到±.5。right3 roll参考绝对P95达到30°饱和边界；这是内部参考，不是独立姿态真值。其改善/污染的净贡献UNKNOWN。15D用SO(3)传播重力，无该自身参考重复信息。

## 决策

CONDITIONAL：保留三维p/v/姿态；banking、坡度和非零vy使纯平面/零侧滑简化缺乏依据。Bias是合理候选状态，不能凭本次末端误差删除。9D研发暂不启动：未来需同NDT、同传感输入/参考点、同R和时序的受控矩阵，并为外置Bias不确定性建模；仅冻结15D Bias不等价于9D。优先处理观测信息预算与记录完整性，再决定是否减维。
