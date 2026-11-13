# BE5 A：base参考点垂向速度软观测（推导草稿）

HYPOTHESIS：以下是待批准的观测候选，尚未实现/测试。CONFIRMED合同：R=R_map_body，v=v_map_IMU，w=omega_body−bg，r=r_base_IMU，右扰动R_true=R Exp(delta_theta)，误差顺序为[p,v,theta,bg,ba]；角速度须先按实际外参旋转，不能用原Bosch轴。

记[a]×b=a×b，c=w×r。刚体参考点关系为

$$v_B=v-Rc,\qquad h_A=n^\top(v-Rc),\qquad z_A=0.$$

这与v_IMU=v_base+R(w×r)一致。n必须是同一map中的单位法向，r在body表达。base是车辆参考点，不是永远静止在路面的接触点；悬架/起伏会使软观测存在模型残差。

## A1：法向由独立、冻结的世界几何给定

在对状态求导时固定n、r和omega测量：

$$H_A=[\,0_{1×3},\ n^\top,\ n^\top R[c]_\times,\ -n^\top R[r]_\times,\ 0_{1×3}\,].$$

推导：delta c=[r]×delta bg，
delta v_B=delta v+R[c]×delta theta−R[r]×delta bg。
位置和ba无直接导数，不等于不会更新；K中的完整P仍使全部15D状态变化。若n随位置/地图插值变化，需另加
H_p=v_Bᵀ(∂n/∂p)，并处理选择/法向估计的不确定性；不能悄悄使用固定n公式。

观测残差r_A=−h_A，Joseph更新、右误差注入/reset与原模型一致；不能将v[2]置零。单次线性无相关噪声模型的P缩减方向由PHᵀ决定，不是单独改P_vzvz。

## A2：保守替代——body局部竖直近似

没有可靠路面法向时，可仅作为低动态Shadow模型试验：
n(x)=R e_z。此时

$$h_b=e_z^\top R^\top v-e_z^\top(w×r),$$
$$H_b=[\,0,\ e_z^\top R^\top,\ e_z^\top[R^\top v]_\times,\ -e_z^\top[r]_\times,\ 0\,].$$

重要区别：不能先设n=R_hat e_z，再用A1固定n的Jacobian冒充A2；n的状态依赖使角度导数发生抵消。A2是假设base法向速度接近0，不是假造路面法向，更不是非完整横向速度为0。保留真实vy。

HYPOTHESIS：坡面上body法向可能比map-Z近似合理，但侧倾/俯仰使body-Z不等于路面法向；误差项近似−n×delta_angle投影到高速水平速度，速度越高越敏感。赛车悬架heave、pitch/roll、压路肩、起伏可使假设不成立，A2仅适合来源清楚的平顺/低动态窗口。

## 噪声与共用IMU相关性

HYPOTHESIS：先用有限的保守contact variance，禁止零噪声硬约束。若独立误差假设成立，
R_A≈sigma_contact²+G_omega C_omega G_omegaᵀ+G_r C_r G_rᵀ+G_n C_n G_nᵀ，
其中G_omega=nᵀR[r]×，G_r=−nᵀR[w]×，G_n=v_Bᵀ；法向应在其2D切空间给噪声。
这只是模型展开，不是凭空已有标定。A2还需反映body-road倾角差，不能仅按陀螺噪声给极小R。

UNKNOWN：contact模型方差、可靠n及其独立性。地图/点云估计n与NDT共享数据，不能当独立证据；无需为找法向改变现有crop或新增B高度约束。

CONFIRMED（数学）：同一gyro输入已用于传播，A的omega项与state误差可能相关。若C=Cov(delta x,measurement noise)非0，S=HPHᵀ+R+HC+CᵀHᵀ，K=(PHᵀ+C)S^−1；标准Joseph无相关项不再足够。不能通过任意大R宣称已严格处理相关性。第一步只Shadow残差/J/K提案；弱软融合若获批准仍须明确“条件独立工程近似”，或先建立相关噪声模型。不存在可无条件立即部署的A。

## 适用/失效

HYPOTHESIS：有已验证路面n时优先A1；无n时只做A2低动态Shadow。上下坡真实map-Vz≈速度×坡度，强制map-Z=0会拒绝真实爬坡并通过P污染XY/姿态。弯道大vy、roll、传感器偏置/时延可能把真实速度误解为垂向；不能用“预测差较大”直接停掉正确NDT横向修正。

应暂停/降权的候选场景：IMU饱和/时间缺口、颠簸/路肩/跳跃、明显heave、法向不确定、姿态参考失效或body-road错位；阈值需在未反馈Shadow记录中制定并冻结，不能用GNSS标签在线触发。频率不得按每条200Hz IMU重复注入未经相关性处理的同一约束。

## 待批准的验证

有限差分：用现有inject对15列作中心差分，A1固定n、A2每侧重新计算R e_z；包含非零r/bg、坡度、bank、vy，步长1e−4/1e−5/1e−6尺度梯度检查，目标相对/绝对误差≤1e−6（近零列另用绝对误差）。验证位置/ba直接零列、r=0退化、静止/恒转弯/坡面反例。

单次更新：随机SPD且有交叉项P、正R；检查Joseph后SPD/对称、reset仅一次、拒绝保留prior/P，比较增益方向和所有15D间接增量；不允许硬裁剪。与wheel一起按固定顺序/唯一ID在测量时刻更新，测试延迟重放一致，不能重复计权。以上仅计划，本轮未运行。
