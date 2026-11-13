# BE5 C：Roll/Pitch参考软约束（草稿）

## 来源决策

CONFIRMED：a2rl_ahrs/src/ahrs_node.cpp中可从VectorNav common选择同步数据，失效时回退Bosch；代码使用陀螺、加速度及Fusion AHRS，输出frame base_link并处理NWU/ENU航向转换。配置use_common_msg=true不能证明每个时刻实际使用VectorNav。源码没有提供已验证的逐帧姿态统计协方差。

CONFIRMED：若使用Bosch回退，与15D传播复用同一原始IMU；不能把它作为独立姿态测量。VectorNav路线在硬件上可能不同，但其内部GNSS/INS、AHRS和初始化依赖及地图共同误差仍UNKNOWN；不能凭不同topic判独立。

CONFIRMED：当前15D离线motion主导出没有独立的AHRS/VectorNav姿态更新事件；离线评价里的姿态被用来转换天线杆臂，不能直接改成在线参考。Deskew服务在BE使用15D自身状态历史，输出/中间姿态不构成新的独立传感器；在当前可查生产树只找到历史Deskew审计/安装快照，不能认证当前实时Deskew实现/部署。

UNKNOWN：loop44是否另有可追溯、时间对齐且足够独立的姿态原始流，当前逐帧AHRS源选择、内部GNSS辅助、误差相关性、姿态协方差和垂向真值。

决策：**C现在仅支持来源/残差Shadow设计；不批准假定独立的强融合。** GNSS关闭时不得借“外部姿态”绕过生产开关引入GNSS位置/隐含未审计的INS辅助信息。

## 由实际右SO(3)误差推导

HYPOTHESIS：参考R_ref必须先在相同map/body约定、外参和观测时间对齐。不要相减两个不同Euler约定的roll/pitch。

一般SO(3)参考误差q=Log(R_refᵀ R)，固定参考右扰动时
delta q=Jr(q)^−1 delta theta。若选择固定投影E，有H_theta=Eᵀ Jr(q)^−1，但当yaw未知/相差大时，简单取q的前两分量会引入非交换耦合；不能声称这样严格只观测roll/pitch。

推荐诊断表述为与SO(3)合同一致的两维tilt方向因子：令k为map向上单位向量，
u=Rᵀk，u_ref=R_refᵀk，E_ref为u_ref正交切空间的3×2基。

$$z=E_ref^\top u_ref=0,\quad h=E_ref^\top R^\top k,\quad r=E_ref^\top(u_ref-u).$$
$$H_C=[0_{2×3},0_{2×3},E_ref^\top[u]_\times,0_{2×3},0_{2×3}].$$

因为Exp(−delta theta)u≈u+[u]×delta theta。该S²残差由右SO(3)扰动推导，yaw沿u方向为零空间，保留参考所含真实车身roll/pitch，而非固定为0。需要同一半球/局部角度有效域；对近反向/大偏差必须标记不可用，不能靠切平面残差在反向时又等于0而误接纳。

若参考姿态有右误差协方差C_ref，局部残差R_C=E_refᵀ[u_ref]× C_ref [u_ref]×ᵀE_ref，并加入时间同步/外参不确定性。切空间是两维方向误差，不是可直接沿用Euler角的单位方差；变换完整协方差并明确参考所在轴。

## 相关性和动态污染

CONFIRMED（物理模型）：f≈Rᵀ(a_map−g)+ba，高速横向/纵向加速度不是重力；直接归一化−f作为重力方向会把转弯/制动误认为倾斜。加速度范数接近g也不充分证明方向可靠。不能锁RP、用yaw-rate或单一加速度门限宣称独立真实姿态。

HYPOTHESIS：若有独立来源、独立时间/外参标定，可用低动态窗口先验证；阈值与噪声来自来源规格和独立记录，保留真实悬架运动。相同IMU来源只能Shadow；单纯调大R不能消除未知相关性。若未来建立共享误差C_xn，则使用A报告中的相关噪声更新，或另审批联合状态/参考模型；本轮不新增复杂姿态滤波器。

C只直接约束两个tilt方向，通过全P可能改善重力投影和部分Bias；没有观测绝对高度，也不直接测Vz，不能从根本上替代垂向运动可观性或正确的地图/速度模型。

## 待批准测试

右inject中心差分，验证任意yaw、非零RP/外参下H；验证R→Rz(alpha)R不改变tilt观测；测试半球/失配/时间缺口、外部源切换和完全重复Bosch来源拒绝融合。检查参考姿态协方差变换、Joseph/reset、延迟ID去重，合成动态加速度反例。没有来源合同则E2/E3只能Shadow/N-A，不得为填满实验表伪造独立姿态。
