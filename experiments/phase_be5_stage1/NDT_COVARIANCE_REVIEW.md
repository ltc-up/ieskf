# BE5 R：Legacy/Native源码及统计审查（草稿）

## 路径和当前事实

CONFIRMED：aa50的lidar_ndt/src/ndt_process.cpp:143–168选择DIRECT7、q50、GLL3和YAML covariance_scale；YAML启用PCL-like fitness。measurement_policy.h:26–103先检查candidate/Native可用性，再在pcl_fit_enabled且score有限时用Fitness对角协方差，关闭或非有限才拷贝完整Native。该policy不是“只有converged才发布”的条件；candidate_valid与converged/reliable不是一个量，15D BE回放另要求converged。实际部署选择哪个节点/参数/二进制仍UNKNOWN。

CONFIRMED：fillFitnessCovariance构造diag(f,f,f,0.5f,0.5f,0.5f)，CUDA发布有f>=1e−5下限。pcl_fit.cu是在变换后的Source点对原Target raw points做有界扩壳最近邻平方距离；找到邻居才进分母，miss不计均值，最大shell命中可能没有证明全局最近邻。不是NDT概率/score，不保证与无界PCL完全相同。f量纲m²，0.5f用于rad²没有测角残差依据；miss比例/搜索完整性需单独诊断。

CONFIRMED：生产ekf_localizer.cpp:510–578检查六个对角范围及cov[0]>=3分档，只改对角；measurement.cpp:37–45提取XY/Yaw含交叉项再整体×smoothing_steps。对Legacy对角矩阵可复现旧行为；对Native有交叉项时仅改对角不等于D R Dᵀ，可能破坏SPD/相关结构。原六状态不在BE5改造范围。

CONFIRMED：BE run_loop.py:136–156调用engineering_R(f)，以已有分档及×5重建完整6D对角，平移三轴同xy，旋转三轴同yaw。BE基线不是未经缩放的发布Legacy公式，不能拿另一个Legacy当唯一E0。ExternalPose有接纳域检查及全P融合，但无额外生产分档。已有Native助手不是BE主循环实际调用。

## Native确切公式

CONFIRMED：finalQualityKernel（ndt_linearize.cu:285–365）在最终位姿为每个Source选择允许邻域内Mahalanobis距离最小的Gaussian。q=min residualᵀLambda residual，不是优化器多邻居score Hessian。其inlier阈值tau=chi²_3,99%，与优化过程q50门控不同：

I=(1/N) sum_{best关联且q<tau} Jᵀ Lambda J；
s_res=(1/(3N))[sum_{关联} min(q,tau)+N_miss tau]。

所以Native的residual_scale已含截断残差和miss惩罚，信息只由inlier贡献。source_count分母包含全部Source；点数/密度变化对量纲及有效样本数的影响不是简单1/N统计平均。将相同点重复一倍时，I和s_res不变，输出C也基本不变，不能直接当“2倍独立数据的MLE协方差”。

CONFIRMED：ndt_gpu_matcher.cpp:301–350对I对称特征分解，拒绝明显负值，lambda_floor默认1e−6，C=s_cov s_res V diag(1/max(lambda,lambda_floor)) Vᵀ。目标Gaussian covariance另有相对特征值地板，两种地板不同。

该C是best-voxel局部GN/Fisher-like近似+经验尺度，不是精确NDT Objective Hessian逆，也没有建模地图系统误差、错误关联、动态目标、重复结构多峰、逐帧/Deskew相关性。信息平均化及截断使绝对统计尺度必须验证。小C不能证明匹配分支正确。

CONFIRMED：质量valid检查有限等条件，nativeCovarianceUsable仅有限/近似对称/正对角，不构成一般6×6 SPD证明。s_res=0时信息地板也不能保证C正定；研究入口仍应严格Cholesky/特征值检查。禁止静默将坏矩阵对角化掩盖问题。

## 六维参数、坐标和交叉项

CONFIRMED：内部pose顺序[t_x,t_y,t_z,a,b,c]，R=Rx(a)Ry(b)Rz(c)，不是通常Rz(yaw)Ry(pitch)Rx(roll)的同名Euler协方差。右旋转误差映射

B=[Rzᵀ Ryᵀ e_x, Rzᵀ e_y, e_z]，
delta theta_body=B delta[a,b,c]。

ExternalPose创新旋转为phi=Log(R_priorᵀ R_NDT)，测量噪声到该图的Jacobian为Jr(phi)^−1 B。与平移合成M，使用R_innov=M C_internal Mᵀ，保留全部交叉项；选XY/Yaw时用真实yaw的Jacobian，不应取native(5,5)冒充yaw方差。

CONFIRMED：已有pose_uncertainty.py:7–30实现同类转换及chart一致性/Cholesky检查，测试文件存在；但native_pose_R第一行仍调用engineering_R(fitness)，会把f>15等Legacy条件变成隐藏Native门控。source_floor_R还会对角化并丢弃交叉项，不能用于BE5 Native。两者当前未被BE主循环调用；不是已经污染BE的证据。

CONFIRMED：生产NDT先将PCM位姿乘地图转换，Native C经registration/measurement_policy到发布字段是直接复制，未见对应RxRyRz→ROS角图/PCM→发布map的完整传播。若该地图旋转非单位或角度较大，会有合同风险；是否现场实际触发UNKNOWN。不可把当前ROS36项当作已验证图的Native R直接导入研究ESKF。

HYPOTHESIS：研究入口优先保留base测量，不必把位姿观测强行换到IMU原点，因为ExternalPose的p_base=p_IMU−Rr已含杆臂H。若确需转换到IMU位姿，p_I=t_B+R_B r的协方差Jacobian有−R[r]×，并伴随旋转图/坐标变换；不要对H和R重复施加杆臂。若PCM→map有旋转A，平移扰动乘A，body-right旋转在左乘固定坐标变换下不变，再转到所选创新图。不能盲目套一个不匹配参数化的SE(3) adjoint。

## 已保存数据与限制

CONFIRMED：抽读BE保存记录包含native_internal(6×6)、parameters_internal、final(4×4)、score_hessian、source_n、fit结果和实际R。私有match_p0.cpp明确native_internal来自align结果z.covariance。已可设计固定pose的Native Raw对照，不需要重新配准。

CONFIRMED：kappa_at_float_pose/inliers等来自另一次evaluateFinalQuality(source,z.final_transform)，即float输出位姿重新查询；z.covariance对应内部最终double参数，二者不保证关联完全相同。不能用后者残差尺度除回前者C来声称恢复了原信息矩阵；缺失的同一线性化原始I/尺度/计数须标UNKNOWN或未来受控导出。服务二进制与源码精确重建等价性仍UNKNOWN。

## 是否保留Fitness：明确结论

HYPOTHESIS（建议）：Native若经正确图转换及独立统计标定，不需要Fitness再参与协方差生成；Native Raw禁止再乘Fitness。当前s_res已表达另一种残差尺度，Fitness可能反映原始NN几何误差，但“不同定义”不等于有独立新增信息。无独立条件统计证据，禁止Native×Fitness经验混合。

CONFIRMED：Fitness当前仍被Legacy接纳范围/基线用到，因此不能直接全局关闭；先保留为独立日志和Legacy回滚依据，不作正确分支标签/单一硬门控。拟议R模式要解除隐藏Fitness依赖，保持匹配接纳合同在三个R模式间一致。

CONFIRMED：GPU fitness在align后独立评估，另有raw-point索引构建/常驻显存、shell搜索和同步开销；result有pcl_fit_ms。UNKNOWN：关闭后实际节省多少GPU/端到端时间，BE保存的quality_extra_ms是额外质量重评，不是PCL计时，不能替代测量。未来需固定pose、仅质量旁路ON/OFF并分启动/常驻/稳态，单独申请小样本验证，不调整优化器。

## 标定与NO-GO边界

HYPOTHESIS：Native Calibrated只允许事先定义的低维正尺度或完整D C Dᵀ，D在统一物理图中有明确单位，保留相关性；先固定标量更易辨识，不拟合任意6×6黑箱。根据可追溯独立误差或条件创新模型，以logdet S+rᵀS^−1r、方向覆盖及时间相关性评价；绝不以最低75ms RMSE选scale。训练/验证需分离连续窗口/独立记录，三圈不等于三次独立采集。

UNKNOWN：现有工程GNSS/INS/map/初始化误差相关性和Z/RP参考充分性不足，无法认证绝对6D统计尺度；创新含P误差和先验-观测相关性，NIS只能条件诊断，不可单独辨识P与R。没有独立真值就不报完整15D NEES；缺失Bias真值不以置零代替。Native Calibrated当前为待证候选，不是已完成或生产替换依据。

混合米/弧度原6×6条件数/地板随单位而变，应以预定L归一化联合XY-Yaw和完整尺度图诊断，保留耦合，不因“条件数大”整帧拒绝。独立空间覆盖、miss/inlier与异常分支另作诊断，不能揉进万能健康度分数。
