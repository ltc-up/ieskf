"""Write self-contained, de-identified reports from preserved aggregate evidence."""
import json,sys,hashlib
from pathlib import Path
p=Path(__file__).resolve().parent;root=Path(sys.argv[1]);d=json.load(open(p/'aggregate_results.json'))
R={(r['series'],r['E']):r for r in d['results']};F={r['id']:r for r in d['performance']};O={r['id']:r for r in d['optimization']}
links='''[Koide原文Sec.III](https://arxiv.org/html/2402.05540v1#S3)、[作者项目页](https://staff.aist.go.jp/k.koide/projects/icra2024_gl/)、[原生VGICP](https://github.com/koide3/gtsam_points/blob/8e94654558dfbe7ddb0341de300daaf17e25ee02/include/gtsam_points/factors/impl/integrated_vgicp_factor_impl.hpp)、[原生LM扩展](https://github.com/koide3/gtsam_points/blob/8e94654558dfbe7ddb0341de300daaf17e25ee02/src/gtsam_points/optimizers/levenberg_marquardt_ext.cpp)。'''
common='''本报告属于KG-S2有限CPU固定滞后Shadow研究。未修改生产、main、NDT、15D核心、地图或Q/Bias；未运行全圈/三圈。所有精度数值使用既有GNSS/INS及天线到base转换的工程参考，不是独立真值。完整原始输入、状态、P/J/H、轨迹和消息身份留在授权私有卷，公开文件仅含自编源码及脱敏标量。CONFIRMED指所述具体数值/实现事实，不等于定位或统计认证。\n'''
def write(name,body): (p/name).write_text('# '+name.removesuffix('.md').replace('_',' ')+'\n\n'+common+'\n'+body.strip()+'\n')
def table(lag='.075'):
 lag='0.075'if lag=='.075'else lag
 t='|有限窗口|方案|有效帧|XY RMSE m|横向绝对P95 m|Yaw RMSE °|\n|---|---|---:|---:|---:|---:|\n'
 for label in ['fault','normal8','right3','ordinary','large_proxy','pass713']:
  for E in range(5):
   z=R[(label,E)]['by_lag'][lag];t+=f"|{label}|E{E}|{z['n']}|{z['XY_RMSE']:.3f}|{z['lateral_abs']['p95']:.3f}|{z['yaw_RMSE_deg']:.3f}|\n"
 return t
write('KOIDE_NATIVE_FACTOR_IMPLEMENTATION_REVIEW.md',f'''
## 实际复现边界

**CONFIRMED**：Koide等ICRA2024将固定地图配准、分别连接历史扫描的配准、IMU预积分和历史状态边缘化放在同一目标中。原论文已知地图定位程序与GLIM不是同一个公开程序；本轮未运行论文完整闭源实现。{links}

**CONFIRMED**：本轮构建并实际链接gtsam_points `8e94654558dfbe7ddb0341de300daaf17e25ee02` CPU库，GTSAM4.2。使用原生`IntegratedVGICPFactor`：残差为目标体素高斯均值与变换后Source均值之差，权重为两局部协方差旋转合成的逆；对应在每次linearize更新，LM试探期间使用缓存的对应/权重。Target体素储存平均局部协方差，不是体素散布协方差。native KNN=20、特征值正则(.001,1,1)、体素1m。本轮不是Stage1的.5m公式诊断的等价复现，也不是DIRECT7 NDT目标。

没有把独立配准结果包装成PoseFactor。E1/E3/E4直接把原始几何H/g及源/目标交叉项送入非线性状态优化。Scan-to-Scan最多分别连接前3帧，未拼接Target。

## 必须处理的原生集成细节

**CONFIRMED**：原生`error()`返回ΣrᵀWr，而原生HessianFactor采用0.5二次型。研究适配器仅把非线性cost归一化为0.5ΣrᵀWr，保留原生H/g/交叉项。合成固定对应和固定权重下有限差分吻合；这不声称包含协方差旋转导数的精确Objective Hessian。

**CONFIRMED**：原生LMExt在线性化更新对应后重算当前cost。普通GTSAM BatchFixedLag LM不会保证同样语义，历史单因子会走一步后因旧cost而过早停止。保留`results/`和`verified/`集成对照，正式结论只用`native_verified/`。原生LMExt + GTSAM CalculateMarginalFactors构成正式滚动图；没有修改第三方源码。

## GLIM与Holistic Fusion

GLIM审查固定`1a197c999e4865f5cab7c9cd25474128413ebbca`。[GPU因子构造](https://github.com/koide3/glim/blob/1a197c999e4865f5cab7c9cd25474128413ebbca/src/glim/odometry/odometry_estimation_gpu.cpp)与[IMU链](https://github.com/koide3/glim/blob/1a197c999e4865f5cab7c9cd25474128413ebbca/src/glim/odometry/odometry_estimation_imu.cpp)用于原理对照。OVERLAP关键帧管理不等于固定最近帧；本轮保持固定1–3帧，以隔离因子本身，未声称新验证了OVERLAP优越性。

Holistic Fusion固定`6e2b3a8841d022552f83cfa90149ff2044645723`。[ImuBuffer](https://github.com/leggedrobotics/holistic_fusion/blob/6e2b3a8841d022552f83cfa90149ff2044645723/holistic_fusion/src/lib/ImuBuffer.cpp)可从优化状态以实际IMU与Bias传播至新时间；[GraphManager](https://github.com/leggedrobotics/holistic_fusion/blob/6e2b3a8841d022552f83cfa90149ff2044645723/holistic_fusion/src/lib/GraphManager.cpp)区分预积分缓存与优化。借鉴的是低频优化和高频传播分离；未安装或运行HF，更未声称其默认使用本轮原始点云因子。

**UNKNOWN**：论文GPU性能、原闭源程序全部参数和跨场景鲁棒性没有复现。CPU结果不能提前外推GPU50ms通过。
''')
gtable='|地图|案例数|有效体素支持比例 min/median/max|支持中大白化残差median|横向条件信息median|消去其它位姿分量后的横向信息median|\n|---|---:|---|---:|---:|---:|\n'
for r in d['geometry']:
 s=r['support_fraction'];gtable+=f"|{r['variant']}|{r['n']}|{s['min']:.1%}/{s['p50']:.1%}/{s['max']:.1%}|{r['large_given_supported']['p50']:.1%}|{r['conditional_lateral_information']['p50']:.1f}|{r['nuisance_marginal_lateral_information']['p50']:.1f}|\n"
write('LOOP44_1013_GEOMETRY_EVIDENCE.md',f'''
## 数据恢复与空间证据

**CONFIRMED**：515–520s恢复100条三LiDAR融合Source；300条原始CDR身份、Header/可用时间、原15D事件重放状态hash、Seed及Source字节均核对原C5。没有复制整包。当前Source来自C5 q50自身15D Deskew，旧q11生产Source单列；时间相差约扫描内参考偏移，禁止冒充同帧旧故障配对。

原生逐点诊断共59项：16个冻结因果Source×三地图，另11项旧Source/Seed/旧最终候选诊断。三个地图只读复用。旧1013补结构图为既有局部Target，不是完整全图；视场不同可能影响覆盖，不能将16项所有差异都归因于帐篷结构。旧同Source、同ROI三地图案例单独保存，未按评分编辑地图。

{gtable}

这些信息值使用预设局部高斯形状和L=10m尺度，单位/绝对标定尚未确认，不能当作统计置信度或匹配正确概率。完整6×6交叉项、XY–Yaw投影和消去其余位姿分量的Schur结果分别保留。前者固定其它分量，后者不是同一个量。

**CONFIRMED**：高整帧Fitness与仍存在有效地图约束可以共存。1013支持的横向信息并未全部消失；但median支持仅26.5%，支持中的大白化残差median约28.7%。无对应点在native因子中不贡献残差；有对应的大残差才会被Huber降权，二者不能合并解释。0105的体素支持数反而可能更少，不能以支持数直接选正确地图/分支。

逐点证据保存Source索引、对应体素ID、残差、白化残差、J/W、Huber权重和法向代理。可分析近远距离、角域、空间块及排除点；未使用GNSS选点。

**UNKNOWN**：没有可信语义标注，不能把水平/竖直法向代理硬称为地面、护栏、建筑。原Source裁剪可能已去掉低处地面，本轮没有扩大Crop。三雷达体素会混合点，最近原始Deskew点给出的owner仅代理；保存其条件H分解，不声称三份独立统计证据或真实逐射线归属。边缘化后的信息也不能逐雷达简单相加。
''')
t=d['tests'];q=d['raw_geometry_checks']
write('GEOMETRIC_FACTOR_NUMERICAL_VALIDATION.md',f'''
## 实测数值结果

|检查|最大误差/结果|
|---|---:|
|原生一元cost归一化后梯度FD|{t['normalized_gradient_fd_vs_gn_max']:.3g}|
|原生二元12列梯度FD|{t['binary_12_column_normalized_gradient_max']:.3g}|
|真实点残差Jacobian FD|{q['residual_fd_max']:.3g}|
|真实手工累计H与native H相对差|{q['H_relative_max']:.3g}|
|真实H绝对差（大矩阵量级）|{q['H_abs_max']:.3g}|
|真实g绝对差|{q['g_abs_max']:.3g}|
|Wheel全15列FD|{t['wheel_all15_fd_max']:.3g}|
|Huber固定对应/权重梯度FD|{t['Huber_fixed_cache_gradient_max']:.3g}|
|深拷贝诊断对原因子cost影响|{t['deep_clone_cost_interference']:.3g}|
|Schur边缘化mean与完整batch差|{t['marginal_mean_vs_batch_max']:.3g}|
|Schur后P与解析值差|{t['marginal_P_vs_analytic_max']:.3g}|
|非零姿态/Bias静止IMU速度误差|{t['stationary_velocity_error']:.3g}|

**CONFIRMED**：上述合成及真实残差数值检查通过。有限差分固定关联及线性化协方差；不是对包含离散对应切换的全局目标求光滑Hessian。真实最大H绝对误差发生在大坐标/信息量级下，相对误差约5e-15。

原生LM窗口21帧，有/无全P/H只读诊断最大均值差{d['diagnostic_equivalence']['mean_max_diff']:.3g}，最终P差{d['diagnostic_equivalence']['P_final_max_diff']:.3g}；预设容差1e-8通过，**不是位级相同**。诊断使用原生因子深拷贝，防止mutable对应缓存影响估计。

所有正式窗口逐步保存完整最新15×15 P及活动图稀疏H/信息向量/变量顺序，最新P均有限、对称、正定。统计标定仍未通过。仅最终P更小不说明估计更可信。

日志勘误：实际执行档案中`H_gradient`/最终H的`gradient`字段存的是GTSAM信息向量η，数学梯度为−η。保存TRACE_SCHEMA_ERRATA及原执行源码快照；公开源码已改为明确的η/梯度字段。原始数值不覆盖。原始summary fullMatrix遵循GTSAM键排序；逐步`P_X_V_B_explicit`按Pose6/Velocity3/Bias(ba,bg)6显式排块，是主分析依据。

**NO-GO**：数值通过不等于独立定位精度、相关性一致性或实时验收通过。
''')
write('FIXED_LAG_GRAPH_ARCHITECTURE.md',r'''
## 状态与参考点

**CONFIRMED**：每节点X=Pose3(map←物理IMU)、V=map下IMU原点速度、B=(ba,bg)为物理IMU轴向Bias，保留全部15维。原15D姿态body轴转到真实IMU轴；Source由base转IMU，地图保持map系。不把base后轴速度当成IMU速度。完整初始15×15 P以右误差图一阶变换，保留交叉块；仅每个窗口第一个合法检查点进入prior，后续旧15D状态不作为在线因子。

令T_BI=(R_BI,r_BI)，则R_MI=R_MB R_BI，p_MI=p_MB+R_MB r_BI；Source p_I=R_BIᵀ(p_B−r_BI)。Wheel模型为

\[
h_w=e_x^T[R_{BI}R_{MI}^Tv_{MI}-(R_{BI}(\omega_m-b_g))\times r_{BI}].
\]

只约束纵向；未增加vy=0非完整约束。沿用既有Wheel方差与slip模型；未调Q/Bias。原始Bosch比力直接用于物理IMU原点，map重力向下，没有重复补偿重力或虚构Wheel。

## 时序与因子

**CONFIRMED**：节点位于实际Wheel测量时刻及LiDAR参考时刻，真实IMU分段ZOH预积分，含姿态/速度/位置、Bias修正、协方差与Bias RW边。未把已融合LiDAR增量当作IMU观测。采用0.5s单一固定lag，没有窗口长度搜索。

E0=IMU+Wheel；E1=E0+原生一元Scan-to-Map；E2=E0+分别连接前1/2/3帧的二元VGICP；E3=两类几何；E4=仅E1地图几何加入逐对应Huber。几何不经过NDT最终位姿或Fitness R。地图Target以预先存档的因果预测裁剪，固定120m平面范围及原点相对高度范围；不随后续优化答案挑地图。Source/Deskew固定原路径，因此本轮是冻结前端的后端Shadow，不是真前端反馈闭环。

原生LMExt保持对应更新与cost同步；GTSAM CalculateMarginalFactors在涉及离窗变量的因子子图上做Schur消元，留下LinearContainer边缘先验，删除旧变量。不是直接丢弃历史信息。线性化边缘先验不能恢复被边缘化变量的完整非线性信息，未实现原GLIM全部FEJ/关键帧策略。

每步记录所有活动节点状态、初值、因子ID、H/g、最新完整P、Bias、迭代、边缘化和计算时间。配置maxIterations=8，但原生扩展的0开始计数可产生9个外层步骤，实际次数如实保存；到上限仅代表预算截断，不叫converged。没有通过增加预算追分。

## 因果性与相关性边界

**CONFIRMED**：fault及四个BE子窗口均按冻结Header/原始可用时间处理；只使用当时已经可用且测量时间不晚于节点的运动输入。优化后的旧窗口节点另存retrospective，不能冒充它们当年的在线输出。最新节点输出仍在扫描时刻，不是已经传播到完成时刻的100Hz控制状态。

713另一次经过没有原LiDAR recorder可用时刻及成熟完整15D检查点。仅做明确标记的条件化离线窗口：初始均值来自原因果Seed，初始速度来自过去Wheel及杆臂、零横向初始假设，Bias从模型零初值、P用既有初始配置。它不是恢复的成熟15D，也不是到达时序合格的因果运行；数据释放时间只作离线输入批次下界。

**UNKNOWN**：原Deskew和Target裁剪共享旧地图反馈，Scan-to-Map/Scan-to-Scan共享当前Source，多边还共享历史扫描与地图。未完整建模跨因子/跨帧相关性，未加任意降权来包装一致性。E3中P进一步收缩但精度未稳定改善，不能将这些因子视为独立额外观测。
''')
write('JOINT_OPTIMIZATION_ABLATION.md',f'''
## 正式实际执行

**CONFIRMED**：原生LMExt共30个有限配置：fault100帧、normal8/right3/ordinary/large_proxy各21帧、pass713条件化离线30帧，每个执行E0–E4。共1070条LiDAR节点决策，不是整圈；评分区间只是原normal8/right3中的约1s子窗口，不能当作完整原路段P95。首批515–520s故障前缀连续，但初始点是合法C5检查点，不是从Pit重新执行前端。

Huber只选一种：E4在E1地图几何逐对应白化3维残差上应用δ=2.795，无鲁棒核/尺度搜索。其χ²3参考分位点不是传感器统计标定。IMU/Wheel与其它参数在各组相同。

75ms工程参考：

{table()}

**CONFIRMED**：fault E1横向P95改善约19.5%，XY RMSE却恶化约8.3%。E3没有稳定超过E1。713条件化窗口E2明显退化，而E1/E3保留绝对地图锚点后较稳定。normal8、right3的横向有局部收益，但Yaw及其它指标并非同时改善，不能称全系统胜出。large_proxy保存的原NDT大修正最高仅约0.786m，不是2m独立正确案例。

与相同初始mean/fullP的原15D纯IMU+Wheel预测对照：fault XY RMSE约1.040m、横向P95约1.270m，图E0约1.046/1.282m；没有证明“改成联合优化本身”改善纯运动预测。其它1s窗口两者基本相近。采样/离散化及Wheel更新时间略有差别，未把坐标表达变化视作新信息。

**UNKNOWN**：没有完成同前端实时反馈的Legacy15D与新图A/B。原C5/BE Legacy轨迹保留作历史对照，它们有各自NDTPose反馈；不能把该轨迹和冻结Source新图差异完全归因为后端。不能声称已解决错峰或已获得独立真值精度认证。

## 0/75/150ms敏感性

'''+''.join(f"- {label} E1 XY RMSE："+'/'.join(f"{R[(label,1)]['by_lag'][lag]['XY_RMSE']:.3f}"for lag in ['0.0','0.075','0.15'])+' m（0/75/150ms）。\n'for label in ['fault','normal8','right3','pass713'])+'''
完整30×3指标、Yaw/Z及纵横向分布见aggregate_results.json。参考时移没有用于在线选因子、初值、对应、鲁棒核或参数。

## 保留的失败

普通Batch LM对应缓存/cost集成问题、第一次诊断clone接口缺失、统计序列化问题以及缺少实际Legacy R的拒绝帧均保留。前两批结果不是最终原生LM证据。所有Native预算截断和不一致改善都保留，未删错误帧。
''')
write('HIGH_FITNESS_AND_WRONG_BASIN_ANALYSIS.md','''
## 正确附近的高Fitness和大修正

**CONFIRMED**：历史高Fitness约20.868案例，原Seed与旧“正确附近”1013候选相差约2.094m。本轮原生单地图因子从原Seed出发，接受的局部横向修正约1.984m，最终距离该旧候选约0.064m。普通LM缓存集成对照仅走约0.30m横向、距旧候选约1.724m；这是原生优化器合同的重要实证。

这证明原始几何因子能利用地图里保留的信息作大修正，**没有用该候选或工程参考初始化优化**。单因子之外，另执行旧五Source条件化联合窗口：E1/E3首帧横向修正约1.987m、距旧候选约0.064m；E4约1.953m、约0.022m。见OLD_BRANCH_CONDITIONAL_JOINT。它用明确的模型初始P/速度/Bias，不是恢复历史成熟状态，不能外推所有窗口必然保留大修正。旧“正确附近”依据是0105同帧工程对照，不是真值；采用现有75ms GNSS/INS合同，该案例候选仍约1.342m XY偏差。两类工程参考不能混为绝对正确认证。

## 错误分支仍可能自洽

**CONFIRMED**：首次错峰旧Source在旧Seed处原生支持404/2499，在旧错误最终位姿处974/2499；错误分支仍有正的局部约束。不能以正定H或支持较多宣布它正确。另一个错误持续样本旧Seed支持530/2331，旧错误最终位姿1052/2331；地图/关联改变会同时改变cost和参与点数，cost更低也不证明正确。

1013高整帧Fitness同时包含缺失覆盖、错误关联和大残差，local残差中仍存在有效横向/Yaw信息。无对应在native直接缺席；Huber只能压低有对应的大白化残差。错误峰若由低残差的重复结构支撑，Huber不能识别分支语义。当前δ仅模型级选择，没有独立尺度标定。

**CONFIRMED**：fault E4没有优于E1全部精度指标；713 E4也没有稳定优于E1。E3让fault最终P trace约从E1的0.00487压到0.00226，XY仅由1.133到1.078m，横向P95反从1.032升至1.106m；这是相关性/模型误差风险证据，不是更可信的定位。

**HYPOTHESIS**：S2S在重复结构中能保留相对一致但绝对错误的运动，并通过共享扫描重复计数增强自洽性。713 E2退化支持风险存在，但不能仅凭最终误差断定唯一原因是重复护栏。

与Phase BB的联系：没有重新生成独立相对位姿测速，也未声称重现同一65m/s案例；本轮joint S2S仍未消除BB揭示的“收敛/正定/局部支持不能证明正确、共享Deskew/Seed不独立”问题。[BB失败记录](https://github.com/ltc-up/ieskf/blob/7cba238/experiments/phase_bb_lidar_velocity/FAILURES.md)。

**UNKNOWN**：尚无可靠的在线分支判别；没有测得正确大修正的总体误拒率或错误分支漏接纳率。当前结果不能为生产设计新的硬门限。
''')
write('LEGACY_NATIVE_GEOMETRIC_R_COMPARISON.md','''
## 三条路线的实际程度

|路线|本轮实际完成|资格|
|---|---|---|
|A Legacy15D位姿融合|复用C5/BE状态及原R/K/δ，另做同初值原15D纯预测|历史工程对照，未重跑前端闭环|
|B 几何信息压缩成位姿R|16个冻结Prior中9个有真实原R完成完整R/K/P候选对照；7个缺实际更新R明确排除|条件曲率代数，不是已标定新R|
|C 原始几何固定滞后图|30个原生联合优化配置，真实IMU/Wheel/几何/边缘化|有限后端Shadow，不是生产闭环|

**CONFIRMED**：B对固定Source/位姿线性化得到H，保留全部6×6交叉项，使用完整坐标图及杆臂变换M H⁻¹ Mᵀ到ExternalPose创新图。完整P、K、δ、Joseph/reset候选保留在私有卷。没有只改对角或乘Fitness。9个有效曲率R全部含非对角项。B命名为conditional inverse curvature，不称统计协方差，不提交状态。

**UNKNOWN**：局部协方差特征值正则、对应选择、未匹配点、采样密度、空间相关和地图误差共同影响尺度。不能把H⁻¹、旧Native Raw或因子P当作匹配正确性概率。没有独立真值支持绝对标定、NEES或跨场景NIS一致性；参考相关性也未解决。

C能对活动位姿重新关联/重线性化，地图因子与速度/Bias经IMU跨时间耦合，信息没有被先压缩成一次完整位姿。它仍可能出现错误关联、退化、重复信息及过度自信，不能因“直接几何”而自动跳过统计建模。

**决策**：停止Fitness→全部方向R的大规模倍率搜索。暂不部署B；C只保留研究资格，尚未证明比轻量化且正确标定的B具有稳定实际收益。Legacy保留历史基线，不代表它的尺度已正确。两条新路线都需要地图模型误差/相关性证据，不能用更小P替代。
''')
perf='|fault方案|总step P50/P95/P99/max ms|优化P50 ms|边缘化P50 ms|50ms超时|\n|---|---|---:|---:|---:|\n'
for E in range(5):
 r=F[f'fault_E{E}'];s=r['step'];perf+=f"|E{E}|"+'/'.join(f"{s[k]:.2f}"for k in ['p50','p95','p99','max'])+f"|{r['optimization']['p50']:.2f}|{r['marginalization']['p50']:.2f}|{r['over50']}/{r['n']}|\n"
write('REALTIME_FEASIBILITY.md',f'''
## 实测CPU窗口成本

{perf}

**CONFIRMED**：固定lag0.5s，fault最多61个15D活动节点，累计534节点被边缘化。RSS峰值约{max(r['RSS_MiB']for r in d['performance']):.1f}MiB。没有构建GPU几何因子，VGICP优化无GPU上传；重建Source曾使用既有CUDA Deskew，不能把它算成新几何GPU性能。

step包含逐Source读取/物理IMU坐标变换、协方差估计、Target裁剪及KNN/体素构建、IMU预积分、Wheel/几何因子构造、非线性优化、Schur边缘化和必要状态记录；一次全图读取另记map_read。逐窗口全H/P重算及大日志作为独立诊断开销，不包含在step列，完整离线墙钟包含它们。这个区分不会使E1/E3/E4通过50ms；fault全部100帧超时。

原生迭代trace保存线性化、求解及lambda trial时间；对应搜索包含在几何linearize内，未获得可无重叠相加的每个KNN/对应子阶段独立计时，明确UNKNOWN。CPU墙钟不是CUDA event；并行归约、缓存及研究机其它任务会影响波动，713与部分独立诊断有共享研究负载，不当作严格隔离性能赛跑。没有ROS/DDS或完整车辆端到端验收。

**NO-GO**：当前CPU原型20Hz/50ms。不能以E0或部分E2较快声称整套几何联合系统通过。P50/P95/P99仅有限样本描述，不是生产保证。

## 100Hz后续设计（未实施）

保留现有事件驱动IMU核与100Hz只读输出原型。若未来允许接入图解，应将某一优化时间的mean/P/Bias原子版本交回状态历史，再用实际已可用IMU传播至当前，保留状态时间/年龄/修订号；旧窗口平滑结果不直接改时间戳当作当前状态。此处未接控制状态，未验证这种协方差回注与Deskew闭环。

工程优化可研究缓存Target高斯、减少重复分配、原生GPU因子与求解解耦；**HYPOTHESIS**为可能节省时间，不是本轮已有50ms证据。不得先靠加权/删点追速度而隐去信息损失。
''')
write('NEXT_GENERATION_LOCALIZATION_DECISION.md','''
## 推荐与停止范围

**CONFIRMED / CONDITIONAL**：已完成真正原生几何因子+IMU+Wheel小型固定滞后联合估计，数值/原生集成和边缘化基本成立。保留这一独立研究实现，保留原15D惯性、Wheel、完整P和事件回滚作为对照。

**NO-GO**：当前版本直接进入1013连续前端反馈闭环或生产替换。故障窗口XY恶化、S2S另一经过显著退化、Huber无跨场景稳定收益、CPU超预算，且相关性及统计尺度仍不合格。不是说Koide论文整体无效，而是本轮具体实现及数据未满足条件。

优先保留简单E1作为下一次机制诊断对象，E3不设默认。下一轮若申请，应先冻结少量真实错误分支、共同合法前史及独立验证点，检查对应覆盖、未匹配点模型、共享Source相关性及方向性更新；仅在这类证据改善后再设计连续反馈。不继续大网格核/尺度/窗口搜索。

路线B仍可作为轻量备选，但必须先解决误差模型与标定，不能直接部署H⁻¹。几何因子能重新线性化，不表示它们提供了新独立传感器。

目前缺少的信息：①对重复结构可区分的空间证据；②缺失地图与动态物体的可靠来源/占用可见性；③不被同一LiDAR地图反馈污染的运动锚点；④可信时间/外参和独立参考以验证统计尺度。更长同类局部扫描并不必然提供这些信息。

本轮到此停止，没有自动执行连续闭环、100Hz状态替换或生产改动。
''')
write('FINAL_CONCLUSION.md','''
## 决策

**原生小窗口实现：CONFIRMED。进入连续闭环及生产：NO-GO。**

本轮实际完成30个原生CPU有限联合优化配置、100帧故障连续短历史恢复、59项原始几何诊断及合成/实数据数值检查。不是PoseFactor包装，也不是只写公式。

|研究问题|证据与结论|
|---|---|
|1. 高Fitness还有多少有效信息？|CONFIRMED：16个Source在1013体素支持17.3–91.8%，median26.5%；局部横向/Yaw信息仍在。不是匹配正确概率。|
|2. 不用Fitness R能否改善？|CONFIRMED：E1在fault横向P95 1.282→1.032m，但XY 1.046→1.133m；收益不全面。|
|3. 联合IMU/Wheel是否胜过原15D？|UNKNOWN：E0与同初值原15D纯预测基本相近；新图没有与原前端反馈做严格闭环胜负验证。|
|4. S2S新增信息多少？|CONFIRMED：E3未稳定优于E1；713条件化E2 XY恶化至约3.319m。独立信息量未标定。|
|5. 地图缺失能否由鲁棒处理解决？|CONFIRMED：Huber能降低有对应大残差；不处理无对应，也未带来跨场景稳定优势。|
|6. 错峰仍有强虚假几何？|CONFIRMED：旧错误最终位姿支持更多、局部H仍正；无可靠分支判别证书。|
|7. 能保留约2m大修正？|CONFIRMED：历史单地图原生因子横向修正约1.984m，距旧正确附近候选0.064m；CONFIRMED：补充五帧联合窗口E1/E3仍约1.987m；UNKNOWN：独立物理正确性及成熟前史。|
|8. normal8/right3是否退化？|CONFIRMED：局部横向有收益，Yaw/其它指标仍可能变差；只测1s子窗口，非完整评分段。|
|9. 几何R还是直接几何？|UNKNOWN：B仅9个有效未标定固定Prior候选，C可重新线性化但更贵；未证明C稳定胜出。|
|10. 最小窗口开销？|CONFIRMED：0.5s，fault E1 step P50约101ms、P95约118ms，100/100超50ms。|
|11. 进入连续闭环？|NO-GO：当前精度、错误保护、相关性和CPU成本均不充分。|
|12. 还缺什么独立信息？|UNKNOWN：需要可靠空间可见性/结构差异、独立短时运动证据及统计标定；不能靠更多相同点/低cost/正定H替代。|

0/75/150ms敏感性、失败和全部有限配置见JOINT_OPTIMIZATION_ABLATION及aggregate_results.json。主要结论不是“几何因子没有价值”，而是它保留局部有效约束的同时仍会受到关联、地图缺失及相关信息的影响，尚不能独立防止错误分支主导状态。

没有把CPU数值通过、P收缩或局部横向改善解释成系统定位成功。生产、main及历史实验保持原状；限定阶段结束后停止。
''')
write('REPRODUCE.md','''
## 身份与先决条件

研究baseline d418fa2c5c996ec859e310eb2321d8abfc47fe6c；原生依赖commit见manifest。需要获得授权的私有输入/外参/历史C5/BE身份清单，公开仓库不包含地图、bag、源点云、轨迹、原始P或设备参数。不能仅凭公开报告精确复算全部实数据。

所有输出、TMPDIR、ROS_LOG_DIR和构建缓存指向已挂载研究盘；先检查lsblk/findmnt/df及Git。不能恢复已清理重复点云，也不能下载新数据或变动生产。

1. 只读检查已有环境，使用已安装GTSAM4.2、Eigen/TBB/Boost、ROS Jazzy。gtsam_points CPU配置BUILD_WITH_CUDA=OFF、OpenMP ON，只编译2路，不重建GTSAM。
2. 按私有路径映射运行recover_inputs.py（需原生Deskew服务及ROS类型）。它会核对CDR/状态/Seed/Source身份并在失败时停止；已有inputs应直接复用，禁止覆盖。
3. CMake配置native目录，传入POINTS_SOURCE/POINTS_BUILD；构建kg_s2、kg_diag、kg_tests。运行kg_tests OUTPUT，确认原生数值/边缘化测试。
4. 冻结series.json、0.5s lag、1m voxel、knn20、Huberδ2.795和所有初值。执行run_matrix.py PRIVATE_ROOT NEW_OUTPUT_NAME；不得覆盖原native_verified。713必须使用独立的条件化离线配置及资格说明，不冒充成熟15D连续前史。
5. prepare_diagnostics.py生成同帧原生诊断计划；kg_diag CONFIG PLAN OUTPUT。conditional_R.py仅生成候选，禁止提交。原始拒绝帧缺R应保留原因，不能自动补Legacy R。
6. 优化结束后才能执行evaluate.py ROOT EXISTING_ROOT OUTPUT_NAME；之后aggregate.py生成脱敏统计。GNSS/INS代码不被优化程序导入。
7. 检查P有限/对称/PSD、原生因子缓存深拷贝数值等价、真实迭代预算与状态时间。应用私有TRACE_SCHEMA_ERRATA：旧H_gradient是η，数学梯度为−η；权威P使用显式Pose/V/Bias顺序。

实际运行分三批：初始Batch LM、只读隔离Batch控制、最终原生LMExt。只有native_verified是主结论；其它及两次clone诊断失败保留。源码勘误只修字段语义，原执行源码SHA快照与二进制SHA另存。多线程数值回归容差1e-8，不保证位级重放。

本轮未执行：GPU几何、完整相关性建模/尺度标定、全圈/三圈、真实前端反馈闭环、100Hz控制接入、生产替换。
''')
write('RESOURCE_AND_FAILURE_AUDIT.md','''
**CONFIRMED**：原始bag、全局地图和历史结果只读复用。新增原始重建仅100帧必要Source及小控制窗口，没有复制整包/全地图，没有恢复重复点云，没有下载数据集或Docker。CPU依赖复用本机GTSAM，仅下载小型官方gtsam_points源码及Holistic源码元数据。

存储/源码/二进制/输入SHA清单在私有卷。资源计划为新增6GiB、软件下载128MiB、最多2路编译；实际占用及盘剩余量见私有STORAGE_AND_GIT_AUDIT，远低于预算。原生几何CPU不使用VRAM；Deskew阶段GPU精确瞬时峰值UNKNOWN。

保留失败：普通Batch LM缓存cost不兼容；首版诊断缺自定义因子clone；统计JSON的numpy标量序列化错误；7个候选缺实际Legacy R；所有达到LM预算的窗口；S2S及鲁棒方案的负面结果。部分初版脚本异常未持久化独立stdout，但原失败前缀、日志和工具审计仍在；没有将其计作成功定位。

真实100Hz/车辆端到端未执行。713原始到达时序及成熟P缺失，所以其结果仅条件化离线；没有伪造传感器、真值或到达时间。
''')
# Separate private originals with complete artifact paths, no private payload staged.
pr=root/'reports';pr.mkdir(exist_ok=True)
for file in p.glob('*.md'):
 body=file.read_text();append=f'''\n\n## 本机证据位置（私有）\n\n实际根目录：{root}\n\n- 原始身份/配置及Source：{root}/inputs/\n- 正式原生联合优化、逐步P/H、状态：{root}/native_verified/\n- 两批集成对照：{root}/results/、{root}/verified/\n- 原生几何逐点残差/J/W及历史候选：{root}/results/native_raw_geometry.jsonl\n- 原始日志和失败：{root}/logs/\n- 原执行源码和有界scratch：{root}/runtime/\n- 原生依赖：{root}/dependencies/\n- 编译产物：{root}/build/\n- 源码参考版本记录：{root}/reference_review/\n\n这些目录不能被公开摘要替代；实数据精确复算需要授权原始输入。\n'''
 (pr/file.name).write_text(body+append)
print('reports',len(list(p.glob('*.md'))))
