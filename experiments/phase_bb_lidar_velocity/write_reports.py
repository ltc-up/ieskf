"""Create self-contained result tables from actual aggregates, not requested counts."""
import json
from pathlib import Path
p=Path(__file__).parent;d=json.loads((p/'results/aggregate.json').read_text());runs=d['runs'];lines=['# BB实际测速与闭环结果','', '**整体修复FAILED。**所有精度数字是冻结GNSS工程参考差异，不是认证真值。测速进入运行链的方式是下一帧NDT平移初值辅助，没有新增独立速度Kalman观测。','', '## 实际执行','', '| 运行 | 实际帧数/全部NDT接纳 | 条件性测速可用帧 | 初值辅助帧 |','|---|---:|---:|---:|']
for n,r in runs.items():lines.append(f"| {n} | {r['processed_frames']}/{r['accepted']} | {r['velocity_qualified']} | {r['auxiliary_applied']} |")
lines+=['','P40/M40/R40各40帧；M40关闭辅助时全部seed/P/NDT/delta与AY最大差异0。P_final三圈前1468帧与P单圈seed/P/NDT/K/delta最大差0。连续三圈206.65s，无重置；每个单圈73.35s。P/R都使用自己的原始三LiDAR、真实Deskew、自己的IMU/光学/轮速、自己的相对配准初值和实际CUDA-NDT/完整15D历史反馈。L为10Hz测速单圈成本消融，未执行L三圈。没有把固定历史匹配当新候选反馈。','', '## 同1197有效样本、固定75ms','', '| 方案 | XY RMSE | 右3横向P95 | 右9 | 正常5 | 正常8 |','|---|---:|---:|---:|---:|---:|']
def table(name,v,key='candidate'):
 c=v['corners'];return f"| {name} | {v[key]['XY']['rmse']:.6f} | "+' | '.join(f"{c[i][key]['abs_lateral']['p95']:.6f}" for i in [3,9,5,8])+' |'
lines.append(table('原版',runs['P_lap']['sensitivity']['0.075'],'baseline'))
for n,v in [('AY',d['AY']),('BA最佳B',d['BA']),('BB-P',runs['P_lap']),('BB-L10Hz',runs['L_lap']),('BB-R反馈运输',runs['R_lap'])]:lines.append(table(n,v['sensitivity']['0.075']))
lines+=['','单位m。全部0–9段的纵/横向P50/P95/max、XY/Yaw/Z、先验/创新/实际更新和P见聚合JSON。R比AY单圈RMSE只约0.38%更低，未解决正常8相对原版的退化；正常5还稍差。不能称统计显著或真实精度改善。','', '![全部固定分段](results/corners.png)','', '## 连续三圈与时间敏感性','', '| 圈 | 原版 | AY | BB-P | BB-R |','|---|---:|---:|---:|---:|']
for i,r in enumerate(runs['R_multilap']['laps75']):lines.append(f"| {i+1} | {r['baseline']['XY']['rmse']:.6f} | {d['AY']['laps75'][i]['candidate']['XY']['rmse']:.6f} | {runs['P_final_multilap']['laps75'][i]['candidate']['XY']['rmse']:.6f} | {r['candidate']['XY']['rmse']:.6f} |")
lines+=['','三圈扩展第一圈1204样本，不与1197公共mask混用。R各圈比AY差异只有毫米级，不能以此证明长期精度更优；只是闭环数值/匹配连续无失锁。','', '| 参考时间ms | 原版XY | AY | BB-P | BB-R |','|---|---:|---:|---:|---:|']
for k,ms in [('0.0',0),('0.075',75),('0.15',150)]:lines.append(f"| {ms} | {runs['R_lap']['sensitivity'][k]['baseline']['XY']['rmse']:.6f} | {d['AY']['sensitivity'][k]['candidate']['XY']['rmse']:.6f} | {runs['P_lap']['sensitivity'][k]['candidate']['XY']['rmse']:.6f} | {runs['R_lap']['sensitivity'][k]['candidate']['XY']['rmse']:.6f} |")
lines+=['','150ms依然使原版排序反转；未选取最优延迟。','', '![参考与连续圈](results/reference_multilap.png)','', '## 测速信息与限制','', '| 区间 | 全部扫描对 | 单对诊断合格 | 与光学X/Y差异RMS | X最大来源差异 |','|---|---:|---:|---:|---:|']
for k,dt in [('1',50),('2',100)]:
 r=d['frozen_pairs'][k];x=r['optical_difference'];lines.append(f"| {dt}ms | {r['total']} | {r['valid']} | {x['RMS'][0]:.3f}/{x['RMS'][1]:.3f}m/s | {x['max'][0]:.3f}m/s |")
s=d['source_study'];lines+=['','这不是正确率；即使H合格且收敛也有大异常。8个按传感器差异固定选取的冻结扫描对，各运行名义/±1mX/±.5mY/+.03radYaw/反向，共56次；竞争解纵向速度跨度最大约65m/s，未知哪个为正确峰。无GNSS错误峰标签，不计算检测准确率。','', '| 工况代理 | LiDAR−原始光学RMS X/Y | LiDAR−滤波光学RMS X/Y | IMU预测−原始光学RMS X/Y |','|---|---:|---:|---:|']
for k,v in s['groups'].items():
 def xy(key):return '/'.join(f'{x:.3f}' for x in v[key]['RMS'])
 lines.append(f"| {k} | {xy('lidar_minus_raw')} | {xy('lidar_minus_filtered')} | {xy('IMU_prediction_minus_raw')} |")
lines+=['','这些是同一候选条件性筛选后的区间来源差异，非独立测量误差；IMU预测已受光学融合影响，其小差异不能作为优于LiDAR的独立真值。减速由自身IMU轴向加速度<−1m/s²定义，弯道由|wz|>.1rad/s定义，不是制动器或位置标签。全时序图和完整速度留私有目录。跨帧相关统计见JSON，不能将重叠扫描对当独立观测。','', '## 初值和真实反馈','']
for n in ['P_lap','R_lap','R_multilap']:
 r=runs[n];z=r['distance_to_own_NDT_endpoint'];lines.append(f"- {n}：对实际应用辅助的帧，约{z['fraction_improved']*100:.1f}%的初值到自身最终NDT解XY距离减小；P95由{z['before']['p95']:.4f}m变为{z['after']['p95']:.4f}m。")
lines+=['','这只说明优化初值与终点的距离，不能证明终点正确；R三圈P95反而稍增，改善并不统一。新增辅助改变真实匹配/后验，但原始初值已在正常收敛域内，所以只有小数值收益，没有解决弯道估计结构。测速本身的采样噪声、有限区间动态和共享误差仍显著。','', '## 实际性能与故障','', '| 运行 | 完整周期P50/P95/P99/max(ms) | >50ms帧 | 测速前端P95(ms) |','|---|---:|---:|---:|']
for n in ['P_lap','L_lap','P_final_multilap','R_lap','R_multilap']:
 r=runs[n];lines.append(f"| {n} | "+'/'.join(f'{r["cycle_ms"][k]:.2f}' for k in ['p50','p95','p99','max'])+f" | {r['cycle_over50ms']} | {r['velocity_frontend_ms']['p95']:.2f} |")
f=d['fault_injection'];age=runs['R_multilap']['modelled_velocity_age_ms'];lines+=['',f"R三圈测速区间中点至记录处理完成的模型年龄P50/P95/max约{age['p50']:.1f}/{age['p95']:.1f}/{age['max']:.1f}ms；包含区间长度、原始到达和实际计算，不等于ROS发布时间认证。速度初值提示仅用上一对当前已完成的结果，未使用未来点云。各模块分位数不相加。",'',f"指定[20,22)s测速输出不可用注入：{f['samples']}帧、区间内合格测速{f['qualified_within']}，恢复后{f['first_qualified_after_end_s']:.6f}s首次重新取得条件性测速；完整1468帧NDT接纳。没有强制速度或伪位姿填补；Kistler/轮速/IMU/地图定位保持。独立服务管道失败和过期提示有测试；此次实际完整运行没有服务故障，不将数值测试冒称实际服务崩溃实验。",'', '![来源差异及实际时延](results/speed_latency.png)','', '234项测试通过，BB新增14项。SE3 log/exp、逆变换、完整杆臂、单位与耦合、过期/未来/差速域、原状态不变、服务退出和反馈扣除均验证。完整原估计器字节不变；Joseph/reset和15D数值资格沿用回归并由真实P对称/最小特征值复核。噪声和统计一致性仍NOT_VALIDATED。独立编译9对pose差8.88e−16、H差5.96e−7为浮点归约差异，非新算法。']
(p/'RESULTS.md').write_text('\n'.join(lines)+'\n')
