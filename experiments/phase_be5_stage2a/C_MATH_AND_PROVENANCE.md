# C：S² tilt数学及来源限制

CONFIRMED：新增attitude_reference.py仅提供残差、H和来源诊断。没有AHRS/VectorNav订阅、真实姿态加载、R标定或融合接口。

u=Rᵀk，u_ref=R_refᵀk，E_ref为u_ref的正交切空间；
r=E_refᵀ(u_ref−u)，H_theta=E_refᵀ[u]×。由于测试差分的是残差，dr/d(delta theta)=−H；测试没有混淆符号。

16个合成参考状态对全部15列、三个步长差分，最大绝对误差1.66597824e-09。yaw不可观方向零空间、共同map-yaw旋转不变性、非零真实tilt、相反半球、坏参考和来源风险通过测试。合成参考只用于单元数学，不当成真实传感器参考。

CONFIRMED：当前数据没有合格独立姿态参考，因此55案例C全部阻止真实融合；未用当前R作“参考”伪造零残差，未以自Deskew/NDT/VectorNav离线评价姿态填实验矩阵。C实数据r/R/K/P候选没有被伪造。

来源诊断对Bosch复用、内部GNSS/INS、自身Deskew以及缺失时序/协方差合同均标相关性UNKNOWN；即使声明来源完整，Stage2A仍fusion_allowed=false。原生产AHRS源切换和逐帧来源不确定性见Stage1，不因数学测试而消失。

不做Euler直接相减、不锁定RP=0。重力方向会被赛车动态加速度污染，范数接近g不足证明可靠。即使未来可信tilt能改善部分重力投影，也不直接观测高度/速度，不能承诺根治Z/Vz。
