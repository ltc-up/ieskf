# KG-S2：原生几何因子固定滞后联合估计

已实际运行原生CPU VGICP + IMU预积分 + Wheel + 0.5s固定滞后图，30个有限配置及59项几何诊断。**实现数值资格通过；连续闭环/生产及50ms资格NO-GO。**

- [最终结论](FINAL_CONCLUSION.md)
- [联合优化E0–E4](JOINT_OPTIMIZATION_ABLATION.md)
- [原生集成审查](KOIDE_NATIVE_FACTOR_IMPLEMENTATION_REVIEW.md)
- [几何证据](LOOP44_1013_GEOMETRY_EVIDENCE.md)
- [数值验证](GEOMETRIC_FACTOR_NUMERICAL_VALIDATION.md)
- [状态与时间合同](FIXED_LAG_GRAPH_ARCHITECTURE.md)
- [高残差与错误盆地](HIGH_FITNESS_AND_WRONG_BASIN_ANALYSIS.md)
- [Legacy/条件几何R/直接因子](LEGACY_NATIVE_GEOMETRIC_R_COMPARISON.md)
- [计算成本](REALTIME_FEASIBILITY.md)
- [下一架构决策](NEXT_GENERATION_LOCALIZATION_DECISION.md)
- [复现与边界](REPRODUCE.md)
- [失败审计](RESOURCE_AND_FAILURE_AUDIT.md)

完整私有输入、逐帧状态/P/H/残差、轨迹和设备配置不在公开仓库。公开标量不是独立真值或统计校准。

补充：原高Fitness/首次错峰五Source条件化联合窗口完成额外E0–E4（5配置、25条决策），详见[联合大修正对照](OLD_BRANCH_CONDITIONAL_JOINT.md)。不混入主矩阵30配置统计；没有进行连续前端反馈。
