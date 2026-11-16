# BG-C3：观测利用效率与错误匹配鲁棒融合

研究基线38c42314，现有研究分支。完成源码/文献审查、297个固定prior的891个R候选、历史错峰账本及100Hz输出合同设计。**零新NDT、零传播/回放、零状态/P提交；没有执行闭环A/B。**

- [100Hz输出合同](100HZ_ESTIMATION_OUTPUT_CONTRACT.md)
- [Legacy R信息审计](LEGACY_R_INFORMATION_AUDIT.md)
- [正确/错误案例库](CORRECT_VS_WRONG_NDT_CASEBOOK.md)
- [R无反馈结果](R_SCALE_COUNTERFACTUAL_RESULTS.md)
- [文献与源码](ROBUST_OBSERVATION_LITERATURE.md)
- [融合机制设计](LATERAL_CORRECTION_MECHANISM_DESIGN.md)
- [闭环停止决定](CLOSED_LOOP_DECISION.md)
- [最终结论](FINAL_CONCLUSION.md)、[复核](REPRODUCE.md)

×1增强right3即时横向修正，但normal8退化、Yaw及错误候选风险上升。现有证据尚不支持一个满足保护条件的闭环候选，因此按用户停止门停在诊断/设计阶段。状态输出合同可独立推进，但没有声称实现/验证真实100Hz发布或50ms实时性。

CONFIRMED=源码/保存数据/固定prior代数直接证据；HYPOTHESIS=尚无独立消融的机制；UNKNOWN=缺少必要记录或独立参考。全部位置参考只用于事后评价，未据评分选择R。
