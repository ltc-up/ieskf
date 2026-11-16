# BG-C2：车辆模型、延迟融合与横向优势的只读复盘

基线 `62eb9472887483d94c22a67c81b90cb2934a90d3`，沿用C1的两次已完成连续运行及冻结评分区间。本轮没有匹配、滤波更新/回放、Q/R修改、下载或生产变更。未在当前研究副本找到独立的原BG-C2报告，因此本补充自包含，不假称核验过缺失报告。

- [9D与15D模型](9D_VS_15D_VEHICLE_MODEL.md)
- [三种延迟融合](DELAYED_FUSION_THREE_WAY_REVIEW.md)
- [自适应R及未来实验](ADAPTIVE_R_RESEARCH_PLAN.md)
- [横向因果证据](LATERAL_ERROR_CAUSAL_HYPOTHESES.md)
- [架构决策](NEXT_GENERATION_EKF_ARCHITECTURE_DECISION.md)
- [C1机制勘误](ERRATA_BG_C1.md)、[源码证据](SOURCE_EVIDENCE.md)、[复核与预算](REPRODUCE.md)

**核心结论：尚不能用FAR最终横向优势证明9D优于15D。**右弯原始NDT横向P95反而较小；发现FAR ROS层重复提交缓存观测，而15D一次信息接入仍沿用Legacy ×5倍率。FAR评分段横向R没有自适应变化，位置残差裁剪也未触发。优先审清观测信息预算和已保存增益，再考虑标定R，模型减维暂不实施。

CONFIRMED指源码或本次保存记录直接支持；HYPOTHESIS指合理机制但没有独立消融；UNKNOWN指缺少所需证据。工程参考不是独立真值，所有精度排序保留0/75/150ms敏感性。文中P95默认为绝对值P95。
