# Phase BE：多模态、多假设与鲁棒延迟融合设计

本轮交付仅含审查与设计，没有算法实现或新精度实验。GNSS按后续用户要求限定为生产现有开关/融合语义，另审计后轴参考点。

阅读入口：[最终建议](FINAL_RECOMMENDATION.md) → [生产审计](PRODUCTION_PIPELINE_AUDIT.md) → [文献](LITERATURE_REVIEW.md) → [统一数学架构](MULTIMODAL_LOCALIZATION_ARCHITECTURE.md) → [实验路线](EXPERIMENT_ROADMAP.md)。

专题：[多假设](MULTI_HYPOTHESIS_ESTIMATION.md)、[方向信息](DIRECTIONAL_OBSERVABILITY.md)、[双匹配后端](NDT_GICP_COMPLEMENTARY_MATCHING.md)、[侧滑](VEHICLE_SLIP_ESTIMATION.md)、[GNSS开关和参考点](GNSS_INTEGRITY_AND_FUSION.md)、[延迟生命周期](OOSM_FIXED_LAG_FUSION.md)、[恢复连续性](RECOVERY_CONTINUITY.md)、[计算预算](COMPUTATIONAL_BUDGET.md)、[源码证据](SOURCE_REVIEW.md)。

结论：支持有限规模Shadow研究，不支持直接生产替换。坐标参考点、现场运行参数、候选权重标定与完整50ms实时性均未通过资格。
