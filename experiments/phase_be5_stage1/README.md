# Phase BE5 第一阶段审查草稿

**只读审查已完成；没有算法改动、新配准、有限差分执行或新的全量/短窗口回放。等待具体实现批准。**

证据标签：CONFIRMED为直接源码/保存记录；HYPOTHESIS为推导候选、机制解释或实验计划；UNKNOWN为缺失物理/运行时证据。数学推导可审查不等于已通过数值/真实数据验证。

- [数据流/生产对照](SOURCE_AND_DATAFLOW_AUDIT.md)
- [垂向失效机制](ESKF_VERTICAL_FAILURE_ANALYSIS.md)
- [A软约束推导](A_VERTICAL_VELOCITY_MODEL.md)
- [C姿态参考](C_ATTITUDE_REFERENCE_MODEL.md)
- [Legacy/Native协方差](NDT_COVARIANCE_REVIEW.md)
- [实验矩阵及验收](EXPERIMENT_PLAN.md)
- [最小改动及NO-GO](MINIMAL_CHANGE_PROPOSAL.md)
- [存储预算](STORAGE_BUDGET.md)

## 决策摘要

HYPOTHESIS：A值得先做冻结诊断/数学测试；没有可靠路面法向时仅用body法向低动态Shadow，不直接融合。A不是map-Vz清零，也不能判错峰。

UNKNOWN：C尚无足够独立、逐帧可追溯且噪声/时间/轴已标定的来源；AHRS能回退Bosch，自Deskew也不独立。因此仅Shadow，强融合NO-GO。

CONFIRMED：Native矩阵和参数已保存在BE记录中，已有完整转换助手，但其隐藏Fitness入口须先解耦。HYPOTHESIS：正确转换且独立标定后可取代Fitness生成R；目前只能支持Native Raw公平对照，不能宣称绝对尺度合格。Fitness保留独立诊断和Legacy回滚，不乘Native。

CONFIRMED：全P耦合是真实传播/更新机制；4.35～6.27m/s是冻结候选，不是已证实状态跳变。ZOH噪声近似、未标定Legacy R和参数图风险已有源码依据；轴/重力/参考点/时间的物理合同与服务二进制等价性仍有UNKNOWN，先查合同不调Q掩盖。

最小下一步建议：经批准后增加独立冻结诊断driver、A1/A2 Jacobian测试，拆解Native助手的Fitness依赖并验证完整图/R；C和Native-Calibrated缺先决证据则N-A。方案B、强RP/垂向硬约束、Native×Fitness、GMF/GICP/全局恢复、优化器/Q/Bias/生产6D修改不在范围。

八份文件为草稿，不表示用户已经批准代码实现、传感器新接入或定位通过。
