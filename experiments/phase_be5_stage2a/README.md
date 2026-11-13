# Phase BE5 Stage 2A：模型验证与有限冻结诊断

**范围已完成，停在Shadow/冻结候选；不进入实际融合或回放。**

基线30de6fa76ba98e29a83235eee77635485d611009，现有research/phase-be-e0-e4-shadow分支；生产、Q/Bias模型、原门控、匹配参数和旧BE结果不变。

- 合成/回归测试303项通过；A1/A2全部15列中心有限差分覆盖，S²与Native图转换验证通过。
- 55个不同既有案例：49个BE配对prior、5个旧Z窗口、1个仅匹配的12632记录。未运行NDT/GICP、Deskew、因果重传播或全圈回放。
- Native助手不再依赖Fitness准入；Legacy行为保持。原始完整6×6交叉项通过全矩阵合同转换，未对角化、未乘Fitness。
- 49个BE帧的原始Legacy提案重建误差为0；Native Raw及A均只是候选，无Stage2A更新提交到状态/P。
- C没有可认证的独立实数据参考，拒绝接入；旧Z和12632缺失的Native/paired prior合同不伪造。

**决策：当前不建议进入短窗口A软融合。** A2一个旧窗口加速度Bias提案约0.382m/s²超过原0.3域；法向、悬架接触残差和共享gyro噪声仍未建模/独立验证。可继续审查独立法向/来源证据，不能用候选Vz变小宣布精度通过。Native Raw也未完成绝对尺度标定或错峰识别。

详见[A验证](A_MODEL_VALIDATION.md)、[C来源](C_MATH_AND_PROVENANCE.md)、[Native修复](NATIVE_CONVERSION_AND_FRAME_CONTRACT.md)、[冻结结果/增量分解](FROZEN_RESULTS_AND_VERTICAL_DECOMPOSITION.md)、[失败与边界](FAILURES_AND_LIMITATIONS.md)、[测试](TEST_RESULTS.md)、[复现](REPRODUCE.md)、[存储/代码](STORAGE_AND_CHANGELOG.md)。
