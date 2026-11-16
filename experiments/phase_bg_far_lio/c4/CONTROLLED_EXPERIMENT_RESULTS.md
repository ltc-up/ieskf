# 本轮实际执行的有限计算

## 数量、身份与未执行项

- CUDA NDT **35次**：18个713组、8个首次切换组、9地图控制；不再增加到40配额。不运行GICP、Seed扰动网格或新15D因果回放。
- 原生FAR合成函数：五步trace off/on数值等价、同50ms单次ID/五次ID、10/50/100ms共同IMU预测。未替代C1真实P。
- C1已有记录：21079对FAR输出/debug，356个评分时隙可恢复受限prior；失败和严格筛选敏感性保存。
- 两次实际ROS2合成发布（每次4s、401时隙），第二次使用补齐输入检查的最终代码；16项发布合同单元测试。
- **未执行**：真实1013长时15D反馈、真实FAR一次ID对照、真实短时无LiDAR消融、生产修改、在线鲁棒门控或新地图构建。

CUDA实际二进制SHA `8ae76d7ece996cf89f498fba2519ba4c73d76b8f64beb69ffd5884a243ea1c08`。复用原先全量Source及captured Target/Seed，没有采样、扩大ROI或OXTS/GNSS选Seed。配置在前置selection中冻结；身份与原始文件SHA只计算这些小冻结输入，不全量哈希bag/map。

Native保持原质量路径，完整6×6、score梯度/Hessian及final保留；未生成Kalman更新、未提交状态/P。历史配置不完整，故新诊断与旧运行分组，不宣称精确历史参数再现。32例旧final差0.000000183～.008069m；3例无可配旧final，明确不可比较。

## 全部35例（差值为0105工程对照，非真值）

| ID | bag(s) | target | iterations | exit | PCL f | support | XY差(m) | align CPU墙钟(ms) |
|---|---:|---|---:|---|---:|---:|---:|---:|

|C4_00|712.486367|1013_captured_raw|30|OBJECTIVE_CONVERGED|19.214|0.622|0.0657|3.442|
|C4_01|712.536363|1013_captured_raw|28|OBJECTIVE_CONVERGED|19.640|0.589|0.0922|3.091|
|C4_02|712.636369|1013_captured_raw|34|OBJECTIVE_CONVERGED|19.252|0.580|0.1612|3.737|
|C4_03|712.736363|1013_captured_raw|26|OBJECTIVE_CONVERGED|22.716|0.563|0.1355|2.685|
|C4_04|712.786371|1013_captured_raw|28|OBJECTIVE_CONVERGED|20.984|0.593|0.2174|3.021|
|C4_05|712.886344|1013_captured_raw|27|OBJECTIVE_CONVERGED|22.607|0.572|0.1427|2.945|
|C4_06|712.986351|1013_captured_raw|33|OBJECTIVE_CONVERGED|23.699|0.524|0.0901|3.429|
|C4_07|713.036343|1013_captured_raw|35|MAX_ITERATIONS|23.748|0.543|0.1082|3.170|
|C4_08|713.136341|1013_captured_raw|35|MAX_ITERATIONS|23.301|0.395|0.8268|3.069|
|C4_09|713.236339|1013_captured_raw|35|MAX_ITERATIONS|17.527|0.557|0.1316|2.872|
|C4_10|713.336340|1013_captured_raw|34|OBJECTIVE_CONVERGED|14.632|0.589|0.4106|3.027|
|C4_11|713.386342|1013_captured_raw|34|OBJECTIVE_CONVERGED|12.393|0.617|0.2034|3.289|
|C4_12|713.486347|1013_captured_raw|31|OBJECTIVE_CONVERGED|10.327|0.647|0.2685|3.163|
|C4_13|713.586355|1013_captured_raw|35|MAX_ITERATIONS|8.033|0.716|0.1010|2.961|
|C4_14|713.636354|1013_captured_raw|35|MAX_ITERATIONS|5.501|0.743|0.3590|2.882|
|C4_15|713.736355|1013_captured_raw|35|MAX_ITERATIONS|4.653|0.772|0.1849|2.863|
|C4_16|713.836367|1013_captured_raw|35|MAX_ITERATIONS|1.854|0.818|0.1276|2.830|
|C4_17|713.936357|1013_captured_raw|35|OBJECTIVE_CONVERGED|0.845|0.839|0.3882|3.318|
|C4_18|516.736391|1013_captured_raw|11|OBJECTIVE_CONVERGED|12.786|0.589|0.0841|1.189|
|C4_19|516.986399|1013_captured_raw|17|OBJECTIVE_CONVERGED|27.133|0.491|0.1114|1.905|
|C4_20|517.486389|1013_captured_raw|17|OBJECTIVE_CONVERGED|23.983|0.566|0.0685|2.429|
|C4_21|517.936350|1013_captured_raw|15|OBJECTIVE_CONVERGED|20.868|0.522|0.1087|1.744|
|C4_22|517.986342|1013_captured_raw|19|OBJECTIVE_CONVERGED|10.232|0.469|4.5063|2.100|
|C4_23|518.036341|1013_captured_raw|19|OBJECTIVE_CONVERGED|8.571|0.446|4.3378|2.059|
|C4_24|518.136349|1013_captured_raw|18|OBJECTIVE_CONVERGED|7.718|0.449|4.3096|2.030|
|C4_25|518.386358|1013_captured_raw|25|OBJECTIVE_CONVERGED|6.868|0.743|0.2252|2.302|
|C4_26|518.036341|1013|19|OBJECTIVE_CONVERGED|8.571|0.446|4.3378|1.908|
|C4_27|518.036341|0105|19|OBJECTIVE_CONVERGED|0.211|0.758|0.0065|1.933|
|C4_28|518.036341|1013_plus_tent|16|OBJECTIVE_CONVERGED|0.248|0.851|0.2146|1.762|
|C4_29|518.136349|1013|18|OBJECTIVE_CONVERGED|7.718|0.449|4.3096|1.836|
|C4_30|518.136349|0105|22|OBJECTIVE_CONVERGED|0.193|0.754|0.0284|2.127|
|C4_31|518.136349|1013_plus_tent|21|OBJECTIVE_CONVERGED|0.218|0.855|0.1905|2.042|
|C4_32|518.386358|1013|25|OBJECTIVE_CONVERGED|6.868|0.743|0.2252|2.308|
|C4_33|518.386358|0105|29|OBJECTIVE_CONVERGED|0.174|0.735|0.0030|2.547|
|C4_34|518.386358|1013_plus_tent|32|OBJECTIVE_CONVERGED|1.925|0.586|4.3058|2.723|

713组的0105对照来自同header旧GPU0105 capture final；首次故障组来自旧profiling的normal_x/y。两者都相关，不能混为统一独立真值或计算全场景RMSE/P95。

## 几何与时序边界

先用RxRyRz角速度图J=[e_x,R_x e_y,R_xR_y e_z]将固定世界XY/世界yaw方向拉回原参数，再对L=10m作Mᵀ(−Hscore)M。保留所有耦合项。它是固定其余方向的局部一阶拉回，不是Schur边缘化；非驻点时不含图转换二阶导数项，不能当精确新坐标Hessian或统计协方差。错误C4_22的投影仍正定，直接反驳“正定=正确”。

align墙钟1.189～3.737ms；首次service冷启动241.604ms，其余最高5.779ms。service包括协议/目标构建/质量，align之外的阶段不能重复相加。`align_internal_ms=0`没有可靠GPU事件意义，GPU细项UNKNOWN；未测完整bag读取/融合/deskew/状态更新周期，不以此验收20Hz。

## 失败、拒绝与分析勘误

保留7个MAX_ITERATIONS，它们不是可提交观测。保留plus-tent地图反例、3个旧final缺失和非等价历史差异。FAR第一次恢复试图将`measured`当raw，FIFO检查失败，未使用错误prior；确认源码后修正为两次补偿差，并保留attempt1聚合。严格10μm与float32坐标舍入界分别保存158/356子集，不能把筛选当成功分类器。

较早ROS原型结果也保留；最终版本有额外无效P/均值/时间检查，未覆盖原输出。没有隐藏重跑失败数据或修改评分区间。当前实际新产物约50MB量级，最终精确字节见私有存储审计。

## 补充验证

16项接口测试、18项保存结果合同检查全部通过。后者包括35唯一注册、完整Native R有限/对称/SPD、H/梯度有限、7失败保留、原生trace等价、同50ms长度、实际ROS消息/数值等价、编译头身份及地图点集增加合同；这不是34项精度测试。原生预测9状态/6输入中心差分检测到速度-倾角A零块不一致，未修复。private A/B直接访问的编译失败保留；修订probe只比较源码审查的零块。报告不得将“发现不一致”写成“完整FAR Jacobian通过”。
