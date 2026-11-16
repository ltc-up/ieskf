# FAR重复/唯一ID真实因果消融：NOT RUN

CONFIRMED：再次检查C1原生输出与文件清单：有pose 6×6及twist covariance、raw状态/诊断和60项debug字段，没有完整9×9 P检查点、FIR内部状态、每次correct的实际唯一LiDAR ID/K/H/S。两个边际协方差不能恢复姿态/位置与速度的交叉块。bag原始IMU和发布账本不证明原生callback/tick精确交错及滤后输入相同。

因此不能构造题设要求的相同完整P、相同连续合法前史A/B。未使用C4合成P或中途冷启动替代，没有新FAR回放。正式真实冻结观测消融NOT RUN。

复用证据：C4受限right3 104 tick prior/post横向P95=.530842/.523392m，单tick增量P95=.008187m；不是完整窗口。C4原生同50ms合成一次/五次同ID融合y=.200087/.272913m，Pyy=.0335211/.00914242m²，仅证明重复信息能改变均值并压缩P，不是C1真实贡献比例。

UNKNOWN：FAR最终横向优势有多少来自累计重复LiDAR、FIR/倾角、不同预测、延迟及初始化。不能因C5避免错峰而补出FAR因果答案。

待单独批准最小方案：从既定Pit初值完整重建原生后端一条合法前史，保存完整9×9 P、FIR状态、回调顺序、各tick滤后IMU/倾角/有效mask与唯一GICP ID；冻结同一既有GICP序列和原始参数，仅对LiDAR有效mask消费次数做A/B。保留其它辅助更新，不能跳过整个joint correction。保存每次x/P/H/R/K/S和有效ID，数值等价插桩后才分析。冻结前端观测只能称后端反事实，不能宣称GICP反馈闭环。当前没有批准自动重跑此方案。
