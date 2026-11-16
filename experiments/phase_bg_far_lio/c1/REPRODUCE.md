# 预检复核

只读输入：既有loop44 SQLite、AO事件JSON、AR bindingslap、BE run_loop.py。不要运行ROS、matcher或replay复核本报告。

SQLite以mode=ro&immutable=1打开；查询SELECT min(timestamp),max(timestamp) FROM messages及SELECT id,name,type FROM topics。用topic表确认/init_flag、/initialpose、/rosout未录制。bag起点只定义相对秒，不替代header。

事件JSON筛选kind=wheel且valid，用(header_ns-bag_first_recorder_ns)/1e9分段统计原speed；分别找abs(speed)>.1/.5/1的首个样本。该判据仅报告运动证据，不证明全球初始化。

bindingslap取675/803/1317/1392/1393/1405，逐项保存cloud message_id/header_ns/arrival_ns，取max arrival作为三雷达可用时刻。终点=1405.native_reference_ns+2e9。

BE run_loop.py的lo=min(first.cloud.scan_start_ns)-100ms；从已有pose事件取最后header<=lo，得到历史起点273.401824152s。该源代码证明初始化选择规则，不证明录制pose来自合法全球初始化。私有timeline_evidence.json保存SQL topic表、原始ID与绝对时间，没有复制点云或轨迹。

本轮无定位运行，故无测试精度/周期P95或新算法测试通过声明。git diff --check通过。生成报告前后算法与生产未修改。任何执行性配置需先解除共同初值阻塞，并完成无定位时间合同检查。
