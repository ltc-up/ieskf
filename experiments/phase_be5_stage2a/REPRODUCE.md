# Stage2A复现与授权边界

只运行合成测试不需要真实点云。设置PYTHONPATH=src、PYTHONDONTWRITEBYTECODE=1和新加卷下TMPDIR/cache，运行：

    python3 -m pytest tests -q -p no:cacheprovider --basetemp=$TMPDIR/be5_tests

可将BE5_UNIT_METRICS指向新输出JSON保存FD误差。只调用纯模型，不启动ROS/NDT/Deskew。

真实冻结诊断需要授权的私有root、源版本匹配的controlled2_matched_lap保存记录、motion/TF、五个旧Z快照和历史12632记录。不能从公开Git重建真实payload，亦不得去旧home路径找点云副本。所有cloud相关数据无需加载；存储清理alias仍可用，但本诊断不需要恢复云。

在新的、未存在输出目录，配合已审查的frame certificate：

    python3 experiments/phase_be5_stage2a/frozen_diagnostics.py --root PRIVATE_ROOT --out NEW_OUT --frame-contract QUALIFIED_CERT

默认选48BE+5旧Z+1历史12632=54；本次随后只补了一个不同BE原始匹配代理困难帧，使用--only-index INDEX保存独立补充输出，没有重跑原54。若复核同一记录，使用原selection.json和代码初版身份；不能通过多次调用突破用户总计60不同帧授权。driver自身每次检查≤60与500MB预算；跨调用总量必须由审批账本约束，不是自动获得新帧授权。

最后analyze_frozen.py --input FIRST_OUT --supplement SUPPLEMENT_OUT --out NEW_ANALYSIS仅聚合同一55案例。候选状态/P只在副本/数学输出中存在，任何ingest/传播/部署或真实A/C更新都不在权限内。

frame证书只允许保存BE离线数值map，不是任意打一个CONFIRMED标志：需核验旧源版本、直接map target/seed、base来源、服务参数/C合同，以及每帧header/base seed/float seed/final参数一致性；不满足则停止，不假设I。生产PCM→ENU及来源不明数据不得使用。私有文件路径、矩阵、源码/数据版本账本另存。

初版driver是事后保存；补充批有运行前digest。结果包含原实际R、候选R/K/P/15D提案、旧提交/发布状态、失败原因和每条保存输入的SHA。旧E0～E4统计文件未改变。重执行任何真实诊断仍需符合当前授权上限，不运行12s、全圈/三圈、配准或点云重建。

当前交付的规范案例在verified_55/cases.jsonl。verify_cases.py只为已保存首54/补充1批次修正时间标签并核对原行SHA，不调用模型重算。新的driver直接输出正确标签；对历史初版不要将actual_published_state字段与row.state_stamp配对。缺最终发布速度导出时保持UNKNOWN。
