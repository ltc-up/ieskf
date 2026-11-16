# 复现与归档

公开内容：新研究脚本、14项测试、去标识统计/诊断图、软件版本/下载账本摘要、各源文件hash。私有内容：逐点/地图输入身份、绝对时间/TF、运行YAML和初始化pose、Native二进制、完整ROS输出/日志及失败尝试。不能从公开Git独立重建受限数据；原始数据始终只读。

软件源只用已有FAR缓存加固定版本依赖归档，fetch_dependency.py累计检查512MiB；不使用git clone或Docker。build_native.py使用单独build/install/tmp/cache/log路径、CUDA120和全断网FetchContent。ROSIDL UTF-8补丁只在私有依赖副本，未分发原系统文件；具体单行变更与hash见BUILD_AND_STORAGE及私有审计。

离线检查命令（ROOT/DATA/SOURCE必须指向获批工作卷；未来再运行真实数据仍需相应批准）：

    python3 -m pytest experiments/phase_bg_far_lio/test_input_adapter.py -q -p no:cacheprovider
    python3 experiments/phase_bg_far_lio/validate_inputs.py --data DATA --selection FROZEN_SELECTION --out NEW_AUDIT_OUT
    python3 experiments/phase_bg_far_lio/analyze_windows.py --root ROOT --be SAVED_BE_ROOT --data DATA

所有缓存/临时/ROS日志使用工作卷，设置PYTHONDONTWRITEBYTECODE和MPLCONFIGDIR。运行器分别只允许normal8/right3，默认拒绝覆盖。仅一次normal8记录错误后以retry1补跑，不能默认反复使用retry1；它同样拒绝覆盖。未来不准以改目录名字绕过运行审批。

新增脚本prepare_native_inputs.py仅将已认证0.8m BIN的XYZ保真序列化PCD并检查共同首帧prior；run_native_window.py不导入离线评价器、不读GNSS/GT参考；analyze_windows.py只在运行结束后加载工程参考。所有时序按原recorder_ns调度。

注意首轮normal8未逐条落盘，原轨迹无法事后恢复，只保留日志/配置/失败说明；不能宣称每次尝试都已精确复现。其后完整输出已逐条保存。OS/DDS/IO调度本来非确定性，重执行不保证逐毫秒一致，也不保证不同实际延迟得到同一闭环轨迹。

静态地图输入SHA相同，Native内部保留点仍有105点跨运行差异，已保留计数；不进行额外重复测试来追求确定性。首次/补跑normal8的运行器代码并非同一版本（仅记录修复），不把两者作为同代码稳态重复性能样本。
