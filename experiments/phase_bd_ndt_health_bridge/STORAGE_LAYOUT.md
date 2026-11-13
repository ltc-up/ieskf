# 实际存储位置与迁移核验

本轮唯一活动存储根目录：`/media/ltc/新加卷/autoeagle/phase_bd_ndt_health_bridge/`。

|用途|完整路径|
|---|---|
|独立研究仓库|`/media/ltc/新加卷/autoeagle/phase_bd_ndt_health_bridge/repository/`|
|报告与公开独立代码|`/media/ltc/新加卷/autoeagle/phase_bd_ndt_health_bridge/repository/experiments/phase_bd_ndt_health_bridge/`|
|全部本轮私有成功/失败结果及原执行脚本|`/media/ltc/新加卷/autoeagle/phase_bd_ndt_health_bridge/private/`|
|迁移后的历史研究输入|`/media/ltc/新加卷/autoeagle/phase_bd_ndt_health_bridge/inputs/research_data/`|
|活动P0编译目录|`/media/ltc/新加卷/autoeagle/phase_bd_ndt_health_bridge/build/p0/`|
|活动KD-tree编译目录|`/media/ltc/新加卷/autoeagle/phase_bd_ndt_health_bridge/build/kdtree/`|
|活动运行时与新路径脚本|`/media/ltc/新加卷/autoeagle/phase_bd_ndt_health_bridge/runtime/`|
|编译/验证日志|`/media/ltc/新加卷/autoeagle/phase_bd_ndt_health_bridge/logs/`|
|原Git完整镜像、bundle、元数据、未提交工作快照及迁移证据|`/media/ltc/新加卷/autoeagle/phase_bd_ndt_health_bridge/preservation/`|

约34GiB本轮结果与约86GiB既有研究输入已复制。迁移前保存全部本地分支、未推送提交、工作树状态和原.git元数据；所有原研究工作树除BD未提交报告外均干净。研究库无stash；生产库3条stash只读核验、原样保留。原/home仓库不移动、不清理、不重写、不提交；新卷是独立普通clone，不共享旧工作树Git元数据。

源目标rsync校验采用checksum模式，私有结果与研究输入的差异日志均为空（须以对应进程成功退出为准）。原Git镜像fsck和bundle verify通过。新卷重新编译并使用新卷source/target执行25次真实CUDA等价校验，final/gradient/Hessian/objective/exit/iterations均逐值一致。原执行记录中的旧路径作为历史证据保留，不是当前工作目录或输出位置。复制的旧CMake缓存仅归档，禁止继续构建；活动构建目录见上表。

原AY/BB等已有新卷目录不覆盖。新增文件、编译临时目录、图形缓存、日志与结果均放在上述根目录内。迁移后的输入绑定和脚本存于runtime；仓库data链接仅指向新卷inputs/research_data。
