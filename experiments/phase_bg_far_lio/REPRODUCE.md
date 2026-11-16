# 复核说明

公开仓库只包含审计脚本、去标识报告与manifest，不包含原bag/点云/轨迹/外参/生产源码。完整本机输入证据和冻结身份在独立私有audit目录，原输入只读复用。

`inspect_inputs.py`只通过SQLite immutable只读URI取bag开始后1秒消息，反序列化字段/时间/IMU/TF统计，不启动rclpy节点、不发布、不回放、不运行配准；输出含私有TF，不提交公开Git。需ROS2 Jazzy Python环境及numpy。指定新加卷输出，PYTHONDONTWRITEBYTECODE=1，所有临时/cache/log也显式设在新加卷。

    python3 experiments/phase_bg_far_lio/inspect_inputs.py --bag AUTHORIZED_BAG_DB3 --out NEW_PRIVATE_AUDIT_JSON

不要通过此命令扩大到全包；不要将带私有TF的JSON提交。既有16812输入和结果manifest可复用；未全量重算大文件哈希。仅新报告、小审计脚本、配置/选择manifest作身份校验。

官方固定commit ef205103；源码对照文件列在SOURCE_AND_PAPER_REVIEW。容器只查询manifest、不下载层。冻结窗口绑定身份列表在私有frozen_windows.json；公开仅保留其hash，后续运行前必须核对。所有构建/回放命令尚未批准，也未执行；本文件不提供一个会隐式拉镜像或FetchContent的‘一键复现’。
