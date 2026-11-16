# 构建、版本与存储

基于研究提交dba3fd1（其基线0d5ffec）；FAR固定 `ef20510316ad1f8dbf2e428e4a193f700493eccd`，缓存Git工作区仍干净，没有重新克隆。CUDA12.8/Jazzy、sm120原生构建成功，最终构建约231.17s。matcher/EKF二进制SHA及配置SHA在各私有run manifest中，软件源/归档SHA在下载账本和公开software_dependencies.json。

TAM__common→param_management/tsl/TAM__msgs/tier4_msgs/autoware_msgs，以及TAM__msgs固定gitlink的autoware_auto_msgs已审。仅补齐必要软件；RAPIDS分支先解析为固定commit，CPM固定0.40.0；CCCL复用CUDA现有2.7.0。FetchContent全部指向本地审过的树并关闭联网，预置上游RAPIDS bootstrap防止隐式下载。没有拉取镜像或rerun可视化SDK。

实际依赖归档累计 **6,451,898字节，约6.153MiB**；少量GitHub树/commit元数据另存，合计远低于512MiB。结束审计已分配总量约463.57MiB；20GiB预算未触发，无清理/覆盖历史结果。所有实际依赖、编译、临时、CUDA/Python/绘图缓存、ROS日志及结果在新加卷BG根目录。未用T7或系统盘放大型产物。

保留两次构建失败：默认中文路径、ASCII符号链接尝试均在ROSIDL消息生成失败。最终在新加卷复制ROSIDL辅助CMake，仅对file(STRINGS)增加ENCODING UTF-8；没有修改系统ROS或生产源。该兼容性修复及前后hash保留。ASCII别名只是一条符号链接，未把编译数据搬到系统盘，也未能独立解决CMake物理路径规范化问题。

首次normal8因日志字节序列化错误丢失内存中的轨迹，日志/配置/二进制身份保留；随后相同窗口/相同算法配置补跑一次，采用逐条落盘。不能将第一次当第二次的独立精度重复测量。第一次运行器源SHA未预先捕获，标UNKNOWN；两次正常段配置和Native二进制身份相同。right3只运行一次。没有自动扩展场景或调参。
