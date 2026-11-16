# 环境、依赖与存储审查及审批方案

CONFIRMED：Ubuntu24.04.5、ROS2 Jazzy、CUDA12.8(nvcc12.8.93)、RTX5070Ti Laptop sm120 /12227MiB、驱动580.178.04。本机版本表面符合官方CUDA12.8.1/Jazzy路线；ABI、sm120实例化/运行能力UNKNOWN，未构建验证。Eigen3.4/PCL1.14及所查Jazzy依赖存在。

Docker29.1.3、NVIDIA Container Toolkit1.19存在，但Docker runtime表未见nvidia；CDI/GPU容器未测试。已有镜像没有FAR-LIO。DockerRootDir在系统盘，禁止直接docker pull/build。新加卷NTFS3实际rw，可用约329GiB；T7未检测挂载。不得相加或静默切盘。NTFS不适合作为原生overlay2根，禁止改daemon或自动迁移。

## 官方镜像元数据（未下载blob）

GHCR latest manifest digest `sha256:5fa1f4ac4ffed4c589f039c8c72f213d60f6a91127fc0a8472f31b21c6a6004e`，17层，压缩层总计 **2,528,517,828字节，约2.355GiB**。这是远程manifest大小和层数，不是已下载占用。解包体积文档约7GB，不是实测；源码Dockerfile提及构建工具链约19GB。镜像内源码commit和实际架构覆盖尚UNKNOWN；不得将latest等同已审commit。暂不申请默认Docker拉取，存储和GPU运行合同未闭合。

## 推荐下一阶段：独立native workspace（本轮未执行）

仅复用现有官方源码缓存，不复制bag/map；TAM__common固定gitlink及robin-map1.4.0、Sophus1.24.6、oneTBB2022.0.0、googletest固定提交、cuCollections固定提交需获取/核验。VISUALIZATION保持OFF，禁止rerun latest下载。googletest FetchContent即使关闭测试也可能获取；cuCollections传递依赖需先审。不得直接cmake触发隐式网络。

以下是**待批准上限，非已知实际下载量**：

|类别|增量预算|位置/策略|
|---|---:|---|
|依赖下载|≤512MiB|仅固定版本；远程大小/传递依赖超限先停止重新申请|
|展开源码/依赖|≤2GiB|新加卷BG专用dependencies，不改官方缓存|
|build/install/编译缓存|≤12GiB|新加卷BG runtime，显式所有CMake/colcon/TMPDIR/cache/log目录|
|两段研究结果|≤1GiB|不存逐帧PCD，不复制bag|
|额外安全余量|4.5GiB|总增量峰值硬预算20GiB，最低空闲保留50GiB|

批准20GiB预算后仍须先补齐输入合同、审查新依赖、冻结有效参数，才可构建和有限运行。原始地图全量加载另需RAM/VRAM预算，磁盘预算不代表显存足够。超下载/存储/内存预算立即停止，不自动清理旧结果。无需历史清理。本轮新增审计预计远小于10MB。
