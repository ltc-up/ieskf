# REPRODUCE

全部命令在 `/home/ltc/flyagle/ieskf-gicp15d` 下执行。数据根只读，大型输出写
`/media/ltc/新加卷/autoeagle/phase_gicp15d_pipeline/`。

## 0. 前置

- g++ ≥ 13、CMake ≥ 3.16、Eigen 3.4、OpenMP、`nlohmann/json`
- 本地 small_gicp：`/home/ltc/flyagle/auto-eagle/src/a2rl_3rdparty/small_gicp_vendor/upstream/include`
  （`fa0cfc98`，MIT；**不下载、不复制、不修改**）

## 1. 构建

```sh
cd /home/ltc/flyagle/ieskf-gicp15d/src/gicp15d
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

如 small_gicp 不在默认位置：

```sh
cmake -S . -B build -DSMALL_GICP_INCLUDE=/path/to/upstream/include
```

## 2. 测试

```sh
./build/gicp15d_tests
# 期望末行: 41 checks, 0 failures
```

## 3. 案例抽取（Python 仅做离线抽取）

```sh
OUT=/media/ltc/新加卷/autoeagle/phase_gicp15d_pipeline/artifacts
python3 tools/make_cases.py --mode loop44  --out $OUT/cases_loop44.jsonl
python3 tools/make_cases.py --mode aw-equiv --out $OUT/cases_aw_equiv.jsonl
```

脚本会先打印两个 Target 的 sha256，并与每个案例中的 `target_sha256` 一起用于运行期校验：

- 0105 `0895ad26af296aeba68b55350f7b9b70cd6b397be551fc78af75794410192968`
- 1013 `2f7c78d7aa61d219033a3cb9bc1822204db450e9d3ce0405cd1bca9fdcd197ac`

## 4. 运行

```sh
# R1: G0 对 Phase AW 基线（1,468 帧）
./build/gicp15d_frontend --cases $OUT/cases_aw_equiv.jsonl \
    --out $OUT/aw_equiv_g0.jsonl --summary $OUT/aw_equiv_g0_summary.json \
    --presets G0_AW_EQUIV --threads 8

# R2: 双图差分 + 机制消融（206 案例 × 8 preset）
./build/gicp15d_frontend --cases $OUT/cases_loop44.jsonl \
    --out $OUT/loop44_g0_g1.jsonl --summary $OUT/loop44_g0_g1_summary.json \
    --presets G0_AW_EQUIV,G1_SPECTRUM,G1_ROI_CROP,G1_NORMAL_GATE,G1_RECIPROCAL,G1_ROBUST_GM,G1_FULL,G1_COARSE \
    --threads 8
```

实测耗时：R1 约 48 s，R2 约 83 s（8 线程，32 核机器）。
多 Seed 探针（本轮主实验未启用）：

```sh
./build/gicp15d_frontend ... --probe --probe-offsets 0,2,5,8,12
```

## 5. 统计

```sh
python3 tools/analyze.py --results $OUT/loop44_g0_g1.jsonl \
    --frames /home/ltc/flyagle/ieskf/data/phase_as_accuracy_quality/F_multilap \
    --out-json $OUT/loop44_aggregate.json --out-md $OUT/loop44_aggregate.md \
    --title "loop44 dual-map G0/G1"
```

## 6. R1 与 AW 的比对

`$OUT/aw_equiv_g0.jsonl` 的 `case_id` 末段是原始 header ns；与
`/home/ltc/flyagle/ieskf-aw/data/phase_aw_native_gicp_fusion/private/matcher_ab_fixed/matches.jsonl`
按 `ref` 对齐后比较 `final` 的平移/旋转。本轮结果见 `G1_VALIDATION_RESULTS.md` §2。

## 7. 数据来源（只读，均不随仓库发布）

| 用途 | 路径 |
|---|---|
| Source `_A1.bin` + Seed | `ieskf/data/phase_as_accuracy_quality/F_multilap/` |
| Target 0105 | `ieskf/data/inputs/map_target.bin` |
| Target 1013 | `analyzer/lidar_ndt/audits/ndt_source_target_resolution_20261017/inputs/map1013_leaf8.bin` |
| AW 基线结果 | `ieskf-aw/data/phase_aw_native_gicp_fusion/private/matcher_ab_fixed/matches.jsonl` |
| 5925/5931 参考 | `analyzer/2026-09-30_ndt_localization_research_archive/.../analysis_full_lap/full_lap_frame_errors.csv`（**不同 map frame，本轮未使用**） |

## 8. 已知非确定性

多线程下 `ParallelReductionOMP` 的归约顺序使末位比特不确定；单线程
（`--threads 1`）下两次 align 逐位相同（已由测试覆盖）。
报告中的数值均来自 `--threads 8` 的单次运行，未做重复运行方差统计。
