# REPRODUCE — Phase G1-R

在 `/home/ltc/flyagle/ieskf-gicp15d` 下执行。源码与构建与上轮相同，无新依赖。

## 1. 构建与测试

```sh
cd src/gicp15d
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/gicp15d_tests      # 期望: 48 checks, 0 failures
```

新增：`testProbeCostContract`（探针 cost 有限、零偏移探针 cost 与主结果相等）。

## 2. C4_22 案例输入

Source / Seed / ROI 取自生产侧历史审计树（只读）：

```
/home/ltc/flyagle/auto-eagle-main/fitness_test/src/a2rl_localization/lidar_ndt/audit/
  ndt_uncertainty_research/phase_e4_local_convergence/runs/A1_source_restore/captures/1763205937849981440.bin   # narrow, 2499 pts
/home/ltc/flyagle/auto-eagle-main/fitness_test/src/a2rl_localization/lidar_ndt/audit/
  ndt_uncertainty_research/phase_e4_local_convergence/runs/A1_wide35/captures/1763205937849981440.bin           # wide, 3832 pts
```

Target 由冻结 1013 云按 json 记录的 ROI 重建（圆柱 r=120，z∈[cz−10,cz+30]），
脚本见 `src/gicp15d/tools/make_cases.py` 的 C4_22 分支说明与下方命令。

```sh
OUT=/media/ltc/新加卷/autoeagle/phase_gicp15d_pipeline/g1_recovery
# 重建 Target 并生成 cases_c4_22.jsonl（python 只做离线抽取）
# 说明：归档的 cases_c4_22.jsonl 生成时 Z 带用 seed.z-10/+30 计算；
# 现在该脚本改读请求里记录的 region[9]/region[10]，两者相差 2e-6 m，
# 对 98k 点 Target 无实际影响。种子与 Source 完全一致。
python3 src/gicp15d/tools/make_cases_c4_22.py --out "$OUT"
./build/gicp15d_frontend --cases "$OUT/cases_c4_22.jsonl" \
    --out "$OUT/c4_22_results.jsonl" --presets G0_AW_EQUIV,G1_SPECTRUM --threads 8
./build/gicp15d_frontend --cases "$OUT/cases_c4_22.jsonl" \
    --out "$OUT/c4_22_probe_results.jsonl" --presets G0_AW_EQUIV,G1_SPECTRUM \
    --threads 8 --probe --probe-offsets 0,2,4,-2,-4
```

实测：4 结果 0.1 s；带探针 0.3 s。

## 3. R1 异常帧归因

```sh
REC=$OUT
# 19 个 >10cm 帧 + 6 个对照，从 artifacts/aw_equiv_g0.jsonl 与 AW matches.jsonl 提取
python3 -c "..."   # 见 G0_NUMERICAL_DISCREPANCY.md §2
for t in 1 8; do
  ./build/gicp15d_frontend --cases "$REC/cases_r1_outliers.jsonl" \
     --out "$REC/r1_outliers_t$t.jsonl" --presets G0_AW_EQUIV --threads $t
done
```

## 4. 数据来源（只读）

| 用途 | 路径 |
|---|---|
| C4_22 Source/Seed/ROI | `phase_e4_local_convergence/runs/{A1_source_restore,A1_wide35}/captures/1763205937849981440.json` |
| C4_22 历史 NDT 结果 | `phase_e4_local_convergence/results/closed_loop/{NARROW,WIDE}_A1_35_pass8_frames.csv` |
| 1013 Target | `analyzer/lidar_ndt/audits/ndt_source_target_resolution_20261017/inputs/map1013_leaf8.bin` |
| 5925/5931 | `analyzer/lidar_ndt/audits/gicp_vgicp_benchmark/runs/20261015_actual/`（未执行） |

## 5. 判读提醒

`PROXY` 是历史 0105 稳定 NDT 代理，**不是真值**。
报告中的 `dPROXY` 只在同一代理口径内可比，不能当作定位精度。
