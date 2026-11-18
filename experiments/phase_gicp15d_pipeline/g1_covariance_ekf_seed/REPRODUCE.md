# REPRODUCE — Phase G1-C

在 `/home/ltc/flyagle/ieskf-gicp15d` 下执行。无新依赖。

## 1. 构建与测试

```sh
cd src/gicp15d
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/gicp15d_tests     # 期望: 63 checks, 0 failures
```

新增：`testFarFrobeniusRegularization`（7 组）与 `testFrobeniusCloudMode`。

## 2. 案例生成（离线抽取，Python 不参与配准）

```sh
OUT=/media/ltc/新加卷/autoeagle/phase_gicp15d_pipeline/g1_covariance_ekf_seed
python3 tools/make_cases_c4_series.py --out "$OUT"
# -> 26 cases, 26 distinct headers, 8 with a proxy row
```

输入（只读）：

| 用途 | 路径 |
|---|---|
| 冻结 Source/Target/Seed | `/media/ltc/新加卷/autoeagle/phase_bg_far_lio/c4/frozen/{requests.json, <NN>/source.bin, <NN>/target.bin}` |
| 工程代理 + 历史 NDT | `analyzer/2026-10-05_ndt_localization_final_archive/analysis/tent_map_hypothesis/WALL_frames.csv` 的 `ndt_x_good/ndt_y_good`、`ndt_x/ndt_y`、`good_diff` |

## 3. 运行

```sh
./build/gicp15d_frontend --cases "$OUT/cases_c4_series.jsonl" \
    --out "$OUT/series_g0_frob_spec.jsonl" --summary "$OUT/series_summary.json" \
    --presets G0_AW_EQUIV,G1_FAR_FROBENIUS,G1_SPECTRUM --threads 8
# 实测: 78 results in 2.5 s
```

## 4. 统计

```sh
python3 - <<'PY'   # 见 CONTINUOUS_SEED_RECOVERY.md 的表格
# 对每个 preset 计算 |final - proxy|、|final - historical_ndt|、|final - seed|
PY
```

## 5. 关键身份

| 项 | 值 |
|---|---|
| C4_22 Source | `phase_bg_far_lio/c4/frozen/22/source.bin`，2499 点，sha `964d20da…` |
| C4_22 Target | `phase_bg_far_lio/c4/frozen/22/target.bin`，98213 点，sha `cfff1a6c…` |
| C4_22 Seed | `requests.json` 的 `seed_common` |
| FAR-LIO | `ef20510`，`map_handler_base.hpp:393-401` |
| small_gicp | `fa0cfc98`（未改） |

## 6. 判读提醒

- 代理 `ndt_x_good/ndt_y_good` 是历史 0105 稳定 NDT 轨迹，**不是 GNSS 真值**。
- 每帧 GICP 从该帧 Seed 独立初始化，结果不反馈下一帧：
  **这是捕捉能力的反事实配准，不是 GICP 连续闭环。**
