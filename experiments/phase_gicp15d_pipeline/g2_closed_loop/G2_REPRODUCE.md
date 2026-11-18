# G2_REPRODUCE

## 0. 环境

g++ 13.3、CMake 3.28、Eigen 3.4、OpenMP、nlohmann/json；Python 3.12 + numpy/scipy；
CUDA 可用（仅 Deskew 使用）。32 线程 / 30 GiB。

## 1. 构建

```sh
cd /home/ltc/flyagle/ieskf-gicp15d/src/gicp15d
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j        # 产出 gicp15d_core / gicp15d_g0_service / gicp15d_tests
./build/gicp15d_tests         # 63 checks, 0 failures
```

## 2. 闭环运行（关键：CWD 必须是仓库根，绑定里是相对路径）

```sh
cd /home/ltc/flyagle/ieskf        # 不是 ieskf-aw
OUT=/media/ltc/新加卷/autoeagle/phase_gicp15d_pipeline/g2_closed_loop
PYTHONPATH=/home/ltc/flyagle/ieskf-aw/src:/home/ltc/flyagle/ieskf-aw/experiments/phase_aw_native_gicp_fusion \
python3 /home/ltc/flyagle/ieskf-aw/experiments/phase_aw_native_gicp_fusion/run_loop.py \
  --input data/phase_ao_ap_validation/loop44.json \
  --bindings data/phase_ar_loop44_map0105/bindingslap.json \
  --target data/inputs/map_target.bin \
  --deskew data/phase_ar_loop44_map0105/runtime/deskew_service \
  --deskew-service data/phase_ar_loop44_map0105/runtime/deskew_service \
  --matcher /home/ltc/flyagle/ieskf-gicp15d/src/gicp15d/build/gicp15d_g0_service \
  --private "$OUT/lap_g0" --output "$OUT/lap_g0_summary.json" \
  --backend gicp --window lap
```

实测 38 s，1468 帧。`--private` 必须是**尚不存在**的目录（脚本断言）。

## 3. 确定性测试

同一命令加 `--limit 200`，两个不同 `--private`，比较
`transactions.jsonl` 的 `ndt.final` 与 `states.jsonl` 的 `p`。

## 4. 已知陷阱

1. **CWD**：`bindings` 里的 `raw_path` 是相对路径，必须在 `/home/ltc/flyagle/ieskf` 下运行。
2. **`--input`**：必须是 `data/phase_ao_ap_validation/loop44.json`。
   `data/history/phase_v/results/input_continuous_extension.json` 只有
   `initial_pose/baseline`，没有 `pose` 事件，会以 `max() iterable argument is empty` 失败。
3. `--deskew` 与 `--deskew-service` 都要给；后者为常驻服务，前者在该路径下不会被调用。
4. 输出目录必须不存在。

## 5. 数据身份

| 项 | 值 |
|---|---|
| Target | `data/inputs/map_target.bin`，sha `0895ad26…`，908,013 点 |
| 绑定 | `data/phase_ar_loop44_map0105/bindingslap.json`（1468 帧，273.556–346.906 s） |
| Deskew | `runtime/deskew_service`（CUDA，SHA 见 `data/phase_ar_loop44_map0105/runtime/`） |
| 参考 | `ieskf-aw/.../private/gicp_lap/evaluation/evaluation.npz` 的 `G_0.075` / `historical_valid_0.075` |

大型输出全部在新加卷，未回退系统盘。
