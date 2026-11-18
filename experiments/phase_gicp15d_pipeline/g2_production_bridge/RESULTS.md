# G2 production GICP + 15D closed-loop replay

Phase baseline: `159868e`. Research branch: `dsh/gicp15d-pipeline`. Production source and small_gicp were not modified. This run used the host GPU for the archived CUDA Deskew service and four CPU threads for GICP.

## Implementation

`gicp15d_production_service` is an offline JSONL bridge around the existing `lidar_ndt::GicpRegistration`. It reads the frozen XYZ source and target format, applies the replay's cylindrical target ROI, passes the predicted map/base Seed to the production adapter, and returns the resulting map/base pose. The 15D IMU/Wheel propagation, delayed `ExternalPose` update, and existing fixed engineering R were retained. The original G0 solver and OpenMP reduction were not changed. No numerical identity with the older research frontend was required.

The bridge verifies the target SHA on first load. The experimental ROI reuse mode retains the cropped target until the Seed moves 10 m horizontally or 5 m vertically. Those limits fit inside the replay source crop (100 m longitudinal, 35 m lateral, 1–15 m high), 120 m target radius, 4 m correspondence gate, and vertical target margins. This preserves the target pointer so the production adapter can reuse its tree and covariances.

## Actual full-loop results

All rows are 1468-frame own-state feedback runs over 73.35 s. The comparison uses the same archived `G_0.075` reference and its 1197-frame historical-valid mask. It is an engineering **relative** comparison; the cached reference is not certified ground truth and its apparent XY offset prevents an absolute accuracy claim.

| Run | Accepted | XY RMSE m | Lateral RMSE m | Lateral p95 m | Matcher p50/p95 ms | Frame p50/p95/p99 ms |
|---|---:|---:|---:|---:|---:|---:|
| Production GICP, fresh ROI | 1406 | 2.469 | 0.361 | 0.795 | 30.90 / 47.01 | 45.62 / 62.86 / 67.84 |
| Production GICP, ROI reuse | 1406 | 2.469 | 0.361 | 0.795 | 4.60 / 34.62 | 15.81 / 48.96 / 61.19 |
| Archived NDT Legacy + 15D | 1467 | 2.622 | 0.295 | 0.669 | 2.19 / 3.66 | not remeasured |
| Archived NDT Fixed-R + 15D | 1467 | 2.506 | 0.365 | 0.745 | 2.22 / 3.42 | not remeasured |

ROI reuse hit 1200/1468 frames. Fresh/reuse trajectories differed by at most 1.45 µm in state position in these runs; this is an observation about the experiment, not a bitwise-equivalence requirement. The largest accepted-observation gap was 0.150 s. The full replay never entered the 2 s observation-loss stop condition. ROI reuse reduces the main adapter cost, but the frame p99 61.19 ms still misses a strict 50 ms deadline; ROS publication and scheduling were not tested.

## Location problem and A/B

The GICP run is better than the two archived NDT runs in the temporary global XY metric, but worse than NDT Legacy in lateral error. The persistent lateral difference is visible around frames 1254–1296 and 1317–1405. In frames 1317–1405, lateral RMSE is 0.669 m for GICP, 0.425 m for NDT Legacy, and 0.654 m for NDT Fixed-R. At frames 869–897 all three have large lateral error (GICP 0.898 m), so the cached reference and common feedback path also require scrutiny. These are paired engineering-reference differences, not proof of vehicle boundary risk.

As a direct feedback A/B, GICP position-only observations kept the established position R and all 15D propagation unchanged. That loop stopped at frame 394 after 2.000 s without an accepted observation: 188 updates, 201 invalid/nonconverged matches, 2 bias-domain rejections, and 3 attitude-domain rejections. The 6D orientation update is necessary for this run's matching stability; position-only is rejected as an improvement. A simple outlier gate is not justified by the observed sustained offset: the problematic frame range 1280–1410 has accepted conditional NIS below 5, while accepted runs peak at 16.15 elsewhere.

**Assessment:** GICP geometry can reduce the temporary global XY difference, but lateral positioning is not yet better than NDT Legacy. The strongest measured engineering bottleneck was repeated ROI target preprocessing; ROI reuse fixes much of its cost. The sustained lateral offset remains unresolved and cannot be assigned uniquely to GICP geometry, EKF feedback, or reference timing from this single lap. Keep 6D EKF feedback and G0 Q/R baseline. Functional closed-loop **GO** for research; precision and strict real-time **NO-GO** for production.

## Reproduce

Set `RESEARCH_ROOT`, `DATA_ROOT`, `LIDAR_NDT_SOURCE_DIR`, `OUTPUT_ROOT`, and `AW_REFERENCE` to authorized local paths. Run from `DATA_ROOT` so raw bindings resolve. The output directory must not exist. `OUTPUT_ROOT` and the per-frame files are private; do not commit them.

```sh
cmake -S "$RESEARCH_ROOT/src/gicp15d" -B "$OUTPUT_ROOT/build" -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_PRODUCTION_GICP_SERVICE=ON -DLIDAR_NDT_SOURCE_DIR="$LIDAR_NDT_SOURCE_DIR"
cmake --build "$OUTPUT_ROOT/build" --target gicp15d_production_service -j4
ctest --test-dir "$OUTPUT_ROOT/build" --output-on-failure
cd "$DATA_ROOT"
export PYTHONPATH="$RESEARCH_ROOT/src:$RESEARCH_ROOT/experiments/phase_gicp15d_pipeline/g2_deterministic"
python3 "$RESEARCH_ROOT/experiments/phase_gicp15d_pipeline/g2_deterministic/run_loop_det.py" \
  --input data/phase_ao_ap_validation/loop44.json \
  --bindings data/phase_ar_loop44_map0105/bindingslap.json --target data/inputs/map_target.bin \
  --deskew data/phase_ar_loop44_map0105/runtime/deskew_service \
  --deskew-service data/phase_ar_loop44_map0105/runtime/deskew_service \
  --matcher "$OUTPUT_ROOT/build/gicp15d_production_service" \
  --private "$OUTPUT_ROOT/lap_fresh" --output "$OUTPUT_ROOT/lap_fresh_summary.json" \
  --backend gicp --window lap
export G2_PRODUCTION_MATCHER="$OUTPUT_ROOT/build/gicp15d_production_service"
python3 "$RESEARCH_ROOT/experiments/phase_gicp15d_pipeline/g2_deterministic/run_loop_det.py" \
  --input data/phase_ao_ap_validation/loop44.json \
  --bindings data/phase_ar_loop44_map0105/bindingslap.json --target data/inputs/map_target.bin \
  --deskew data/phase_ar_loop44_map0105/runtime/deskew_service \
  --deskew-service data/phase_ar_loop44_map0105/runtime/deskew_service \
  --matcher "$RESEARCH_ROOT/experiments/phase_gicp15d_pipeline/g2_deterministic/matcher_production_roi10.sh" \
  --private "$OUTPUT_ROOT/lap_roi10" --output "$OUTPUT_ROOT/lap_roi10_summary.json" \
  --backend gicp --window lap
```

Run `analyze_production.py` with `--input`, `--candidate`, `--legacy`, `--fixed`, `--reference`, and `--output` to create the local trajectory plot, lateral-error plot, per-frame CSV, and summary. For position-only A/B add `--position-only` to `run_loop_det.py` and choose a new output directory. The baseline NDT trajectories and reference must be supplied separately.
