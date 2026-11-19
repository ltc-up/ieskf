# Phase G2-N — GICP vs NDT-CUDA under fixed LiDAR covariance

**Status: completed exploratory closed-loop comparison.** Research branch: `dsh/gicp15d-pipeline`, based on `eb0de085b7431071d66943909d2e948cf8d9665a`. All runs used 4,134 consecutive frames from the existing three-lap loop44 stream, each matcher seeded from its own current 15D ESKF prediction. Production defaults and the E2 health implementation were unchanged.

## Question and controlled conditions

The comparison asks whether NDT-CUDA can replace production small_gicp in the existing 15D ESKF + Kistler C pipeline while retaining accuracy and improving runtime. All runs used the same IMU, optical Kistler events, mutually exclusive Wheel fallback, 15D propagation, delayed full-six-degree pose update, Deskew, Q, base validity rules, initial state, three-lap bindings, target map, and 75 ms reference archive. Each branch rebuilt its own ROI around its own predicted Seed and performed a fresh match. No run reused another branch's final pose as its observation.

| Arm | Matcher | LiDAR pose covariance |
|---|---|---|
| A | Production `lidar_ndt::GicpRegistration` / small_gicp | Fixed: translation 0.25 m² on XYZ; each rotation component (1°)² |
| B | NDT-CUDA | Same fixed six-dimensional covariance as A |
| C | NDT-CUDA | Existing `engineering_R(fitness)` Legacy FitnessR callback |

NDT's returned transform is target-from-source and is passed directly as a six-dimensional external pose. Rotation residuals are evaluated on SO(3); no planar truncation was applied. NDT's ordinary objective/convergence result is retained. A/fixed-R reused the G2-E2 baseline with its diagnostic gate enabled: all 3,996 valid GICP candidates passed; the 138 remaining frames were already invalid/nonconverged, so that gate did not alter this baseline's posterior. B and C used the same matcher validity rules and did not add the optional NIS gate.

## Full three-lap outcome

The archived temporary `G_0.075` engineering reference has 3,635 valid paired frames. It is useful for relative comparison only: it is not independent surveyed truth, does not certify absolute accuracy, and curvature/lap group labels are derived from this same reference. The three estimate sequences are compared at the LiDAR Header time. The table reports immediate posterior error, not later frame-end output.

| Valid-reference metric | A: GICP/fixed | B: NDT/fixed | C: NDT/Legacy |
|---|---:|---:|---:|
| Accepted updates / 4,134 | 3,996 | 4,133 | 4,134 |
| XY RMSE (m) | **0.390** | 0.420 | 0.519 |
| Lateral RMSE (m) | **0.215** | 0.244 | 0.219 |
| Along-track RMSE (m) | **0.325** | 0.341 | 0.470 |
| Absolute lateral error p95 / max (m) | 0.429 / 0.722 | 0.474 / 0.742 | **0.433 / 0.668** |
| Longest valid consecutive run with \|lateral\| > 0.3 m | 70 | 106 | 59 |

NDT/fixed's full-course XY RMSE is 7.6% higher than GICP/fixed. NDT/Legacy's lateral RMSE is close to GICP, but its along-track RMSE is 45% higher, pushing XY RMSE to 0.519 m. The Legacy covariance produces noticeably larger gain: median lateral position gain is 0.0866 versus 0.0529 for GICP/fixed and 0.0508 for NDT/fixed. This is a feedback difference; it does not prove Legacy FitnessR is a calibrated covariance.

| Lap | Valid frames | GICP XY | NDT fixed XY | NDT Legacy XY |
|---|---:|---:|---:|---:|
| 1 | 1,204 | **0.374** | 0.411 | 0.518 |
| 2 | 1,235 | **0.401** | 0.440 | 0.573 |
| 3 | 1,196 | **0.394** | 0.407 | 0.457 |

## Difficult intervals and error source

| Frames / group | Metric | GICP/fixed | NDT/fixed | NDT/Legacy |
|---|---|---:|---:|---:|
| 800–899 | XY RMSE (m) | **0.546** | 0.658 | 0.732 |
|  | lateral RMSE (m) | **0.396** | 0.432 | 0.401 |
|  | along RMSE (m) | **0.375** | 0.497 | 0.612 |
| 1254–1405 | XY RMSE (m) | **0.389** | 0.407 | 0.565 |
|  | lateral RMSE (m) | **0.297** | 0.318 | **0.268** |
|  | along RMSE (m) | **0.251** | 0.255 | 0.497 |
| Normal segment 8, three laps | XY RMSE (m) | **0.237** | 0.377 | 0.391 |
|  | lateral RMSE (m) | **0.159** | 0.271 | 0.250 |

In frames 800–899, all three raw matches have nearly the same lateral RMSE (GICP 0.339 m, NDT/fixed 0.340 m, NDT/Legacy 0.339 m), but NDT has larger along-track error and worse posterior lateral error. In frames 1254–1405 raw lateral-match RMSE is again nearly identical: 0.192/0.189/0.187 m. Posterior lateral RMSE becomes 0.297/0.318/0.268 m; Legacy's stronger feedback lowers lateral error there, while its along-track RMSE rises to 0.497 m. This points to feedback and branch-specific prediction/matching interactions, especially along-track, rather than a large systematic raw-match lateral difference between GICP and NDT.

By reference-derived curvature phase, GICP vs NDT/fixed XY RMSE is 0.413 vs 0.441 m on straight sections, 0.435 vs 0.459 m at entry, 0.316 vs 0.378 m at apex, and 0.311 vs 0.325 m at exit. NDT/fixed also regresses all three normal segment 8 lap slices: XY 0.395/0.426/0.292 m versus GICP 0.246/0.271/0.186 m. The added aggregate plot shows these group-level values only; no coordinates or per-frame traces are published.

Peak/error audit: the largest posterior XY error for GICP and NDT/fixed occurs at frame 2030 (1.659 and 1.678 m); NDT/Legacy's largest posterior XY error is frame 1950 (2.095 m). The largest raw match XY error for all three is frame 2030 (2.282/2.494/2.498 m). These are isolated reference-relative peaks rather than evidence of a persistent multi-meter branch jump. NDT/fixed has one `MAX_ITERATIONS` invalid result at frame 2924 and accepts the next observation; it completes without a prolonged loss. NDT/Legacy reports objective convergence for all frames. GICP's 138 rejected candidates are matcher-invalid/nonconverged frames, not gate rejections.

## Runtime

All values are milliseconds. Matcher align is the matcher's own internal wall timing; roundtrip includes research bridge process/request overhead. Frame wall includes the full replay work, and these recorded offline values do not establish ROS publication latency or a hard deadline.

| Stage / percentile | GICP/fixed | NDT/fixed | NDT/Legacy |
|---|---:|---:|---:|
| Matcher align p50 / p95 / p99 / max | 4.87 / 35.25 / 45.49 / 58.60 | 2.17 / 3.31 / 4.03 / 5.08 | 2.10 / 3.25 / 3.96 / 5.37 |
| Full frame p50 / p95 / p99 / max | 17.15 / 50.64 / 61.08 / 192.76 | 13.40 / 17.11 / 18.78 / 185.02 | 13.86 / 17.03 / 18.34 / 185.60 |
| Matcher request roundtrip p99 / max | not recorded in reused baseline | 4.32 / 5.87 | 4.23 / 5.75 |
| CUDA Deskew p99 / max | 2.74 / 3.50 | 2.73 / 3.27 | 2.70 / 3.18 |

NDT cuts match-align p99 by about 11.3× and full-frame p99 by about 3.25× in these offline runs. GPU Deskew p99 is effectively identical across conditions. Both NDT frame maxima remain around 185 ms, so scheduler / IO / whole-pipeline long-tail remains unresolved. GICP matcher align timings and Deskew timings were extracted from its private per-frame transactions and Deskew metadata; an equivalent request-roundtrip timer was added only for the NDT reruns. The comparison supports matcher-align, Deskew, and complete-frame percentiles, but does not attribute all frame-level savings to NDT's kernel.

## Decision

**NO-GO to replace GICP with NDT-CUDA as the default matcher.** Fixed-R NDT is a promising real-time candidate: it completes all three laps, has a single quickly recovered nonconvergence, and substantially better match and full-frame p99. Its aggregate XY is within 3 cm of GICP, but this average hides repeatable normal segment 8 degradation, a 0.112 m regression in frames 800–899, and worse XY in every lap. NDT/Legacy is also NO-GO: stronger lateral feedback does not offset substantial along-track loss. Keep production GICP and its existing default unchanged; any NDT follow-up should target its localized error/seed/feedback behavior and validate on an independent reference, while preserving its runtime advantage.

This is a research comparison, not an absolute accuracy qualification, statistically calibrated covariance study, ROS2 online test, or 20 Hz hard-real-time acceptance.

## Reproduction

Set authorized `RESEARCH_ROOT`, `DATA_ROOT`, `OUTPUT_ROOT`, `REFERENCE`, and executable paths. Use the archived artifacts identified in `manifest.json`; create a new output directory for each run. Build the existing CUDA NDT matcher and production GICP bridge as in the linked G2 reports; do not substitute a frozen match cache.

```sh
export PYTHONPATH="$RESEARCH_ROOT/src:$RESEARCH_ROOT/experiments/phase_gicp15d_pipeline/g2_deterministic"
export G2_PRODUCTION_MATCHER="$BUILD_DIR/gicp15d_production_service"
cd "$DATA_ROOT"

run_g2n() {
  name="$1" backend="$2" covariance="$3" matcher="$4"
  python3 "$RESEARCH_ROOT/experiments/phase_gicp15d_pipeline/g2_velocity/run_g2v.py" \
    --input data/phase_ao_ap_validation/loop44.json \
    --bindings data/phase_ar_loop44_map0105/bindings_multilap.json \
    --target data/inputs/map_target.bin \
    --deskew data/phase_ar_loop44_map0105/runtime/deskew_service \
    --deskew-service data/phase_ar_loop44_map0105/runtime/deskew_service \
    --matcher "$matcher" --optical "$OPTICAL_C" \
    --private "$OUTPUT_ROOT/$name" --output "$OUTPUT_ROOT/$name.json" \
    --policy C --window multilap --matcher-backend "$backend" \
    --pose-covariance "$covariance" --g2e2-health --prune-geometry
}

run_g2n gicp_fixed gicp fixed_025_1deg \
  "$RESEARCH_ROOT/experiments/phase_gicp15d_pipeline/g2_deterministic/matcher_production_roi10.sh"
run_g2n ndt_fixed ndt_cuda fixed_025_1deg "$NDT_MATCHER"
run_g2n ndt_legacy ndt_cuda legacy_fitness "$NDT_MATCHER"

python3 "$RESEARCH_ROOT/experiments/phase_gicp15d_pipeline/g2_velocity/analyze_g2v.py" \
  --input data/phase_ao_ap_validation/loop44.json --reference "$REFERENCE" \
  --valid-key raw_valid_0.075 \
  --run gicp_fixed "$OUTPUT_ROOT/gicp_fixed" \
  --run ndt_fixed "$OUTPUT_ROOT/ndt_fixed" \
  --run ndt_legacy "$OUTPUT_ROOT/ndt_legacy" \
  --output "$OUTPUT_ROOT/analysis"
```

The exact machine paths, full per-frame CSV, sensor event logs, and coordinate trajectory remain private and are not part of this repository. The report is reproducible only when the authorized matching inputs and executable builds are available.
