# G2 feedback A/B: lateral offset in loop44

Base: `f2cb84b`, research branch `dsh/gicp15d-pipeline`, 2026-11-18. Production `lidar_ndt::GicpRegistration`, unchanged small_gicp, the 15D IMU/Wheel propagation, full 6D pose update, CUDA Deskew on the host GPU, and 10 m ROI reuse were retained. Both GICP runs completed 1468 own-state frames. The archived NDT Legacy and NDT Fixed-R runs used the same loop and filter implementation. No matcher or process noise code was changed.

## Mechanism found

The raw matcher is substantially closer to the temporary reference than the predicted Seed in the sustained-offset interval, but the posterior moves little per observation. For frames 1317–1405, the signed lateral error of the **raw match** is +0.160 m (GICP), +0.156 m (NDT Fixed-R), and +0.154 m (Legacy); the **Seed** is +0.573, +0.558, and +0.313 m, respectively. The immediate post-update base pose is +0.549, +0.534, and +0.297 m. Thus GICP and Fixed-R share essentially the same geometric correction opportunity; their large offset is mainly in the filter's prior and feedback history, not an independent GICP lateral misregistration in this interval. Legacy's different posterior follows its stronger historical updates. This is a causal hypothesis supported by the A/B below, not a claim that the archived runs isolate every matcher difference.

The baseline GICP translational R is 0.25 m² (0.5 m standard deviation), versus median 0.0378257 m² (0.1945 m) among the 1467 accepted Legacy NDT observations. In frames 1317–1405, GICP and Fixed-R have median lateral position gain 0.066 and 0.061; Legacy has 0.103. GICP's mean raw match-to-Seed lateral correction is −0.413 m, while its mean immediate posterior-to-Seed correction is only −0.024 m. The direct position innovation contributes −0.023 m; attitude-to-position cross gain −0.001 m and lever rotation +0.001 m. Raw match yaw correction is −0.473° per frame on average, but posterior yaw correction is −0.027°. Full 6D coupling matters for long-term stability, yet the instantaneous lateral correction here is dominated by the translational innovation rather than cross gain. P and R are both part of this outcome: mean prior lateral P is about 0.018 m² in GICP and 0.005 m² in the A/B; the gain does not scale simply as the inverse of R after repeated feedback.

The 800–899 peak has the same pattern: GICP mean Seed/raw-match/immediate-posterior lateral error is +0.501/+0.264/+0.488 m. Fixed-R is +0.491/+0.292/+0.481 m, Legacy +0.417/+0.291/+0.406 m. In frames 1254–1296, the reference-valid portion (30 frames) has GICP −0.277/−0.119/−0.261 m. These signs show successive over- and under-correction regions. Accepted conditional NIS alone does not reliably distinguish this persistent offset from a valid match.

## One-factor test

The A/B changes only GICP's *fixed translational observation variance* from 0.25 to 0.0378257 m², the archived Legacy accepted median measured without the reference. The three orientation variances stay at (1°)²; all six pose components, GICP, Q, propagation, gate, and ROI policy remain unchanged. This is a diagnostic transfer of an established engineering weight, **not** a calibrated GICP covariance or proposed production setting. The baseline remains the default; the new CLI option must be explicit.

All accuracy numbers use the archived `G_0.075` reference and its 1197-frame historical-valid mask. This reference is temporary, has an apparent longitudinal offset, and does not establish absolute accuracy.

| Metric | Baseline GICP | A/B stronger translation weight | Legacy NDT | Fixed-R NDT |
|---|---:|---:|---:|---:|
| Accepted / 1468 | 1406 | 1412 | 1467 | 1467 |
| Full-loop XY RMSE, m | 2.469 | 2.592 | 2.622 | 2.506 |
| Full-loop lateral RMSE, m | 0.361 | 0.314 | 0.295 | 0.365 |
| Full-loop absolute lateral p95, m | 0.795 | 0.727 | 0.669 | 0.745 |
| Full-loop absolute lateral max, m | 1.295 | 1.097 | 1.208 | 1.376 |
| Longest reference-valid run with absolute lateral >0.3 m, frames | 105 | 45 | — | — |
| Frame p95 / p99, ms | 48.96 / 61.19 | 49.65 / 60.22 | — | — |

| Frame range | Baseline lateral RMSE / p95, m | A/B lateral RMSE / p95, m | Baseline XY RMSE, m | A/B XY RMSE, m | Mean signed lateral baseline → A/B, m |
|---|---:|---:|---:|---:|---:|
| 800–899 (100 valid) | 0.681 / 1.192 | 0.621 / 0.990 | 1.996 | 2.215 | +0.516 → +0.410 |
| 1254–1296 (30 valid) | 0.630 / 0.902 | 0.562 / 0.836 | 2.605 | 2.867 | −0.540 → −0.443 |
| 1317–1405 (89 valid) | 0.669 / 0.858 | 0.451 / 0.714 | 3.294 | 3.319 | +0.659 → +0.424 |
| 1254–1405 (139 valid) | 0.631 / 0.861 | 0.479 / 0.750 | 3.090 | 3.194 | +0.352 → +0.232 |

In frames 1317–1405, the A/B Seed/raw-match/immediate-posterior lateral means are +0.340/+0.156/+0.324 m. Median lateral gain rises from 0.066 to 0.109. The continuous positive offset shrinks, while raw-match lateral barely changes. This supports an EKF feedback contribution. However, longitudinal RMSE rises from 2.442 to 2.573 m over the full valid set; its loss dominates XY. The A/B therefore fails a joint XY/lateral acceptance test. Matcher geometry, reference timing, and the prediction model remain possible contributors to the residual error. A different observation covariance would require independent calibration, not a sweep over this proxy reference.

## Execution and qualification

Both runs completed with full sensor history and no 2 s loss-stop. The A/B maximum accepted-observation gap was 0.151 s. The p99 frame time still exceeds 50 ms with ROI reuse. The intended next engineering test is to examine the along-track observation/Seed decomposition and measurement timestamp alignment, then evaluate a separately justified observation model against independent reference. No production default is promoted. **GO** for continuing 6D closed-loop GICP research; **NO-GO** for the A/B weight as a production fix or for strict 50 ms timing/absolute accuracy claims.

## Reproduce

Set `RESEARCH_ROOT`, `DATA_ROOT`, `OUTPUT_ROOT`, and `AW_REFERENCE` to authorized local paths, build the production bridge as in [the G2 production report](../g2_production_bridge/RESULTS.md), and run from `DATA_ROOT`. The output directory must not exist. Full inputs and per-frame trajectories stay private.

```sh
export G2_PRODUCTION_MATCHER="$OUTPUT_ROOT/build/gicp15d_production_service"
export PYTHONPATH="$RESEARCH_ROOT/src:$RESEARCH_ROOT/experiments/phase_gicp15d_pipeline/g2_deterministic"
cd "$DATA_ROOT"
python3 "$RESEARCH_ROOT/experiments/phase_gicp15d_pipeline/g2_deterministic/run_loop_det.py" \
  --input data/phase_ao_ap_validation/loop44.json \
  --bindings data/phase_ar_loop44_map0105/bindingslap.json \
  --target data/inputs/map_target.bin \
  --deskew data/phase_ar_loop44_map0105/runtime/deskew_service \
  --deskew-service data/phase_ar_loop44_map0105/runtime/deskew_service \
  --matcher "$RESEARCH_ROOT/experiments/phase_gicp15d_pipeline/g2_deterministic/matcher_production_roi10.sh" \
  --private "$OUTPUT_ROOT/g2_feedback_ab" --output "$OUTPUT_ROOT/g2_feedback_ab_summary.json" \
  --backend gicp --window lap --gicp-translation-r-variance 0.0378257
python3 "$RESEARCH_ROOT/experiments/phase_gicp15d_pipeline/g2_deterministic/analyze_production.py" \
  --candidate "$OUTPUT_ROOT/g2_feedback_ab" --legacy "$OUTPUT_ROOT/ndt_legacy" \
  --fixed "$OUTPUT_ROOT/ndt_fixed" --reference "$AW_REFERENCE" \
  --input data/phase_ao_ap_validation/loop44.json --output "$OUTPUT_ROOT/g2_feedback_ab_analysis"
```

The command generates local trajectory and lateral-error curves, per-frame errors, and aggregate metrics. The detailed Seed/match/posterior decomposition was calculated from each transaction's `prior`, `seed`, `ndt.final`, 6D `innovation`, `K`, `P`, `R`, and `delta`, with the saved vehicle-to-IMU transform. Published numbers are de-identified aggregates only.
