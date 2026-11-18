# G2-V: Wheel/Kistler velocity fusion in the production GICP + 15D loop

Base `b2ed91a`; branch `dsh/gicp15d-pipeline`; 2026-11-18. Three 1468-frame loop44 runs used the same production `lidar_ndt::GicpRegistration` adapter, host-GPU CUDA Deskew, 10 m ROI cache, 15D IMU prediction, complete 6D LiDAR update, and G0 LiDAR Q/R baseline. No GICP, state-dimension or LiDAR covariance code changed.

## Production input audit and implemented policy

Original `ekf_converter` subscribes to `/localization/wheel_velocity`, `/sensor/kistler/measurement`, and `/sensor/kistler/status`. It feeds both raw Kistler axes to `velocity_sensor_filter` (three-state filter, `sigma_a=1`, `sigma_r=0.01`, 1 s reset). Its timer chooses Wheel x and zero y when filtered Kistler x is below 1 m/s, otherwise filtered Kistler x/y. Kistler status and wheel thresholds alter output covariance (0.01 or 50) but do not consistently govern source selection. Receiving Wheel alone sets `linear_vel_received`, so the original path can publish zero without a valid Wheel sample. G2-V requires a valid recent Wheel for low-speed x, uses valid filtered Kistler x at high speed or when Wheel is unavailable, and never injects missing Wheel as zero. A bad Kistler packet immediately permits Wheel fallback. The existing `OpticalVelocity` models the Kistler lever and real vy; no vy=0 constraint is imposed. Its frozen engineering variance is 0.26 (m/s)² per axis, not independently calibrated.

The research motion JSON has 91,777 Wheel samples and no Kistler events. The raw loop44 bag contains both Kistler topics. We reused Phase AY's read-only extraction (52,160 raw measurements in its extracted span; 20,865 selected 10 ms events with past-received status), then ported the production filter over **all raw measurements before selection**. Mean absolute filtered-minus-raw change is 0.392/0.393 m/s in x/y. The offline filter uses measurement headers, not the original 10 ms ROS timer interpolation.

- **A:** Wheel vx only.
- **B:** Wheel vx plus filtered Kistler vy; isolates lateral observation.
- **C:** filtered Kistler vx at high speed, otherwise valid Wheel vx; Kistler vy whenever status is good. A Kistler vx update suppresses nearby Wheel vx, avoiding double fusion.

The lap is predominantly high speed: C logged 7,351 accepted Kistler xy updates, 7,347 Wheel suppressions, 9 accepted Wheel fallbacks, and 5 invalid Kistler status events. Low-speed switching was covered by focused unit tests but not by sustained low-speed loop44 driving.

## Correctly timed reference comparison

We compare Seed, raw GICP match and **immediate EKF posterior at the LiDAR reference time** with the archived temporary `G_0.075` reference, using its 1,197-frame historical-valid mask. Previous G2 reports compared the later `states.json` frame-end state with the earlier reference; the median state-to-reference offset in this replay is 54 ms (maximum 84 ms), which produces about 2 m apparent along-track travel at 40 m/s. Thus the old 2.469 m XY number is not directly comparable. The present reference still does not establish absolute ground truth.

| Valid-reference metric | A | B | C |
|---|---:|---:|---:|
| GICP accepted / 1468 | 1406 | 1413 | 1421 |
| Full-loop lateral RMSE, m | 0.326 | 0.322 | **0.212** |
| Along-track RMSE, m | 0.500 | 0.498 | **0.305** |
| XY RMSE, m | 0.597 | 0.593 | **0.372** |
| Absolute lateral p95 / max, m | 0.635 / 0.871 | 0.628 / 0.697 | **0.469 / 0.615** |
| Longest valid run with abs lateral >0.3 m, frames | 84 | 86 | **52** |
| Frame p95 / p99, ms | 48.23 / 59.38 | 51.17 / 61.66 | 49.64 / 60.37 |

Filtered vy innovation RMSE is 0.134 m/s in B and 0.132 m/s in C; A/B Wheel vx innovation RMSE is about 0.276 m/s. C improves lateral **and** longitudinal error. B's aggregate gain is marginal, and its 800–899 XY error worsens. Unfiltered preliminary B/C runs also completed; their aggregate outcomes are retained in the manifest as diagnostic only and superseded by the production-filtered runs.

| Frames (valid count) | Run | Lateral / along / XY RMSE, m | Mean Seed → match → posterior lateral, m | Mean prediction / LiDAR-update lateral error step, m/frame |
|---|---|---|---|---|
| 800–899 (100) | A | 0.521 / 0.556 / 0.762 | +0.501 → +0.264 → +0.488 | +0.0149 / −0.0125 |
| | B | 0.525 / 0.567 / 0.773 | +0.513 → +0.270 → +0.500 | +0.0137 / −0.0132 |
| | C | **0.397 / 0.375 / 0.546** | +0.377 → +0.270 → +0.367 | +0.0093 / −0.0098 |
| 1254–1296 (30) | A | 0.271 / **0.207** / 0.341 | −0.277 → −0.119 → −0.261 | −0.0157 / +0.0191 |
| | B | 0.123 / 0.238 / 0.268 | −0.116 → −0.110 → −0.107 | −0.0083 / +0.0117 |
| | C | **0.099** / 0.239 / **0.259** | −0.095 → −0.110 → −0.087 | −0.0081 / +0.0101 |
| 1317–1405 (89) | A | 0.585 / 0.723 / 0.930 | +0.573 → +0.160 → +0.549 | +0.0193 / −0.0235 |
| | B | 0.498 / 0.735 / 0.887 | +0.501 → +0.155 → +0.480 | +0.0180 / −0.0212 |
| | C | **0.335 / 0.191 / 0.386** | +0.241 → +0.151 → +0.228 | +0.0052 / −0.0128 |

At 1254–1296 the mean yaw rate is +20.5°/s and baseline body vx is 43.9 m/s; at 1317–1405 they are −18.5°/s and 45.5 m/s. The Kistler vy measurement includes the mounting lever's yaw-rate term, which `OpticalVelocity` handles. In 1317–1405, gyro bias z is −0.075°/s in A and −0.083°/s in C, while median LiDAR lateral gain **falls** from 0.066 to 0.052 because velocity observations reduce prior P. C chiefly improves the prediction/Seed before GICP, rather than strengthening LiDAR feedback. Filtered Kistler vx averages 0.463 m/s below Wheel vx in this interval and 0.218 m/s below Wheel in 800–899. These differences can accumulate into path error, but the temporary reference cannot certify which sensor is physically correct. The C raw match remains about +0.15 m lateral and immediate posterior about +0.23 m in 1317–1405, leaving matcher/reference bias and finite filter response unresolved.

## Curvature phases

The analyzer smooths temporary-reference tangent curvature by five frames. Below the valid-frame median absolute curvature is called straight; above its 85th percentile is apex; the remainder is entry or exit according to rising/falling curvature. This is an engineering grouping, not a surveyed corner classification.

| Phase (valid frames) | A lateral / XY RMSE m | B lateral / XY RMSE m | C lateral / XY RMSE m |
|---|---:|---:|---:|
| Straight (599) | 0.253 / 0.545 | 0.266 / 0.552 | **0.210 / 0.399** |
| Entry (222) | 0.397 / 0.683 | 0.344 / 0.659 | **0.263 / 0.384** |
| Apex (180) | 0.358 / 0.613 | 0.373 / 0.602 | **0.196 / 0.340** |
| Exit (196) | 0.395 / 0.630 | 0.397 / 0.626 | **0.163 / 0.291** |

The private analyzer output contains per-frame CSV, source decisions and phase-marked error curves; the committed code regenerates them from authorized inputs.

## Decision and reproduce

**Research GO for C:** high-speed filtered Kistler vx plus real vy, with mutually exclusive Wheel fallback, improves lateral, along and XY conditional error across the whole lap and every curvature phase. **NO-GO for B as a general repair:** its global improvement is marginal and the 800–899 interval worsens. **NO-GO for production default or strict 50 ms qualification:** the reference and velocity variance lack independent calibration, low-speed behavior needs real-data testing, and C p99 is 60.37 ms. Next validate C on another complete loop and independent speed/position reference, then address the remaining match-to-posterior gap.

Use the authorized `DATA_ROOT`, `BAG`, `AW_REFERENCE`, `OUTPUT_ROOT`, and `RESEARCH_ROOT`; build the production bridge as in [G2 production results](../g2_production_bridge/RESULTS.md). Run `extract_optical.py` from Phase AY with the bag and lap bindings, then `filter_optical.py` on its JSON. From `DATA_ROOT`, set `G2_PRODUCTION_MATCHER` to the built production service and `PYTHONPATH` to `RESEARCH_ROOT/src:RESEARCH_ROOT/experiments/phase_gicp15d_pipeline/g2_deterministic`. For each `POLICY` A/B/C run:

```sh
python3 "$RESEARCH_ROOT/experiments/phase_ay_localization_solution/extract_optical.py" \
  --bag "$BAG" --bindings data/phase_ar_loop44_map0105/bindingslap.json \
  --output "$OUTPUT_ROOT/optical_raw.json" --audit "$OUTPUT_ROOT/optical_raw_audit.json"
python3 "$RESEARCH_ROOT/experiments/phase_gicp15d_pipeline/g2_velocity/filter_optical.py" \
  --input "$OUTPUT_ROOT/optical_raw.json" \
  --output "$OUTPUT_ROOT/optical_filtered.json" \
  --audit "$OUTPUT_ROOT/optical_filter_audit.json"
export G2_PRODUCTION_MATCHER="$OUTPUT_ROOT/build/gicp15d_production_service"
export PYTHONPATH="$RESEARCH_ROOT/src:$RESEARCH_ROOT/experiments/phase_gicp15d_pipeline/g2_deterministic"
cd "$DATA_ROOT"
python3 "$RESEARCH_ROOT/experiments/phase_gicp15d_pipeline/g2_velocity/run_g2v.py" \
  --input data/phase_ao_ap_validation/loop44.json \
  --bindings data/phase_ar_loop44_map0105/bindingslap.json \
  --target data/inputs/map_target.bin \
  --deskew data/phase_ar_loop44_map0105/runtime/deskew_service \
  --deskew-service data/phase_ar_loop44_map0105/runtime/deskew_service \
  --matcher "$RESEARCH_ROOT/experiments/phase_gicp15d_pipeline/g2_deterministic/matcher_production_roi10.sh" \
  --optical "$OUTPUT_ROOT/optical_filtered.json" \
  --private "$OUTPUT_ROOT/g2v_$POLICY" --output "$OUTPUT_ROOT/g2v_$POLICY.json" \
  --policy "$POLICY" --window lap
```

Run `analyze_g2v.py --input ... --reference "$AW_REFERENCE" --run A "$OUTPUT_ROOT/g2v_A" --run B "$OUTPUT_ROOT/g2v_B" --run C "$OUTPUT_ROOT/g2v_C" --output "$OUTPUT_ROOT/analysis"`. The output directory must be new. Run `PYTHONPATH="$RESEARCH_ROOT/src" python3 -m unittest tests.test_g2v_velocity_replay tests.test_ay_optical`. Private sensor events, source logs, full trajectories and curves are excluded from Git.
