# E1 — Coupled geometry and lateral-velocity diagnostics

Implemented switches: diagnostic-only `--directional`; separate experimental `--project-directional`; existing optical research policy `--optical xy`. No production Q/R, map, TF or optimizer is changed.

## Physical charts

For q=[s,d,L·yaw], information is BᵀHB with full XY-yaw cross terms. NDT's maximum-score Hessian is symmetrized and negated before transport from its Rx-Ry-Rz Euler chart to physical world yaw. GICP's positive right-rotation/right-translation information is transported using Rᵀ world axes. L=5/10/20m is logged; L=10m is the declared projection setting. Independent finite-difference tests check both chart transforms. An unscaled mixed-unit 6×6 condition number is never a rejection test.

The optional projection retains eigenvectors above max(1e−12, 1e−3·largest eigenvalue), then retains the z/roll/pitch complementary rows. It transforms the entire observation and R with the same row map. It does not use inverse Hessian as statistical covariance, retune the original measurement noise, or reject an entire scan for one weak eigenvector. This threshold is an exploratory direction-selection setting, not calibrated integrity. A well-conditioned wrong peak remains possible.

The runtime tangent comes from the branch's own predicted map velocity; GNSS/track labels do not enter this calculation. Post-analysis uses frozen section labels only for comparison. Lateral retained energy is the squared projection of the lateral basis into the retained eigenspace; it indicates algebraic information retention, not correctness of the retained correction.

## Multiple LiDARs

The offline lineage diagnostic splits the actual CUDA-native point stream by original sensor counts, verifies total counts, samples up to 512 points per LiDAR, estimates common-map local normals and builds coupled [s,d,L·yaw] point-to-plane information. The per-LiDAR matrices sum exactly by construction. This is a pre-voxel sampled geometry diagnostic, NOT exact attribution of the fused NDT objective Hessian. Shared map errors/correspondences remain correlated; algebraic additivity is not independent statistical evidence. No per-sensor repeat NDT matching is inserted online.

## Lateral motion

The 15D baseline already permits a full 3D velocity. Its scalar wheel observation constrains a forward component; it does not explicitly set vy=0. The production planar six-state model is a different case. At the base point, body velocity is Rᵀv_IMU − (omega−bg)×r_base,IMU and beta=atan2(vy,vx). Diagnostics include ay−r·vx. These can indicate mismatch but do not by themselves distinguish real sideslip, attitude error, optical mounting/delay or accelerometer bias.

The raw optical update predicts sensor-frame velocity from the IMU-state velocity plus angular lever-arm motion, using the existing sensor transform. Raw sensor vy is not silently treated as rear-axle vy. The XY candidate reuses AY's explicit engineering variance/status/time-bin policy and suppresses the redundant wheel forward update while optical data are recent. There is no new tire model, adaptive-Q change or hidden 16th state.

Historical AY evidence is retained: conditional 75ms gains at difficult corners coexisted with normal-section-8 regression and 150ms ranking reversal. Lower optical residual does not prove lower physical map error; no new causal claim explains that discrepancy. This phase remeasures paired residuals under the declared transforms but does not independently re-calibrate them.

## Results

The complete-run aggregates, per-section projection energy, optical residuals and multiscale eigenstructure are appended from `diagnostic_summary.json`; accuracy and latency are separately reported. Preserving a lateral eigenvector alone is not proof that correct lateral updates were retained. Independent match labels are unavailable; reported good/bad acceptance counts use an explicitly imperfect offline engineering proxy.

## Executed final evidence

The final direction-selection control retains all three coupled directions on **1468/1468 single-lap and4134/4134 three-lap frames**. Lateral retained energy is1 to floating-point precision. Consequently it is algebraically equivalent to the full observation in this dataset; no real weak-direction suppression benefit has been established. The synthetic end-to-end regression does suppress an explicitly unobservable longitudinal component while preserving the lateral mean/P update. That test is not a real-data accuracy result.

Normal5/8 proxy-good observations accepted: single lap79/79 and98/98, three laps238/238 and275/275, identical between matched and direction-selected controls. Proxy cutoffs and missing/ambiguous cases are explicit in directional_current_review.json. These observations were retained, but actual correct-match labels are not available.

151 sampled multi-LiDAR frames pass the point-lineage/sum check. Their mean information traces are front5215.665, left3095.674, right1720.768 in this declared sampled normalized metric. The values depend on range, normal estimation and sampling; they are not calibrated sensor weights or grounds to remove a LiDAR.

The optical XY candidate gives single-lap75ms XY reference RMSE0.517m versus matched0.577m, but normal8 lateralP95 increases0.344→0.427m; normal5 increases0.394→0.420m. Three-lap normal8 increases0.411→0.436m. At150ms the single-lap global ordering reverses (optical2.711m, matched2.682m). Therefore no universally improved sideslip prediction model is accepted. Current diagnostic optical innovations are in-sample for the XY filter; they cannot substitute for independent physical prediction validation. Historical AY held-out analysis remains separate.
