# Synthetic validation — executed

7 new unittest cases PASS;94 existing public tests PASS. Deterministic Python/NumPy/SciPy experiment, no real input or restricted source. See results.json for all aggregate outputs, prototype.py and test_prototype.py for exact construction. The independent15D experiment reuses State, InertialPredictor, inject and chart_map from this repository.

## 1. Local/global containment

Two identical initial local states and synthetic full P;100 steps at.02s with existing inertial propagation. Direct estimator also receives normal pose, wrong Z(+8m), wrong XY(+10,−5m)/yaw(.4rad), then returning position-only GNSS. Full direct updates intentionally bypass engineering guards: this is a stress counterexample, not AJ replay or realistic perturbation-size estimation. Later linearization after gross error is not an accuracy benchmark.

Layered local position/velocity/bias equals no-map control exactly. Global translation errors at four anchors:0,8,11.18034,0m; returning GNSS position leaves .4rad rotation error. Direct velocity correction norms0,15.3203,49.0601,42.9093m/s and combined bias norms0,6.53866,14.5333,15.8798 illustrate propagation risk only; bias norm combines heterogeneous units and is not a sensor-performance metric. No map gate is tested in this experiment.

Unobserved local sigmaZ grows1→3.00444m over2s under synthetic prior/noise. No map-induced local jump does not imply height observability. Correction is C=M O^-1 at matching time; returning GNSS changes translation only. No valid global covariance is fabricated: correction and odom share local error and NDT depends on local seed/Deskew. Prototype explicitly labels UNCALIBRATED_CORRELATION_UNKNOWN (ASCII value in results.json). Covariance of global error would need cross terms; this test verifies isolation, not posterior accuracy.

## 2. Full / Schmidt / projected Gaussian

Known P=[[2,.6,.4],[.6,1.5,.3],[.4,.3,1]], H=[[1,0,1],[0,1,0]], R=diag(.2,.3), residual[1,−.5]. Protect state2; active0,1. Full correction[.572770,−.363850,.356808]; Schmidt[.572770,−.363850,0]. Protected variance remains1; active–protected cross changes from[.4,.3] to[−.439437,−.002817]. Full posterior matches independent information-form inverse to1.11e−16. Schmidt matches augmented joint-error covariance exactly and an independent scalar closed-form example to tolerance. All covariance matrices symmetric to5.56e−17, minimum eigenvalues>.0934.

Trace full1.31127 < Schmidt1.80141 < projected2.36750. These are different estimators, not competing claims of true accuracy. Projection keeps first measurement only; another null-channel test ensures unobserved independent state keeps its variance. Same-scan duplicate test retains variance.5 after two attempted consumptions rather than falsely shrinking to1/3. Assumptions: known linear Gaussian model and independent observation noise; unknown correlations are discussed mathematically, not implemented as a calibrated real fusion scheme.

## 3. Geometry / multiple peaks

Known infinite-surface Jacobians give ranks3,5,6 for one/two/three normals. Removing two surfaces lowers rank6→3; support fraction1/3. This simple missing-block construction does not test a finite road map. A1D Gaussian-cell NDT-style likelihood has separate optima near0 and5m, equal cost (difference<1e−10) and both curvature≈25. Other axes can be independently constrained without resolving this translation alias. Good local curvature and low residual therefore cannot establish the correct peak. Not a CUDA-NDT execution or reproduction of X-ICP/LF-GICP/SuperLoc algorithms.

## 4. Source facts

11 prescribed events distinguish ACCEPTED, ACCEPTED_PARTIAL(rank3), AMBIGUOUS, STALE, MISSING, REJECTED_INCONSISTENT, NOT_CONVERGED and DUPLICATE; fresh GNSS recovery accepts again. Ambiguous NDT has received=true, accepted=false; missing Kistler has received=false. Inconsistent wheel does not mean missing wheel. A separate ambiguity-to-valid recovery test passes. Minimal eligibility only: proposed global lifecycle, hysteresis, actual per-LiDAR extraction and delayed scheduling remain future work.

No experiment establishes real localization accuracy, statistically calibrated uncertainty, reliable global peak choice or production readiness. See FAILURES.md for negative results and corrected draft assumptions.
