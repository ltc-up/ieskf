# E2 — NDT and small_gicp competing candidates

The frozen experiment actually executes 1,468 paired requests and a separately identified historical-frame request. Each paired request uses identical input Source/Target/seed/header and crop contract. Both fit the same deterministic spatial fit partition, with the remaining cells used for one common validation cost. Algorithm-native voxel/covariance/neighbor/optimizer definitions remain different; identical raw requests do not imply identical objectives or correspondence support.

NDT uses the reused research CUDA implementation/configuration. small_gicp is the existing CPU service, not a CUDA GICP replacement. Its output includes right-chart information, source-tree/covariance, target-build, alignment and refresh timings. Fitness values between implementations are never compared. Convergence/finite-candidate flags are necessary, not labels of correct pose.

One common held-out cost is the mean clipped squared nearest-map residual, with a declared engineering scale. Next-scan analysis transports each candidate by the frozen seed-relative motion and evaluates the next held-out cloud. This is useful temporal validation, but both the transport and map are correlated with the current experiment; it is not an independent truth source. The online Gaussian bank does not read the offline GNSS reference.

Translation/rotation separation is reported as candidate difference, not certified different basin membership. Favorable offline proxy errors and genuinely complementary geometry must be distinguished. The complete-loop dual Shadow keeps NDT as the actual updating backend. The GMF experiment instead lets each branch use its own prediction, deskew and assigned backend. GICP residuals are not directly inserted as point-level ESKF observations.

The reused service binary has a recorded SHA. An archived C++ recovery snapshot contains a known dangling-expression problem and does not establish exact source correspondence to that binary. This provenance limitation forbids asserting a clean production-ready implementation or exact source rebuild. Restricted legacy source/binaries are not redistributed.

## Results

`frozen_summary.json` contains paired convergence, separation, common validation and offline proxy sensitivity. `ACCURACY_COMPARISON.md` compares true own-feedback loops; a fixed-seed improvement cannot substitute for those results. CPU/GPU resource samples cover only their explicitly recorded monitoring interval. Any full-cycle overruns or shared wrong peaks remain negative evidence; no backend replacement is approved.

Final frozen results: NDT eligible 1468/1468, GICP eligible 1395/1468. 1073/1468 exceed the engineering pose-separation test. Translation differenceP50/P95 is0.143/0.488m; this does not establish separate attraction basins.

At75ms, NDT/GICP reference RMSE is0.612/0.620m. Under the strict0.5m-good/2m-bad proxy labels, GICP-good/NDT-bad=0 and NDT-good/GICP-bad=0; both-good477, both-bad1, with the remaining cases ambiguous or missing. Thus these labels provide no demonstrated rescue of a clearly bad NDT mode. Native-validity-constrained common-cost selection has conditional reference RMSE0.605m; it is an offline correlated diagnostic, not a tested closed-loop improvement. The next-scan raw common-cost comparison favors GICP490/1467 times, without establishing correct pose.

The single historical12632 sample is retained in HISTORICAL_CASE_SUMMARY.json, including both convergence outcomes and common validation. One sample with no independently verified correct-mode label cannot establish backend complementarity. Dual Shadow does not improve final selected NDT single-/three-lap accuracy, while increasing compute. Keep small_gicp as an optional research candidate source; replacing NDT or treating agreement as independent evidence is NO-GO.
