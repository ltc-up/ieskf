# Accuracy comparison

All numbers are engineering-reference differences, not certified truth. GNSS is offline only, with historical antenna-to-base conversion. The primary table evaluates reconstructed scan-time posteriors; `aggregate_results.json` separately evaluates the delivered current state at its actual timestamp (interpolated reference, no extrapolation). Empty/out-of-span samples are explicit. Proxy-good/bad labels do not establish true match correctness.

Full-source/100-iteration original15D control uses explicit legacy lifecycle; the held-out/35-iteration matched control uses the managed lifecycle. Both preserve the original motion/pose models. These execution controls are not asserted bitwise equivalent to historical timing. New methods must be compared to matched control. All use the reused GLL3 research matcher. The audited aa50 lidar_ndt source/YAML also selects GLL3; historical c647 OFF and an unverified deployed lidar_ndt_v1 path must be distinguished. Source/binary and deployed wiring equivalence are not asserted. Production six-state performance is not measured here.

| Run | Frames | XY RMSE 0/75/150ms (m) | Lateral P50/P95/max 75ms (m) | Longitudinal RMSE (m) | Yaw RMSE (deg) |
|---|---:|---|---|---:|---:|
| fullbaseline_lap | 1468 | 3.433 / 0.579 / 2.680 | 0.145 / 0.492 / 0.705 | 0.525 | 2.113 |
| matched_lap | 1468 | 3.431 / 0.577 / 2.682 | 0.145 / 0.496 / 0.704 | 0.523 | 2.113 |
| directional_lap | 1468 | 3.431 / 0.577 / 2.682 | 0.145 / 0.496 / 0.704 | 0.523 | 2.113 |
| optical_lap | 1468 | 3.389 / 0.517 / 2.711 | 0.118 / 0.442 / 0.600 | 0.465 | 2.112 |
| dual_lap | 1468 | 3.431 / 0.577 / 2.682 | 0.145 / 0.496 / 0.704 | 0.523 | 2.113 |
| gmf_lap | 1468 | 3.431 / 0.577 / 2.682 | 0.145 / 0.496 / 0.704 | 0.523 | 2.113 |
| robust_lap | 1468 | 3.416 / 0.582 / 2.703 | 0.155 / 0.546 / 0.785 | 0.521 | 2.115 |
| fullbaseline_multilap | 4134 | 3.671 / 0.571 / 3.016 | 0.147 / 0.462 / 0.864 | 0.519 | 2.187 |
| matched_multilap | 4134 | 3.672 / 0.571 / 3.015 | 0.146 / 0.465 / 0.868 | 0.519 | 2.188 |
| directional_multilap | 4134 | 3.672 / 0.571 / 3.015 | 0.146 / 0.465 / 0.868 | 0.519 | 2.188 |
| optical_multilap | 4134 | 3.647 / 0.517 / 3.027 | 0.135 / 0.430 / 0.667 | 0.468 | 2.204 |
| dual_multilap | 4134 | 3.672 / 0.571 / 3.015 | 0.146 / 0.465 / 0.868 | 0.519 | 2.188 |

## Fixed sections — lateral P95 at 75ms

| Run | Right 3 | Right 9 | Normal 5 | Normal 8 |
|---|---:|---:|---:|---:|
| fullbaseline_lap | 0.674 | 0.668 | 0.414 | 0.344 |
| matched_lap | 0.676 | 0.665 | 0.394 | 0.344 |
| directional_lap | 0.676 | 0.665 | 0.394 | 0.344 |
| optical_lap | 0.567 | 0.428 | 0.420 | 0.427 |
| dual_lap | 0.676 | 0.665 | 0.394 | 0.344 |
| gmf_lap | 0.676 | 0.665 | 0.394 | 0.344 |
| robust_lap | 0.709 | 0.665 | 0.394 | 0.340 |
| fullbaseline_multilap | 0.714 | 0.681 | 0.397 | 0.413 |
| matched_multilap | 0.717 | 0.686 | 0.392 | 0.411 |
| directional_multilap | 0.717 | 0.686 | 0.392 | 0.411 |
| optical_multilap | 0.607 | 0.442 | 0.395 | 0.436 |
| dual_multilap | 0.717 | 0.686 | 0.392 | 0.411 |

All ten sections, longitudinal distributions, yaw, validity masks/counts, proxy admission counts, branch counts, switches and both timestamp protocols are in the aggregate JSON. Admission labels use frozen 0.5m/2m proxy cutoffs with an explicit ambiguous band. They are not a new online gate or reliable correctness-rate estimate. Mode recognition latency against a true mode is UNKNOWN because independent mode labels are unavailable.

## Failure conditions

A scheme fails qualification if normal sections regress, timing/reference assumptions reverse its ranking, modes cannot be independently identified, expired data dominate, or inclusive runtime exceeds the budget. A smaller global RMSE alone is insufficient. The full failure inventory is retained alongside complete runs.
