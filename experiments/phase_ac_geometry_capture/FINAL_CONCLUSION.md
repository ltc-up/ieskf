# Phase AC final conclusion

**A feasible candidate-operator fixed point was found, but a reliable capture method from the original prediction was not. The first-frame failure is not explained by a four-step budget alone.**

## Direct answers

1. Under the ORIGINAL association rule, no qualified first-frame root was found in the 13 prescribed starts: 12 reach the same two-cycle and one lacks initial support. Under the one nearest-admissible candidate, ONE nonnominal start reaches a safe interior endpoint in 13 outer steps (GN norm 5.796e-12, dual gradient 1.457e-9). It is a root for the candidate factor selector, NOT the original selector. Endpoint association/optimality evidence exists; robust capture is not established, and 3/12 micro-queries alter factors.
2. AA's four accepted steps ended far from local qualification (GN 0.443617). More accurate fixed-factor work helps initially, but 60 outer steps reveal an assignment/support cycle. A single Source loses its factor when the raw nearest Target crosses from an admissible plane to an invalid plane fit, even though both distances are below the original radius. Fixed-factor solves are accurate, no first-frame cumulative guard is active, and full steps are accepted. Thus the failure is not a first-frame Bias cap or inner numerical solve defect.
3. More computation lowers some fixed objectives but does not resolve the original two-cycle. The one association candidate removes that exact hole but nominally enters a five-cycle due to remaining radius and valid-plane switches. Neither cycle proves an absolutely wrong physical pose; no independent truth was qualified. It does prove the requested fixed-point certificate is absent. Costs with different factors cannot establish geometric accuracy.
4. The experiment establishes one qualified candidate endpoint and periodic attractors, NOT multiple qualified local minima. The deterministic 13-direction design is not an exhaustive basin search or a probability estimate.
5. No tested minimal change reliably preserves continuous LiDAR constraints from the original prediction. Only one association mechanism was changed; the successful seed was not promoted to the initializer. No AB gate, Q/R, velocity model, data, map, weight formula or safety guard was retuned.
6. Actual Z-vz improvement remains NOT_VALIDATED: no AC continuous trajectory was produced. The isolated endpoint's conditional posterior can be submitted atomically in a frozen binding check, but sigma Z becomes about 0.009383m under the same uncalibrated model. This is not evidence that covariance confidence or velocity spikes improve.
7. Longer replay is NOT eligible. Next investigate association/support selection and the optimization merit/measurement model together, so a fixed-factor descent is compatible with re-association. Do not spend larger budgets on a repeated cycle, choose a favorable start after outcomes, average velocities, loosen guards or reduce information merely to obtain acceptance.

## Qualifications

| Qualification | Status |
|---|---|
| Input/prior/safety preservation | PASS |
| New solver/query interface checks | PASS: 14 unique new tests |
| Original nominal capture | FAIL |
| Candidate nominal capture | FAIL |
| Candidate feasible endpoint existence | PASS as a finite mathematical finding under changed selector |
| Robust initial capture / full-neighborhood stability | NOT_VALIDATED |
| Isolated conditional state/P transaction | PASS numerically; uncalibrated |
| Continuous LiDAR retention / actual Z-vz improvement | NOT_VALIDATED: not run |
| Continuous or longer replay admission | FAIL |
| Independent accuracy / statistical consistency | NOT_VALIDATED |
| Production candidate | FAIL |

Self-contained evidence: [three states](FIRST_FRAME_STATES.md), [original cycle](DYNAMIC_ASSOCIATION.md), [capture design/results](CAPTURE_RANGE.md), [constraints](CONSTRAINTS_AND_SOLVER.md), [single candidate](CANDIDATE_COMPARISON.md), [posterior semantics](POSTERIOR_TRANSACTION.md), [admission decision](CONTINUOUS_DECISION.md), [failures](FAILURES.md), and [key_results.json](key_results.json). Full local data are not needed to understand the claims and their limits; exact real-input replication still requires separately authorized payloads.


Publication boundary: independent research code and de-identified numerical aggregates only. Full state/P, map/Source points, event timestamps, Target identities, equipment/extrinsic information and original project source are not distributed. No independent pose accuracy, calibrated covariance or production qualification is inferred from local numerical results. See [manifest.json](manifest.json) and [REPRODUCE.md](REPRODUCE.md).
