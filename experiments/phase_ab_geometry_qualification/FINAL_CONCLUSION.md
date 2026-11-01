# Phase AB final conclusion

**Qualification-only gating exposes a real semantic defect but is not a usable filter repair.** It prevents the first unqualified covariance contraction and also removes every LiDAR update in the controlled continuous run. The negative outcome is retained, not tuned away.

## Answers to the research questions

1. Of 226 iteration-limit acceptances, **26** have joint final local numerical and strict-pair witness evidence. **28** meet the original final untruncated step-only tolerance; **48** have identical last/final pairs. An iteration-limit label does not itself establish failure: 26 only reached the certificate after the last accepted step and the old code never checked again. The remaining 200 are not certified; this does not prove every pose incorrect.
2. The first update uses 917 point-plane factors, 732 unique Targets, Huber influence and full state cross covariance. Its unchecked final-state information matrix is directly added to the prior, producing a **4546.4x variance contraction**, sigma Z **1.077427 → 0.015979m**, despite GN white norm **0.443617** and dual gradient **30.939004**. This reproduces the actual algorithm; it is not an independently inferred registration covariance.
3. Matrix arithmetic and coordinate transport are valid numerically. Observation trust is **NOT_VALIDATED**: unique-Target weights are correctly normalized, but normal neighborhoods overlap, shared errors remain unmodeled and association/optimization are not stationary. A 5m grouping sensitivity yields sigma Z **0.035201m**, without proving that alternative calibrated.
4. There are interacting mechanisms, not one universal cause. Missing final qualification is confirmed; some events are already converged at budget end, some retain pairs yet remain nonstationary, and first-frame fixed-factor refinement does not survive reassociation. Information counting beyond exact Target duplicates remains a modeling limitation. The evidence does not support changing Q or blaming all events on dynamic association.
5. An eligibility-only, atomic state/P rule was implemented and passed **8 new numerical tests plus 5 frozen implementation checks**. It keeps **32/233** old acceptances on their original frozen predictions. This verifies the rule's semantics, not localization quality.
6. The one admitted continuous test commits **0/284** LiDAR updates: 25 tentative updates fail eligibility and 259 terminate with low support. It exactly reproduces the saved inertial/wheel-only control. First sigma Z remains **1.077427m**, but maximum carried |vz| rises from **2.213607 to 3.480163m/s**, and end sigma Z reaches **51.782665m**. Zero accepted spikes are NOT improvement because all LiDAR constraints disappeared.
7. **No longer-window replay qualification.** Stop at this negative result. The next minimal investigation should establish a certified first-frame/local capture solution and consistent association before full posterior commitment, retaining the frozen prior and legal initialization contract. Do not solve the bootstrap failure by accepting nonstationary full information, loosening guards, cutting velocity, or merely extending every iteration budget.

## Qualifications

| Item | Status |
|---|---|
| Final objective/optimality reconstruction | PASS |
| Numeric information/posterior reconstruction | PASS |
| Geometric information trust/calibration | NOT_VALIDATED |
| Strict local association witness audit | CONDITIONAL: finite local tests, not neighborhood/global proof |
| Atomic eligibility numerical/frozen implementation | PASS |
| Continuous LiDAR constraint retention | FAIL |
| Actual combined motion/constraint improvement | FAIL: no retained LiDAR constraints; no accuracy ranking |
| Longer continuous replay eligibility | FAIL |
| Independent accuracy / statistical consistency | NOT_VALIDATED |
| Production candidate | FAIL |

Read [iteration diagnosis](ITERATION_LIMIT_DIAGNOSIS.md), [association evidence](ASSOCIATION_STABILITY.md), [first information](FIRST_GEOMETRY_INFORMATION.md), [eligibility design](UPDATE_QUALIFICATION.md), [frozen A/B](FROZEN_AB_RESULTS.md) and [continuous failure](CONTINUOUS_AB_RESULTS.md). [key_results.json](key_results.json) contains de-identified complete aggregate distributions; [FAILURES.md](FAILURES.md) preserves limitations. These reports stand alone without local-log access.


Public scope: de-identified research aggregates; no sensor timestamps, global trajectories, map points, equipment/extrinsic details or restricted production code are published. All state/P/factor arrays and event-level identities remain private. Numerical qualification is not independent accuracy or statistical calibration. Code/data bindings and reproduction prerequisites are in [manifest.json](manifest.json) and [REPRODUCE.md](REPRODUCE.md).
