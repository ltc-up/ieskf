# One evidence-driven association candidate

The original 13-start experiment is the AC2 CONTROL. After direct attribution, one candidate changes only the ordering of admissibility and nearest selection: select the nearest Target with a valid ORIGINAL local-plane fit, inside the ORIGINAL radius. No tuned k is introduced: when the nearest raw Target is invalid, exact radius candidates are evaluated lazily. The raw map, all normal-fit thresholds/neighbors, full frozen prior, Huber, target multiplicity weights, source support requirements, constrained optimizer, step limits and AB posterior eligibility stay unchanged.

This is an explicit ALGORITHM change to the selected factor set, not a claim that the original association rule or dynamic objective was preserved. It changes which factors exist and their multiplicities, so selected costs between variants must not be ranked as accuracy. The amendment was recorded before candidate outcomes. No dynamic-merit line-search change, hysteresis, soft weights, noise scaling or guard relaxation was combined with it.

At the SAME original cycle states, the identified Source is supported / unsupported by the original rule and supported / supported by the candidate, selecting the same valid Target both times. This isolates the intended mechanism without changed optimizer history. The specific invalid-nearest support hole is repaired.

However, nominal capture still fails: 60 outer steps lead to a FIVE-cycle, repeat error about 7.79815e-14 prior-white units at lag five. Final GN norm 0.002071709, dual gradient 0.092719956. On the inspected final edge, four Sources exit the support and four retained Sources change valid Targets. Each exiting Source crosses the unchanged 1m distance limit relative to its prior valid Target. The changed valid-Target normals differ by up to about 12.195202 degrees. Hard support and plane-selection changes therefore remain even after removing the invalid-nearest hole.

The one certified candidate endpoint is NOT an original-rule solution: querying the ORIGINAL rule at the SAME endpoint gives 871 support points, GN norm 0.031576 and dual gradient 3.121536; under candidate selection it has 1979 support points and GN norm 5.79575e-12. This prevents falsely claiming the original frozen-factor-selection problem was solved.

Candidate nominal support remains around 1,978 rather than collapsing, and all first-frame cumulative safety guards stay inactive. More factors or a different selected cost do not demonstrate a better physical pose. The new valid-nearest rule is a useful mechanism isolation, NOT a validated capture repair from the original prediction.

Nominal original/candidate diagnostic wall time is approximately 1190.443/3464.871ms, including 60 local solves and additional queries. Normal caches are immutable but shared across starts, so cold/warm timing differs. These are offline frozen timings, not 20Hz production performance.


Publication boundary: independent research code and de-identified numerical aggregates only. Full state/P, map/Source points, event timestamps, Target identities, equipment/extrinsic information and original project source are not distributed. No independent pose accuracy, calibrated covariance or production qualification is inferred from local numerical results. See [manifest.json](manifest.json) and [REPRODUCE.md](REPRODUCE.md).
