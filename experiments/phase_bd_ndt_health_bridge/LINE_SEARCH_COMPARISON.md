# Line-search study

See [P0 source audit](P0_CONVERGENCE_AUDIT.md), [19084 frozen matches](P0_FROZEN_COMPARISON.md), [true closed loop](P0_CLOSED_LOOP_COMPARISON.md). Current source/default path is GLL, not OFF. More-Thuente suppresses some observed numeric cycles but can retain a wrong mode. Primary21-start CSV independently counts15 correct OFF,11 MT,16 GLL, with ambiguous states retained; the comparison is one frame and one perturbation grid. Original12632 trace documents MT satisfying Wolfe at0.005 while CPU accepts a temporarily lower-score0.062938 step with search disabled. These are independent mechanisms: fewer iterations can coexist with worse branch selection.

P0 frozen target/seed grid and true AY closed-loop runs now exist. DIRECT27 historical37→3 /5.10→2.16ms report has not yet been found; do not claim it reproduced. The current evidence supports conditional further study, not unconditional MT enable/disable or a single epsilon recommendation.

Actual neighbor interaction: DIRECT7 OFF/MT historicalHIGH coverage20/21→6/21; KD0/21→0/21. Whole-lap frozen RMSE changes .6248→.5910(DIRECT7), .6164→.5851(KD). Numerical improvement and wrong-mode selection therefore coexist. See NEIGHBOR_SEARCH_COMPARISON.md.
