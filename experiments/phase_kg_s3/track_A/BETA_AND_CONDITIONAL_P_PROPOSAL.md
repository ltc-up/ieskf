# Score scale and conditional covariance — proposal, not yet real-data freeze

No real-data A2 run has started. `Contract.beta=1/64` in the numerical prototype is a proposal. Neither 64 nor 1 has a verified statistical interpretation for the production multi-neighbor Score. No fitness or reference-derived scaling is used. A requested contract review is pending before real-data MAP.

For the exactly quadratic scalar illustration `F(z)=z²/(2P)+β C(z−d)²/2`, the solution is `z=d·βCP/(1+βCP)`. This illustration has no correspondence to a selected real frame and does not establish real-data accuracy.

| P (m²) | Score curvature C | d (m) | β=1 result (m) | β=1/64 result (m) |
|---:|---:|---:|---:|---:|
|0.04|4000|2|1.98758|1.42857|
|0.01|4000|2|1.95122|0.76923|
|0.04|50000|2|1.99900|1.93798|

A temperature of 1/64 can suppress the desired approximately 2 m correction even while producing a numerically valid MAP. β=1 preserves the Score's raw declared scale but can dramatically over-weight densely repeated support. A fixed β does not distinguish a wrong basin from a correct one. The recommendation is to expose and record β explicitly and, subject to coordinator/user decision, freeze β=1 as the literal unnormalized Score-MAP primary model; a separately named β=1/64 sensitivity model may diagnose this unresolved scaling assumption. This is not a recommendation to select β from reference error or to pretend either model is calibrated.

## Conditional P is a distinct engineering model

Let `D=diag(0.5 m,0.5 m,0.5 m,π/60 rad,π/60 rad,π/60 rad)` and `Cq=−β sym(H_score)` in the production additive translation/RxRyRz chart. Diagonalize `D Cq D=U diag(λ) Uᵀ`. Use `Iq=D⁻¹ U diag(clip(λ,0,1)) Uᵀ D⁻¹`. Zero/negative curvature gets zero information; the information ceiling is explicit. This clipping only constructs conditional P and never changes Score, gradient, MAP Hessian, MAP objective, or candidate selection.

With frozen-prior error coordinate `z` and pose-chart Jacobian `J=∂q/∂z`, use `Qz=[(P−)⁻¹+JᵀIqJ]⁻¹`; transport to the optimized right-error chart by `Q=chart_map(z) Qz chart_map(z)ᵀ`. No damping matrix enters this formula. Full 15×15 matrices and cross terms are preserved. This P is not a statistically calibrated posterior; curvature caps and β cannot certify wrong-basin safety.

A1 instead implements a spatial-cluster Score sandwich using fixed 8 m source-frame blocks, with point-to-multiple-Gaussian contributions kept inside each point's block. `C=−sym(H_score)`, `M=K/(K−1)Σ_b(g_b−ḡ)(g_b−ḡ)ᵀ`, and the identifiable-subspace covariance is `C⁻¹ M C⁻ᵀ`; dimensionless variance is floored at one in the same D units. Null curvature directions remain absent from the observation. This is a separately labelled conditional engineering R until independent calibration, not a claim that raw Score curvature is statistical information.

## User decision superseding the proposal

The user explicitly froze **β=1** as the sole A2 complete-run constant. No β=1/64 data run, sweep, per-map, per-Fitness or per-reference adjustment is authorized. The prototype default and production-facing research factory now enforce β=1. The preceding scalar examples remain historical mathematical explanation, not additional experimental variants. Score magnitude, valid pair counts, prior quadratic, both objective contributions, both gradient vectors/norms/angle and full-P qualification are recorded at every retained candidate and the selected result. If geometry overwhelms the prior, report that result without changing β.
