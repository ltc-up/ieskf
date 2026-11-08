# Architecture comparison and decision

Priority: **B first**, C only for a separately justified need, A retained as comparison. Recommend local15D + NDT/GNSS global map correction + explicit integrity manager. This is a containment and operational choice, not a proof of minimum estimation error.

| Criterion | A: one15D ESKF | B: local15D + map/odom | C: selective/Schmidt15D |
|---|---|---|---|
| Loose coupling | Pose frontend, common state update | Separate local and global responsibilities | Pose frontend, constrained state update |
| Physical correlations | Full joint P | Full local P; inter-layer correlation must be modeled or marked unknown | Full joint cross blocks retained |
| Wrong NDT peak | Can change all states | Can corrupt global output; local trajectory protected | Active states still vulnerable; protected means held |
| Z/vz isolation | None by default | No map-induced local correction; wheel-induced effects remain | Only chosen protected states; lost information |
| GNSS | Direct physical antenna observation | Alternative anchor with datum/lever/time contract | Active/consider design must include antenna model |
| Map holes | Prediction growth, source rejects | Global coasting/partial/unavailable; continuous odom | Reduced subspace or no update |
| Information meaning | Requires calibrated R and correlations | Conditional correction uncertainty separate from global accuracy | Requires proper S, cross blocks and constrained covariance |
| CUDA-NDT reuse | Full | Full | Full |
| Compute | Small fixed15D | Small local15D +6D correction/hypotheses | Small fixed blocks; more bookkeeping |
| Engineering | Simple update; hard integrity problem | Moderate interface/time/history changes | Hard state partition/reset/consistency verification |
| Bias | Joint15D estimate | Local15D remains; no map bias correction by default | Protected biases lose measurement benefit |
| Deskew feedback | Map corrections can alter future deskew | Local odom drives deskew; map only seeds candidate search | Depends on protected-state choice |

A could work with credible observations and robust source selection, but AJ demonstrates current gates/R are insufficient; do not claim A inherently wrong. B reduces damage without certifying global matching and can increase drift by giving up map-based bias information. C is a valid estimator only with stated constrained-error statistics, not gain masking followed by an optimal-K covariance shortcut. No immediate need to add C to B's first implementation: an explicit feedback-free service boundary is simpler to verify. Later map-to-local feedback needs joint delayed-state/consider modeling or conservative fusion under validated marginal covariances.
