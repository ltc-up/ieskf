# Phase X: Joint objective and nonlinear solver audit

Separate exact Huber objective, local IRLS surrogate and reassociated objectives. Fixed-association full-state objective/Jacobian behavior was numerically consistent. A limited line-search grid omitted some feasible small steps; a minimal correction produced partial decreases without establishing genuine convergence. This is a recorded unsuccessful convergence repair, not an accepted production improvement.

The original test plan, source, parameters, full evidence, failure logs and input hashes are retained unchanged in ignored `data/history/phase_x/`. This is a separate public redaction; it contains no real sensor results. Portable independent modules and transformation hashes are listed in `docs/CODE_PROVENANCE.json`. Private extraction/driver/configuration dependencies remain local and are not independently reproducible from this repository.
