# AV — Frozen candidate before evaluation

## Primary repair

Retain x=[p,v,R,bg,ba], full P, inertial/wheel equations, rear-axle lever, local SO3 pose chart, Joseph/reset and bounded delay replay. Original mode remains byte-for-byte selectable. New `single_use` policy interprets the inherited engineering mapping as **per unique packet**: `R_packet=engineering_R(fitness,smoothing_steps=1)`. Legacy uses5 to reproduce AR; no scale search, covariance calibration or GNSS optimization is performed. Production source establishes the factor's queue origin; ROOT_CAUSE contains the evidence.

This is a likelihood information-budget correction. It changes full-state K and posterior P naturally through the original Kalman equations. It does not multiply K, move means manually, erase cross-covariance or interpolate XY. Baseline Q/noise/Bias/vehicle parameters are unchanged. Statistical independence of different clouds remains an unverified model assumption.

## Primary guarded candidate

`guarded_single_use` applies the packet correction only after independent diagnostics:
1. Central matcher candidate_valid and converged retain their exact optimizer meanings.
2. Reuse AM MapSupport. Require full numeric rank and a majority of sampled Source to have nearby Target support. Rank/support are geometry eligibility, never a pose covariance.
3. Three fixed hypotheses share the same cloud group/Target: own predicted seed and two ±one-NDT-voxel lateral seeds with yaw±atan(voxel resolution/median Source horizontal range). Cluster with existing AM1m/.1rad convention; require three converged candidates in one cluster. This is bounded local agreement, not proof of a globally correct peak. Multiple basins are AMBIGUOUS; failed probes are INSUFFICIENT_HYPOTHESES, not automatically ambiguity or no cloud.
4. At the historical update instant, a full6D innovation/S χ² diagnostic at predeclared probability .9999 rejects model-inconsistent observations. The nominal statistical probability is not a measured false-alarm rate because R/P are uncalibrated.

Only the central result is fused once; hypotheses are not independent measurements and are not averaged into extra information. Partial geometry is explicitly withheld from a full6D update rather than assigned fabricated strong noise. A common wrong basin can still pass local agreement; this remains a known limitation. Missing/rejected observations preserve the original full state/P prediction and explicit coasting/lost status;2s without accepted NDT stops the research run.

The center/±seed matcher and geometry services persist for the run; Target and CUDA context are not recreated per frame. Logs separate optimizer result, local hypotheses, geometry, innovation gate and actual filter acceptance. No production TF or node is changed.

## Controlled branches and rollback

A: `legacy` exact AR behavior. B0: `single_use` causal ablation to isolate packet accounting. B1: `guarded_single_use` primary candidate. C: recorded oldEKF, an engineering reference rather than truth. Full-body state and covariance are always15D. Any serious degradation retains legacy as default and preserves failed candidates.

Validation order: reproduce legacy40; candidate40; independent continuous candidate1468 from original initialization; single_use ablation1468; if numerically/operationally viable,4134 continuous candidate frames without per-lap reinitialization. Fixed AS/AT corner IDs1/3/9 and all normal corners remain unchanged. Existing raw lap/multilap ranges and map0105 are frozen; GNSS never enters these branches. All0/75/150ms evaluation hypotheses are retained.

Repair GO requires both difficult corners improved at common valid engineering reference, no degradation in other fixed corners/global XY under the same convention (strict Pareto research screen), complete healthy input execution, no new numerical/association failures, and honest reference sensitivity. A mixed outcome is PARTIAL/FAILED against full repair, not a newly tuned threshold. Adjacent frames are not treated as independent statistical trials; per-corner/lap summaries and coverage are reported. Passing this research screen is not certified accuracy or production safety.

## Expected effects and risks

Packet information is restored when local evidence supports an update; increased gain should reduce history lag. P may shrink more, and wrong map peaks become more harmful, motivating separate integrity gates. Probes may fail or cause excessive withholding, and geometry/IPC increases cost. A χ² gate can reject correct observations when P/R are inconsistent. Every such failure is retained; thresholds or covariance scales will not be swept to improve GNSS metrics. If the initial hypothesis fails, redesign must follow the specific observed failure, not a hidden multiplier adjustment.
