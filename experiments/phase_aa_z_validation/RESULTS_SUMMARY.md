# Phase AA — vertical-velocity diagnostic and independent migration

## Research question and scope

Does the identified height-improved deskew input eliminate abrupt vertical-velocity requests or actual carried-state changes? Preserve the original inertial prediction, wheel observation, initial state/covariance, noise, map, geometry rules, robust loss, guards and iteration budget. No production source, parameters, topics or TF were changed.

Executed: one corrected-input continuous offline replay, one legacy-input continuous replay, and 284 legacy frozen-frame solver evaluations at the identical corrected-run prediction priors. Three independent migration smoke checks passed. Historical arithmetic/KKT campaigns, NDT/GICP matching and point-cloud deskew were not rerun. Instrumented corrected replay exactly reproduced historical output/IMU states, Bias, full covariance, LiDAR pre/post snapshots and decisions.

## Definitions

Velocity is world-frame velocity at the inertial-state origin. `proposed_delta_vz` is the last unconstrained absolute MAP candidate relative to a frozen frame prediction, before line search/guards. `accepted_delta_vz` is the actually committed corrected-minus-predicted velocity and is zero when rejected. `replayed_vz` is the carried offline state; nothing was published to ROS. The candidate is neither necessarily feasible nor converged.

## Corrected-input results

| Metric | Result |
|---|---:|
| Duration | 14.70 s |
| IMU / wheel updates / output states | 2,939 / 1,470 / 1,470 |
| LiDAR accepted / rejected | 233 / 51 |
| Accepted at iteration limit / genuinely solver-converged | 226 / 7 |
| Maximum absolute raw vertical-velocity proposal | 4.809311 m/s |
| Maximum absolute committed vertical-velocity correction | 1.081766 m/s |
| Committed corrections above 2 / 4 m/s | 0 / 0 |
| Raw proposals above 2 / 4 m/s | 2 / 1 |
| Maximum absolute carried vertical velocity | 2.213607 m/s |
| Committed absolute vertical-velocity correction P50 / P95 / P99 | 0.017716 / 0.166281 / 0.401419 m/s |

The largest raw proposal was rejected and did not become a state jump. Historical constrained diagnostic proposals in the 4–6 m/s range also must not be relabeled as committed filter states. Smaller committed changes and propagation drift persist; without independent truth their physical correctness is unknown. Observation thresholds above are descriptive, never new gates.

## First-update evidence

The first geometry update was accepted at `MAX_ITERATIONS`, not genuine convergence. Position-Z standard deviation contracted from 1.077427 to 0.015979 m, while vertical-speed standard deviation remained 1.858844 m/s. Its committed vertical-speed correction was 0.652306 m/s. Target-set turnover remained substantial, so low residual/information strength is not association convergence.

About 50 ms later, the Z–vertical-speed correlation was 0.985528 and conditional coefficient P(vz,Z)/P(Z,Z) was 19.417368 /s. A position-Z proposal near 0.250287 m accompanied a 4.809311 m/s raw velocity proposal; the frame was rejected. Full pose cross terms also contribute, so this scalar coefficient alone is not an exact six-axis attribution.

## Conditional legacy/corrected comparison

Same prediction/measurement configuration and map; only stored Source changes. Initial frozen-prior evaluation is separated from continuous recursion because later state/association histories diverge.

| Common-prior metric | Corrected | Legacy |
|---|---:|---:|
| Median absolute proposed Z correction, m | 0.013021 | 0.062050 |
| Median supported point-plane residual P90, m | 0.183996 | 0.731305 |
| Maximum absolute proposed vertical-speed correction, m/s | 4.809311 | 2.117930 |
| Continuous committed vertical-speed correction P95, m/s | 0.166281 | 0.293545 |
| Continuous maximum committed vertical-speed correction, m/s | 1.081766 | 0.912795 |

Typical indicators improve, worst cases do not uniformly improve. The historical Sources are hash-bound, but old per-sensor reference-time and pose-cache coverage/clamp histories are not fully recoverable. This is a CONDITIONAL pipeline comparison, not a strict isolated proof of one height-fix patch.

## Failures and limitations

All 51 corrected-run rejections have the original combined `LINE_SEARCH_OR_TRUST_LIMIT` reason; this is not evidence of one common cause. The 226 iteration-limit partial acceptances retain unqualified historical behavior. No thresholds or constraints were relaxed. Legacy run: 228 accepted, 56 rejected; 47 combined line-search/trust failures and 9 low-support failures; no solver-converged accepted frames.

The identified corrected chain was already used historically; AA did not generate a distinct later fix. Prior height improvement is user-confirmed rather than independently quantified here. Geometry proxies depend on pose, correspondence filtering and map support. Static-patch jitter, independent accuracy, probability calibration and live ROS/deskew scheduling were not established. Positive finite covariance does not prove statistical consistency. CPU timings are diagnostic offline timings, not production latency.

An executed legacy log inherited the reference event's Source-hash field. A separate immutable identity binding disambiguates actual legacy input hashes; the portable CLI now labels reference and replacement identities separately. This was a metadata correction, not a numerical algorithm change, and did not require another replay.

## Qualifications and decision

| Qualification | Result |
|---|---|
| Portable migration and historical numeric parity | PASS |
| Input-chain identification | CONDITIONAL: identified chain; generation metadata incomplete |
| Legacy/corrected causal comparison | CONDITIONAL |
| Reliable dynamic association/update convergence | NOT_VALIDATED |
| Independent cloud-height improvement | NOT_VALIDATED |
| Independent localization accuracy | NOT_VALIDATED |
| Statistical consistency / production qualification | NOT_VALIDATED |

There is no evidence that deskew improvement alone eliminates the covariance-amplification/partial-update mechanism. Prioritize convergence/association eligibility and credible geometry information/posterior handling, keeping the velocity model fixed. Do not clip or smooth velocity to hide the behavior.

## Reproduction and publication boundary

Public algorithms and small synthetic smoke checks are independently runnable; use the repository REPRODUCE.md. Complete real-input replication requires authorized event/initialization records, preprocessed Sources, target and frozen configuration supplied separately. The phase manifest binds these inputs and the research-code revision without distributing payloads, exact event timestamps, trajectories, device details or sensitive extrinsics. Full traces remain private, but all conclusions, key aggregate evidence, failures and limitations needed to assess this report are provided above.
