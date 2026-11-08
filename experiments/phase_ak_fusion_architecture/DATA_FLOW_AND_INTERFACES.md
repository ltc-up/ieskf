# Data flow and interfaces

Statuses: EXISTING means found in research or read-only local source, not validated production behavior; PROTOTYPE means synthetic only; REQUIRED means future contract; UNVERIFIED means actual availability/calibration unknown. No private message/config contents are distributed.

| Interface | Fields and semantics | Status |
|---|---|---|
| LocalMotion | measurement Header stamp, odom parent, base/inertial reference, continuous T, world/body velocity with frame, R,bg,ba, full15D error P plus chart/reset convention | State/predictor EXISTING; ROS message and calibration contract REQUIRED |
| LocalMotion health | per-source received/age/status/accepted counts, degraded motion reasons, covariance provenance, sequence/reset epoch | REQUIRED; minimal facts PROTOTYPE |
| NdtCandidate | Header scan reference stamp, map frame/version, base reference, pose, seed/deskew provenance, backend and convergence, fitness | Pose/score/backend convergence EXISTING; complete metadata REQUIRED |
| NdtSupport | matched/eligible counts, valid-map fraction, cells/tiles, spatial coverage, per-LiDAR IDs/availability and contribution | REQUIRED; current SDK export ability UNVERIFIED |
| NdtDirections | tangent frame, unit metric/length scale, basis U, rank/eigen evidence, finite-support uncertainty flags | known-plane diagnostic PROTOTYPE; production extraction/calibration REQUIRED |
| NdtModes | candidate cluster IDs/poses, score gaps, independent consistency, ambiguity reason, same-input correlation group | synthetic counterexample PROTOTYPE; bounded GPU search REQUIRED |
| NdtEligibility | publish candidate vs eligible observation vs accepted transaction, reasons, accepted subspace, input group | REQUIRED; minimal health facts PROTOTYPE |
| GlobalCorrection | τ and current publish stamp, map/odom epochs, T_map_odom, hypotheses, active source set, reason, anchor age, last decision, recovery state | composition PROTOTYPE; ROS/history REQUIRED |
| GlobalUncertainty | conditional vs marginal semantics, calibration flag, cross-correlation provenance, unresolved directions/modes | UNKNOWN defaults REQUIRED; real statistics UNVERIFIED |

Suggested independent message names: LocalMotion, NdtCandidateArray, SourceHealth, GlobalCorrection. These are contracts, not claimed installed message types. Standard Odometry alone cannot express ambiguity, calibration or accepted-subspace identity. Candidate stream is diagnostic; it does not authorize fusion.

Keep separate facts: received, fresh, solver-converged, support-sufficient, direction-rank, mode-count, consistent, eligible, accepted. Summaries: INITIALIZING, GLOBAL_TRACKING, PARTIAL, AMBIGUOUS, COASTING, UNAVAILABLE, RECOVERING. Local health and global health are independent. Recovery requires new fresh qualified evidence; a timeout clearing alone is not acceptance. Use both sensor timestamp and monotonic receipt age; future timestamps, clock resets, late/duplicate groups require explicit reasons.

Partial LiDAR: attach per-sensor received/used lists, time windows and valid extrinsic/version identifiers; recompute support and modes on actual participating cloud. Reusing prior all-LiDAR covariance is forbidden. Received cloud with zero accepted observations is MATCH_REJECTED or MATCH_UNAVAILABLE, not NO_CLOUD. Correlation group follows the same cloud through multi-starts and frontend transformations. A partial-direction acceptance records exactly which basis was consumed once.

Kistler gap → source missing/stale while valid wheel may continue under its own model; inconsistent wheel → reject source transaction and retain explicit local degraded state. NDT ambiguous + GNSS valid → use qualified GNSS dimensions and retain unresolved NDT modes. GNSS stale + NDT rejected → global coasting; continuous local output does not mean globally valid localization.
