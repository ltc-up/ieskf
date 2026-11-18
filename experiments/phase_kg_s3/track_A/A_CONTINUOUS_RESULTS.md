# A continuous results — partial research evidence

Both A1 maps completed the entire recorder-order 130–525 s input with legal 135 s initialization and their own posterior feedback. This runner processes actual recorded availability order but does not advance the recorder clock by computation latency. It certifies neither real-time scheduling nor 100 Hz publication.

| Quantity | 0105 | 1013 |
|---|---:|---:|
| Post-initialization scans | 7800 | 7800 |
| Legal matches after two initialization-spanning skips | 7798 | 7798 |
| A1 accepted | 7229 | 7770 |
| Original matcher-invalid rejection | 569 | 28 |
| Full cycle P50 / P95 / P99 (ms) | 27.316 / 37.082 / 43.219 | 27.055 / 34.310 / 38.784 |
| Full cycle maximum / over 50 ms | 60.355 / 8 | 59.513 / 2 |
| Accepted frames with every variance direction at floor | 7217 | 7770 |
| Fraction of variance directions at floor | 99.9677% | 100% |
| Final estimated z relative to initialization (m) | −35.173 | −0.986 |

The altitude differences are internal state changes, not independent truth. The 0105 run fails a normal-map stability claim: original Align first becomes invalid at 229.806 s, and no-valid-pair failures begin at 514.556 s. It is inappropriate to claim overall A1 improvement from small numerical covariance or low median cycle time. Nearly complete floor saturation also means these runs do not establish benefits from measured directional sandwich uncertainty.

Streaming independent audits found no Source-version mismatch, duplicate covariance update count, nonfinite/asymmetric/indefinite full P, or accepted original invalid candidate. Every accepted P was explicitly uncalibrated. SPD establishes numerical admissibility only. These checks examine update records at ingestion; any later relinearization changes require event-lifecycle auditing, not inference from initial accepted flags.

The two input ledgers share identity `b52a32bc8e91d11d1b4200e95cb3ed083f017e86b42c755e40e236515db1d940` (78974 IMU, 39499 Wheel, 7900 fused clouds). Their posterior-dependent Source/Seed chains may differ by design. Raw input equality does not imply identical conditioned Sources after feedback.

0105 runtime freeze is `02e3ee9`; 1013 is `3743aca`. The latter only adds passive R/chart/projection/floor-spectrum logs. Twelve same-input CPU cases verified mean and full P bitwise unchanged. In 0105, 7194 complete chart covariances were exactly reconstructed for full-rank/all-floor frames and their projected eigenvalues verified; the remaining 35 accepted direction matrices cannot be recovered and are explicitly missing. No matrix was fabricated. 1013 contains all required direction matrices for all 7770 accepted frames.

Artifacts are in archive `phase_kg_s3/track_A_geometry_fusion/A1_{0105,1013}_causal_serial_01`, alongside independent audit JSON and the explicitly qualified 0105 chart reconstruction. Both A2 maps have now completed with beta=1 and unchanged guards; see the complete evidence below. The 713 s and actual realtime qualifications remain separate pending work.

## A2 complete dual-map evidence

The complete restarted `A2_0105_causal_serial_02` run finished with the identical raw-input identity, 7798 legal matches, 7797 conditional MAP updates and one whole-scan numerical-domain rejection. The earlier `_01` directory contains a preserved 159-frame interrupted prefix after queued cancellation raced lock release. It has no final status, is not an algorithm-failure result, and is not merged into complete metrics. The restart uses the coordinator's passive lifecycle-audit freeze `7ba2d80`; the objective/solver/guards remain unchanged.

A2 0105 full cycles P50/P95/P99/max were 36.022/45.142/50.223/128.617 ms, with 86 over 50 ms. Wall time was 352.887 s. This still does not establish actual real-time publication. All 7798 lifecycle observations retained one numerical linearization; 7806 observer calls included equivalent cached calls. Independent recomputation of actual-prior hashes found no mismatch, full-P numerical anomaly, accepted-to-rejected transition, or prediction/P change on domain failure.

776/7798 frames (9.951%) rejected at least one candidate or trial because of the Euler-snap contract, although only one frame had no valid geometry candidate. The longest consecutive reference-time span with any domain rejection was 0.3007 s. Roll/pitch triggered 2024/470 events; two conservative path-proof-budget failures were retained. Thus normal-map candidate/trial domain limitations remain significant even when whole-frame rejection is rare.

The selected candidate counts were Prior 665, Prior-refined 2875, Align 1917 and Align-refined 2340. Every raw-Prior selection preserved the state mean but changed conditional P (median Frobenius change 0.00312); these are covariance-bearing geometric updates, not prediction-only outcomes. Limited iteration and candidate comparison do not certify a converged MAP minimum. In the common full-prior metric sqrt(gᵀPg), geometry/prior gradient ratios had median 12.57 and P95 253.58 among 7132 nonzero-prior-gradient cases; 665 zero-prior-gradient cases are reported separately. This is a scale diagnostic, not statistical calibration or an alternative online weight. Beta remains exactly one.

The largest accepted position increment was 0.8327 m, so the run did not naturally reproduce the historical roughly 2 m seed correction. Estimated z changed approximately −0.737 m from first committed output to final output; this is internal behavior, not independent accuracy. Reference accuracy and branch classification remain the coordinator's independent evaluation. A2 1013 also completed; the conditional old21 result is separately qualified below. The 713 s and actual realtime qualifications remain pending.

The complete A2 1013 run retained all 7798 legal frames: 7796 conditional updates and two whole-frame numerical-domain rejections. Its raw input identity equals the 0105 run. Independent lifecycle audit recovered 7798 distinct observations, all revision zero, with no prior-hash mismatch, invalid full P, accepted-to-rejected transition or state/P change on domain failure. All accepted covariances remain explicitly statistically uncalibrated.

Four-candidate selections on 1013 were Prior 552, Prior-refined 2863, Align 1958 and Align-refined 2423. Every raw-Prior selection preserved the mean and changed conditional P (median Frobenius change 0.003154). Candidate/trial numerical-domain rejection affected 685 frames (8.7843%); roll/pitch triggers were 1756/440, with five conservative path-budget rejections. The longest any-domain reference-time span was 0.24988 s. These rejected trials and the two prediction-only frames remain in complete statistics.

In the actual full-prior metric sqrt(gᵀPg), 1013 geometry/prior gradient ratios had P50 11.9898 and P95 224.846 over 7244 nonzero-prior-gradient cases; 552 zero-prior cases are separate. Selected raw Score P50 was 8989.259 with 6987 valid point–Gaussian pairs; prior contribution P50 was 0.174693. These objective/gradient scales do not establish statistical information or correct-basin preference. Comparing pairs and Score per pair across maps is necessary because raw geometric sums scale with support. Beta remains one.

1013 full-cycle P50/P95/P99/max were 35.777/44.296/49.151/124.076 ms, with 62 cycles over 50 ms. Wall time was 349.878 s; sampled RSS/VRAM peaks were 660066304 bytes / 1079 MiB. The largest accepted position increment was 0.69178 m, so neither complete A2 trajectory itself reproduced the historical roughly 2 m correction. The independently evaluated conditional old21 case retains 1.97625 m lateral correction, but uses model prior/P and is not evidence of continuous recovery. See A_WRONG_BASIN_ANALYSIS.md.

## Cross-map Score/support diagnostic

Selected lawful A2 frames number 7797/7796 for 0105/1013. Raw Score median/P95 are 8413.093/17903.290 versus 8989.259/16381.102; valid-pair median/P95 are 6966/13770.2 versus 6987/12806.5. Prior half-quadratic median/P95 are 0.1647/1.6748 versus 0.1747/1.6808. Each run has its own feedback-conditioned Source and P, so this is not a fixed-Source map-weight ablation or proof of localization gain. Absolute objective magnitude includes score offsets; it alone cannot establish gradient domination. Actual-P-whitened gradient diagnostics are the appropriate reported relative-gradient measure. Beta stays one.
