# AV failure record

V1 guarded_single_use ran actual raw-cloud Deskew/CUDA with its own predictions. It stopped at881/1468 frames (~44.0s), after2s without an accepted observation:796 accepted,79 AMBIGUOUS whole-pose rejections,6 innovation inconsistencies. It did not reach right3 or finish right9, so no full repair is claimed. Three converged hypotheses sometimes differed predominantly longitudinally, and later by metres laterally; low fitness did not certify one peak. Raw hypotheses, source/state/P and all rejection logs are private and preserved.

V1's conservative all-or-nothing rejection lost useful consensus directions. V2 is a separately frozen mathematical redesign, not a relaxed gate or R-scale search. Its projections retain cross covariance and use original packet R. Failure data/config and the V1 commit remain unchanged. V1 is not a candidate default.

Initial guarded40 completed but first-frame CPU geometry indexing cost170ms. Later runs explicitly prewarm this index in the startup transaction. This lifecycle adjustment does not alter the measurement equations; startup and processing are reported separately.

V2 actual CUDA run stopped at1159/1468 (~57.90s):1112 updates,47 innovation rejections. Partial consensus resolved V1's whole-pose ambiguity starvation, but Gaussian hard-NIS gating under uncalibrated P/R produced another lockout. V2 full data/config/source are preserved. V3 changes the likelihood model, not the old cut probability or a hand-tuned R multiplier.

Initial V3 quadrature64 failed one numerical extreme-residual test before any GPU run (mean difference5.47e−7 versus128 at100m). Refined128/256 agrees within the declared tolerance; snapshot/log of64 are retained. This is numerical accuracy work; no GNSS result selected the order or precision prior.

B0 single-use packet ablation completed1468/1468 but FAILED the frozen repair screen: right3/9 improved, global75ms engineering XY RMSE worsened and some other fixed corners worsened. This refutes packet semantics alone as a complete accuracy fix.

V3 single lap completed1468/1468, but also FAILED the no-other-region-degradation/global-XY screen. Both difficult corners improved; this mixed result is not repaired qualification. No new R/precision-prior search followed that evaluation.

V3 first multilap failed after1480 accepted frames on pending scan history coverage after a505ms completion age exceeded500ms retention. NDT itself had converged. All JSONL/Source/state/P remain preserved. A subsequent explicit-history adapter attempt failed at initialization because a factory did not expose the replay's static `key`; zero processed frames. The corrected subclass and independent2.1s-retention rerun are separately identified. Neither failure is erased or included as a completed4134-frame run. See HISTORY_RETENTION.md.

One V2 offline evaluation command initially pointed to a nonexistent historical directory named `lap`; corrected to the frozen `fullpose100_lap` identity without changing the V2 run or selecting a different baseline. Both execution logs remain private.

The bounded2.1s retry completed3458/4134 native frames (172.85s), all center matches converged and all packets accepted, with113 partial-consensus events. It then failed the next Deskew-history assertion as backlog/model observation age reached2128.105238ms. Last several completion ages grew1971→2018→2073→2128ms. More retained history did not solve the processing/backlog feedback. Do not continue increasing retention or call this a completed three-lap test. Two full preselected laps plus638 frames of lap3 were processed continuously from one original initialization; no production-state reset was used. Prefix artifacts were recovered from immutable JSONL and labelled interrupted; missing full wall-time statistics are not invented. A final complete3-lap qualification remains FAILED/NOT_COMPLETED.

At the raw-engineering-reference75ms convention, preselected lap2 candidateXY RMSE0.666m versus frozen0.626m; the third-lap prefix0.517m versus0.422m. These held-out-in-time results reinforce the absence of a general accuracy fix. First-lap native states/Source/matches/P in the2.1s run exactly equal the500ms run; retention alone did not improve accuracy. Actual-source time correlation, data-dependent hypothesis selection and physical P/R calibration remain unresolved.
