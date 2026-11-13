# Historical primary evidence audit

Original artifacts remain read-only outside Git. Hash-only inventory will be recorded in phase manifest.

- `optimizer_46250_ab/seed_robustness.csv`:84 rows,21 per strategy including CPU. Counted directly: CUDA_OFF15 correct/5 wrong/1 ambiguous; MT11 correct/4 wrong/6 ambiguous; GLL16 correct/5 wrong. Exact original seed remains wrong for all. Correctness uses original report proxy, not certified truth.
- `ndt_problem_capture_20260928_191919/results/trace_comparison_12632.csv`, CPU/CUDA line-search CSV and `line_search_root_cause.txt`: first divergence iteration12; CPU use_line_search=false accepts0.062937964 despite temporary score decrease, CUDA accepts0.005 satisfying Wolfe. This is not evidence of a broken MT interval solver or proof that epsilon0.001 solves the basin.
- Source12632 PCD5000 points, local target122060 points, saved full initial matrix and original Gaussian configuration are retained. Private exact-coordinate PCD→binary conversion performs no resampling or target substitution. Current-backend rerun must be labeled separately from historical reproduction because the AR backend contains derivative fixes.
- Historical DIRECT27 oscillation numbers remain UNVERIFIED after searching relevant report/log/iteration artifacts; no invented reproduction.
- Existing NIS historical warning: native-calibration Phase L report cites144/440 rejected and11.623m RMSE. This is a lead; primary Phase B artifacts still require reconciliation. q50 is an optimizer point-Gaussian pair gate, separate from filter innovation gating.
