# Reproduce with separately authorized persistent inputs

Public repository contains only independent code/tests, aggregate statistics and de-identified figures. It does not contain restricted Deskew/CUDA implementation, raw bag/map/clouds/trajectories, transforms, vehicle sizes or full state/P. Resolve local bindings privately; hashes and frozen versions are in manifest. Run from repository root. Python numpy/scipy/matplotlib/PyYAML plus original GeographicLib `CartConvert`, authorized GPU runtime/driver and original external binaries are required. No download of another dataset or copied production source is needed.

```
export IESKF_CONFIG=data/history/phase_v/config/frozen.json
export PYTHONPATH=src
export PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1
python3 -m unittest discover -s tests
```

Use a NEW persistent ignored `--private` directory for every run; the wrapper refuses an existing one. The original input/bindings/Target have their historical relative names below; private bindings resolve actual raw clouds and extrinsics. Services persist and warm once per run. The original ROS2 production node/TF owner is never launched or modified.

```
python3 experiments/phase_av_filter_improvement/run_candidate.py \
 --input data/phase_ao_ap_validation/loop44.json \
 --bindings data/phase_ar_loop44_map0105/bindingslap.json \
 --target data/phase_ar_loop44_map0105/map0105_target.bin \
 --deskew data/phase_ar_loop44_map0105/runtime/deskew_service \
 --deskew-service data/phase_ar_loop44_map0105/runtime/deskew_service \
 --matcher data/phase_ar_loop44_map0105/runtime/match_fixed100 \
 --private data/phase_av_filter_improvement/NEW_lap \
 --output data/phase_av_filter_improvement/NEW_lap_summary.json \
 --policy robust_consensus \
 --candidate-config experiments/phase_av_filter_improvement/candidate_robust_consensus.json \
 --window lap
```

For fresh frozen reproduction select `--policy legacy --limit40` (write `--limit 40` as two arguments); compare original first40 native transactions, not wall-clock scheduling. `single_use` isolates packet accounting; `guarded_single_use` uses candidate.json; `guarded_consensus_subspace` uses candidate_consensus_subspace.json. V1/V2 failures and original configs remain separate, with no fallback pose fused as a measurement.

For continuous3laps replace binding with `data/phase_as_accuracy_quality/bindings_multilap.json`, window with `multilap`, use another NEW output directory. Original500ms run failed after1480 frames; explicit `--history-seconds 2.1` retry failed after3458. This is an expected qualification failure, not a successful reproduction target. Actual failure frame depends on CPU/GPU/runtime load; do not claim it is deterministic or enlarge history indefinitely to hide it. Preserve stdout/stderr and private JSONL. On interruption:

```
python3 experiments/phase_av_filter_improvement/archive_interrupted.py \
 --run data/phase_av_filter_improvement/NEW_interrupted \
 --history 2.1 --output data/phase_av_filter_improvement/NEW_interrupted_summary.json
```

This explicitly labels incomplete input/summary and does not fabricate missing transaction wall times. For actual completed runs, evaluate with the corresponding original AR/AS branch and frozen AS native-time axes:

```
python3 experiments/phase_av_filter_improvement/evaluate.py \
 --run data/phase_av_filter_improvement/NEW_lap \
 --baseline data/phase_ar_loop44_map0105/fullpose100_lap \
 --motion data/phase_ao_ap_validation/loop44.json \
 --as-evaluation data/phase_as_accuracy_quality/single_lap/evaluation.npz \
 --at-bindings data/phase_at_gnss_footprint_corners/bindings.json \
 --run-summary data/phase_av_filter_improvement/NEW_lap_summary.json \
 --private data/phase_av_filter_improvement/NEW_lap/evaluation \
 --output data/phase_av_filter_improvement/NEW_metrics.json \
 --label V3 --expected 1468 \
 --geometry data/phase_at_gnss_footprint_corners/visual_outline.json
```

Multilap uses baseline`data/phase_as_accuracy_quality/F_multilap`, AS`multilap/evaluation.npz`, expected4134, interrupted-summary path; optional vehicle geometry omitted because full multi safety is not qualified. All0/75/150ms hypotheses are always evaluated; no offset/extrinsic fitting. Historical/reference masks and raw-reference extension are separate. Numeric CSV/NumPy floats may differ by micrometres from AT cached arrays; historical1197 mask reproduces the comparison convention. Do not conflate it with1204 raw-valid first-lap samples.

```
python3 experiments/phase_av_filter_improvement/verify.py \
 --run data/phase_av_filter_improvement/v3_lap \
 --output data/phase_av_filter_improvement/NEW_verification.json
python3 experiments/phase_av_filter_improvement/feedback_metrics.py \
 --run data/phase_av_filter_improvement/v3_lap \
 --motion data/phase_ao_ap_validation/loop44.json \
 --as-evaluation data/phase_as_accuracy_quality/single_lap/evaluation.npz \
 --output data/phase_av_filter_improvement/NEW_feedback.json
python3 experiments/phase_av_filter_improvement/build_reports.py
```

The verifier also requires inherited AT/AU private artifacts to verify immutability; absent inputs must fail explicitly rather than fabricate a PASS. build_reports consumes committed aggregate metrics only and recreates public tables/plots. Private full overlays/corner curves/state/P/hypothesis logs remain in each run/evaluation directory. Original failed64-point numerical log/snapshot and unsuccessful CUDA candidates are retained. Final210-test success is distinct from that historical failed attempt.

Rollback: run original unchanged AR runner or `--policy legacy`; original`engineering_R` default5, full-state models, vehicle/config and existing AR/AS/AT/AU reports remain byte-identical. The candidate is not enabled by a production/default setting.
