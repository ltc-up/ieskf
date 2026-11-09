# Reproduction and private-data boundary

Checkout the AW branch/worktree based on c647ac3. Existing AR/AS/AT/AU data, original raw binding files, external authorized Deskew/NDT binaries, map0105 Target and the MIT small_gicp vendor must be separately available. No private point clouds, maps, tracks, transforms, vehicle sizes or restricted binaries are published. Do not pop/drop stashes or modify production. Original production HEAD aa50e354 and relevant stash c964ce45 remain assets, not code to apply to15D.

Use an authorized private history root as `data` (directory or symlink) and a NEW AW private persistent directory on the user-selected disk. Link it at `data/phase_aw_native_gicp_fusion/private`; do not overwrite any existing link or output directory. `/data` is ignored even when it is a symlink. Original AR/AS/AT/AU/AV directories remain untouched.

Dependencies: C++17, Eigen3, TBB, nlohmann/json; Python numpy/scipy/matplotlib/PyYAML; original GeographicLib CartConvert for uncached reference conversion; authorized original CUDA/Deskew runtimes. Existing vendor UPSTREAM.json pins small_gicp fa0cfc983417c044b6036f71dfa448038bb9c820 MIT. No library clone/copy is required.

```
export IESKF_CONFIG=data/history/phase_v/config/frozen.json
export PYTHONPATH=src
export PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1
# Set SMALL_GICP_INCLUDE to the existing authorized upstream/include directory.
export AW_GICP_BINARY="$(realpath data/phase_aw_native_gicp_fusion/private/runtime/gicp_service)"
sh experiments/phase_aw_native_gicp_fusion/build_gicp.sh
python3 -m unittest discover -s tests
```

AW_GICP_BINARY must be exported to actually run the3 executable integration tests. Final result195 tests passed (183 frozen+12 new), no final skips/failures. Without that variable the binary test class explicitly skips; do not report that as the same qualification. Historical compile/zero-pair adapter failures remain in private logs/source snapshot.

Whole-lap actual same-Source/same-float-seed matcher experiment:

```
python3 experiments/phase_aw_native_gicp_fusion/match_ab.py \
 --baseline data/phase_ar_loop44_map0105/fullpose100_lap \
 --target data/phase_ar_loop44_map0105/map0105_target.bin \
 --gicp "$AW_GICP_BINARY" \
 --ndt data/phase_ar_loop44_map0105/runtime/match_fixed100 \
 --as-evaluation data/phase_as_accuracy_quality/single_lap/evaluation.npz \
 --output data/phase_aw_native_gicp_fusion/private/NEW_ab
```

The wrapper refuses an existing output directory.1468 pairs plus predefined0/1/3/5/8/9 midpoint probes/repeats are executed; no GNSS-selected frames or seed tuning. This diagnostic is not the actual feedback loop.

Actual own-state GICP single-lap loop:

```
python3 experiments/phase_aw_native_gicp_fusion/run_loop.py \
 --input data/phase_ao_ap_validation/loop44.json \
 --bindings data/phase_ar_loop44_map0105/bindingslap.json \
 --target data/phase_ar_loop44_map0105/map0105_target.bin \
 --deskew data/phase_ar_loop44_map0105/runtime/deskew_service \
 --deskew-service data/phase_ar_loop44_map0105/runtime/deskew_service \
 --matcher "$AW_GICP_BINARY" \
 --private data/phase_aw_native_gicp_fusion/private/NEW_lap \
 --output data/phase_aw_native_gicp_fusion/private/NEW_lap_summary.json \
 --backend gicp --window lap
```

For actual4134 continuous frames replace bindings with `data/phase_as_accuracy_quality/bindings_multilap.json`, window`multilap`, and new output names. Keep original500ms history; do not reset each lap, drop slow frames or invent completion timestamps. No frozen matching poses feed this loop. `--profile` obtains per-call timing; a complete timed1468 replay and200-frame preliminary check are retained. Use `/usr/bin/time -v` for jobCPU/RSS, not device memory. Profiling states/P/acceptance are compared against the uninstrumented own-state loop.

Original baseline reproduction: backend`ndt_legacy`, matcher`data/phase_ar_loop44_map0105/runtime/match_fixed100`, limit40, new outputs. SameR control: backend`ndt_fixed`, same actual NDT matcher, entire lap, no limit. Both fixed modes use the preregistered0.5m/1deg covariance; no raw GICP score orH inverse is used asR. The original`ndt` record key and reject string are schema compatibility names. Backend identity and metadata explicitly distinguish CPU small_gicp versus CUDA-NDT, both with CUDA Deskew.

NativeB reuses actual `data/phase_as_accuracy_quality/N_lap`; no Native solver redesign or unrecorded rerun. A is original `fullpose100_lap`;3lap A is `F_multilap`. These remain frozen.

Unified evaluation, including all0/75/150ms hypotheses and optional conditional visual box:

```
python3 experiments/phase_aw_native_gicp_fusion/evaluate.py \
 --matching-backend gicp \
 --run data/phase_aw_native_gicp_fusion/private/NEW_lap \
 --baseline data/phase_ar_loop44_map0105/fullpose100_lap \
 --motion data/phase_ao_ap_validation/loop44.json \
 --as-evaluation data/phase_as_accuracy_quality/single_lap/evaluation.npz \
 --at-bindings data/phase_at_gnss_footprint_corners/bindings.json \
 --reference-cache data/phase_aw_native_gicp_fusion/private/gicp_multilap/evaluation/evaluation.npz \
 --run-summary data/phase_aw_native_gicp_fusion/private/NEW_lap_summary.json \
 --private data/phase_aw_native_gicp_fusion/private/NEW_lap/evaluation \
 --output data/phase_aw_native_gicp_fusion/private/NEW_metrics.json \
 --label GICP --expected 1468 \
 --geometry data/phase_at_gnss_footprint_corners/visual_outline.json
```

Without reference-cache, original historical GNSS CSV must exist with AT's recorded hash. That file became unavailable during AW, so later evaluations use the already-computed AW4134 reference arrays/masks. They contain the exact same native Headers; the evaluator asserts equality before slicing. INS orientation is reconstructed from AT raw attitude/config, not from an estimator or a fitted yaw. Masks/converted reference SHA are sealed. This is not a newly calibrated truth. To reproduce from scratch without cached derivatives, recover the original qualified historicalCSV/config/raw reference assets; do not guess its eligibility mask. Public figures/reports remain reproducible from aggregate JSON without private geometry.

For3lap evaluation use actualGICPmultilap, baseline`data/phase_as_accuracy_quality/F_multilap`, AS`multilap/evaluation.npz`, expected4134; matching-backend`gicp`. Native or NDTsameR uses matching-backend`ndt`; declaration is checked against actual transaction metadata. Historical primary common mask1197 and raw first-lap1204 mask have distinct semantics and RMSE values.

```
python3 experiments/phase_aw_native_gicp_fusion/analyze_ab.py \
 --run data/phase_aw_native_gicp_fusion/private/NEW_ab \
 --as-evaluation data/phase_as_accuracy_quality/single_lap/evaluation.npz \
 --at-bindings data/phase_at_gnss_footprint_corners/bindings.json \
 --reference-cache data/phase_aw_native_gicp_fusion/private/gicp_multilap/evaluation/evaluation.npz \
 --output data/phase_aw_native_gicp_fusion/private/NEW_ab_metrics.json
python3 experiments/phase_aw_native_gicp_fusion/geometry_stability.py \
 --probes data/phase_aw_native_gicp_fusion/private/NEW_ab/probes.jsonl \
 --output data/phase_aw_native_gicp_fusion/private/NEW_stability.json
python3 experiments/phase_aw_native_gicp_fusion/native_diagnostics.py \
 --matches data/phase_aw_native_gicp_fusion/private/NEW_ab/matches.jsonl \
 --output data/phase_aw_native_gicp_fusion/private/NEW_native_diagnostics.json
python3 experiments/phase_aw_native_gicp_fusion/feedback_diagnostics.py \
 --run data/phase_aw_native_gicp_fusion/private/NEW_lap \
 --motion data/phase_ao_ap_validation/loop44.json \
 --as-evaluation data/phase_as_accuracy_quality/single_lap/evaluation.npz \
 --output data/phase_aw_native_gicp_fusion/private/NEW_feedback.json
python3 experiments/phase_aw_native_gicp_fusion/build_reports.py
```

The last command regenerates public aggregate reports/figures from committed metrics. Full private overlays, state/P, Source/Deskew history, failed candidates and logs remain persistent. Numerical variation from parallel reductions is measured (~1e−13 in replay), not misrepresented as bitwise portability. A different host/load may alter timing and future callback completion; never normalize those times to fabricate a realtime pass.

Rollback: original AR script or ndt_legacy with the original NDT binary; no production/default config or original estimator source was modified. AW3 adaptiveR remains deliberately absent, not a hidden runtime switch. Main is not promoted on failed accuracy acceptance.

Optional aggregate eigenstructure: `python3 experiments/phase_aw_native_gicp_fusion/eigenstructure.py --geometry data/phase_aw_native_gicp_fusion/private/NEW_ab/geometry_analysis.npz --output data/phase_aw_native_gicp_fusion/private/NEW_spectra.json`. This consumes retained full matrices only, with no new matching or covariance tuning.

Final sealing with separately authorized paths: `python3 experiments/phase_aw_native_gicp_fusion/seal.py --private-root data/phase_aw_native_gicp_fusion/private --production-root "$AW_PRODUCTION_ROOT" --output data/phase_aw_native_gicp_fusion/private/NEW_manifest.json`. It reads source/stash identity only, does not apply changes, and verifies fixedR in actual accepted packets.
