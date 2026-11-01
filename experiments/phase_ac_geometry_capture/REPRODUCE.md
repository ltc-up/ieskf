# Reproduce AC

## Public tests

```bash
python3 -m pip install -e .
export PYTHONPATH="$PWD/src"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
python3 -m unittest discover -s tests -p test_capture.py -v
python3 -m unittest discover -s tests -p test_valid_correspondence.py -v
```

These are 14 new solver/query-interface checks; AB algebra/posterior proof suites are not rerun.

## Authorized real-input requirements

Requires the immutable AA corrected replay/traces, frozen Source/event/initialization/full15 P contract, static Target and configuration, plus AB's saved first fixed-final-factor refinement. Input and code hashes are in manifest.json. No original production source is imported by the independent package. Private device/extrinsic/config values and payloads must be supplied separately; GitHub alone cannot recreate the recorded sensor experiment.

```bash
export IESKF_DATA_ROOT=/path/to/authorized/local/data
export IESKF_CONFIG="$IESKF_DATA_ROOT/history/phase_v/config/frozen.json"
export RUN_ROOT="$IESKF_DATA_ROOT/reproduced_phase_ac"
mkdir -p "$RUN_ROOT" "$RUN_ROOT/nearest_valid"
python3 experiments/phase_ac_geometry_capture/compare_states.py --data-root "$IESKF_DATA_ROOT" --output "$RUN_ROOT"
python3 experiments/phase_ac_geometry_capture/run_capture.py --data-root "$IESKF_DATA_ROOT" --output "$RUN_ROOT" > "$RUN_ROOT/capture.log" 2>&1
python3 experiments/phase_ac_geometry_capture/audit_cycle.py --data-root "$IESKF_DATA_ROOT" --output "$RUN_ROOT"
python3 experiments/phase_ac_geometry_capture/audit_support_boundary.py --data-root "$IESKF_DATA_ROOT" --output "$RUN_ROOT"
python3 experiments/phase_ac_geometry_capture/run_valid_candidate.py --data-root "$IESKF_DATA_ROOT" --output "$RUN_ROOT/nearest_valid" > "$RUN_ROOT/nearest_valid/capture.log" 2>&1
python3 experiments/phase_ac_geometry_capture/isolate_selection_order.py --data-root "$IESKF_DATA_ROOT" --output "$RUN_ROOT"
python3 experiments/phase_ac_geometry_capture/analyze_endpoints.py --data-root "$IESKF_DATA_ROOT" --output "$RUN_ROOT"
python3 experiments/phase_ac_geometry_capture/audit_candidate_edge.py --data-root "$IESKF_DATA_ROOT" --output "$RUN_ROOT"
python3 experiments/phase_ac_geometry_capture/summarize.py --input "$RUN_ROOT" --output "$RUN_ROOT/public_aggregates"
```

Use a NEW RUN_ROOT, never overwrite original evidence. Run scripts preserve per-outer histories and refuse an already completed seed_results.json. Both variants use exactly the same deterministic starts and full prior. Cache warming affects timing, not admissibility definitions; timings are frozen offline measurements only.

The additional conditional covariance calculation is a discarded diagnostic transaction, not a replay. No continuous runner is provided/admitted for AC because nominal capture failed. Public output contains only de-identified aggregates and objective-versus-iteration charts, not vehicle trajectories. The self-contained reports draw their numeric tables from key_results.json; direct field checks can assess every claim without local data.
