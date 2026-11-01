# Reproduce Phase AB

## Self-contained public checks

```bash
python3 -m pip install -e .
export PYTHONPATH="$PWD/src"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
python3 -m unittest discover -s tests -p test_qualification.py -v
```

These eight tests target only the new qualification/transaction interface. Do not rerun U/V/X/Y campaigns to claim new phase results.

## Real-input prerequisites and boundaries

Authorized local inputs are required: saved AA corrected replay and traces, its immutable Source/event/initialization/15x15 covariance contract, frozen configuration, static Target, and saved V IMU/wheel-only control. Non-reconstructive identity hashes and the exact research-code commit are in manifest.json. GitHub does not distribute private payloads, raw events, full trajectories or production sources; therefore exact real-data replication is conditional on separately obtaining those authorized inputs. No original production source is imported by the independent mathematical package.

```bash
export IESKF_DATA_ROOT=/path/to/authorized/local/data
export IESKF_CONFIG="$IESKF_DATA_ROOT/history/phase_v/config/frozen.json"
export RUN_ROOT="$IESKF_DATA_ROOT/reproduced_phase_ab"
mkdir -p "$RUN_ROOT"
python3 experiments/phase_ab_geometry_qualification/diagnose.py --data-root "$IESKF_DATA_ROOT" --output "$RUN_ROOT" > "$RUN_ROOT/diagnose.log" 2>&1
python3 experiments/phase_ab_geometry_qualification/check_candidate.py --data-root "$IESKF_DATA_ROOT" --analysis-file "$RUN_ROOT/frame_diagnostics.json" --output "$RUN_ROOT/candidate_frozen_checks.json"
```

The diagnostic re-queries factors at recorded poses and checks original Target IDs, objectives and P; it does not run baseline predictions again. It executes only four selected independent fixed-factor refinements, preserving the full unmodified prior. Results include raw derivatives, information and weights privately.

Before any reproduced continuous run, verify reconstructed cost error <1e-7, P error <1e-9, all five frozen checks and all eight tests. Record `continuous_admission.json` in RUN_ROOT with `eligible_diagnostic_only: true`, the check results, exact parameter freeze, one-run cap and anticipated coverage-loss warning. This phase's original admission was made before the candidate run. Admission is a controlled diagnostic, not deployment approval.

```bash
python3 experiments/phase_ab_geometry_qualification/run_candidate.py --data-root "$IESKF_DATA_ROOT" --output "$RUN_ROOT" > "$RUN_ROOT/continuous.log" 2>&1
python3 experiments/phase_ab_geometry_qualification/summarize.py --data-root "$IESKF_DATA_ROOT" --output "$RUN_ROOT" --public-output "$RUN_ROOT/public_aggregates"
```

The runner refuses to overwrite a prior candidate replay, preventing accidental repetition. Preserve the original phase evidence and use a fresh RUN_ROOT. Full continuous plots are emitted ONLY to the private output directory; public output contains approved de-identified distributions, aggregate comparison charts and key_results.json. Published report tables are derived from those aggregates. Diagnostic CPU timings include identity reconstruction/extra queries and are not a production benchmark.

No deskew, new matching backend, constrained MAP/KKT experiment, sensor/noise reconfiguration or additional candidate window is needed. Given this phase's coverage failure, do not extend replay without a new concrete mechanism and its own qualification.
