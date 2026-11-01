# Reproduce

Use Python3 with NumPy/SciPy/matplotlib, from this independent repository. No production imports are needed for the public synthetic checks:

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=src python3 -m unittest discover -s tests -p test_ndt_pose_observation.py -v
```

Real-data reproduction requires separately authorized inputs under IESKF_DATA_ROOT: the original V event/Source/contract files, A control, AA B replay, AH five frozen records/NDT outputs and complete original Target. manifest.json hashes bind them. A2RL_ROOT can identify authorized original source; it is never written or imported by the public observation module. Restricted sources/payloads and complete original trajectories are not downloadable from this repo.

An authorized private external NDT service is also required. It reads one JSON job per stdin line and returns one JSON result per stdout line, caches the immutable map, takes the caller's source/seed/active region, and exposes final pose, fitness, convergence, iteration and timing fields. The local-only adapter uses unchanged SDK library/parameters and remains ignored; its source/binary/library hashes are recorded. It passed a same-input/seed exact AH parity check. Public research code does not implement or redistribute NDT.

Commands, in order (each continuous command executes a new experiment; do not run it just to regenerate figures):

```bash
AJ=experiments/phase_aj_ndt_pose_update
OPENBLAS_NUM_THREADS=1 PYTHONPATH=src python3 "$AJ/validate_frozen.py" --data-root "$IESKF_DATA_ROOT"
# After synthetic and frozen checks pass; run ONCE:
OPENBLAS_NUM_THREADS=1 PYTHONPATH=src python3 "$AJ/run_causal.py" --data-root "$IESKF_DATA_ROOT" --ndt-service "$IESKF_NDT_SERVICE"
# Saved-result analysis only, no new matching or recursion:
OPENBLAS_NUM_THREADS=1 PYTHONPATH=src python3 "$AJ/analyze_saved.py" --data-root "$IESKF_DATA_ROOT" --public-output "$AJ"
OPENBLAS_NUM_THREADS=1 PYTHONPATH=src python3 "$AJ/audit_causality_and_plot.py" --data-root "$IESKF_DATA_ROOT" --public-output "$AJ"
OPENBLAS_NUM_THREADS=1 PYTHONPATH=src python3 "$AJ/horizontal_summary.py" --data-root "$IESKF_DATA_ROOT" --public-output "$AJ"
OPENBLAS_NUM_THREADS=1 PYTHONPATH=src python3 "$AJ/wheel_height_summary.py" --data-root "$IESKF_DATA_ROOT" --public-output "$AJ"
```

One run was executed, plus one service parity alignment; A/B reused. No retries or tuning were performed after seeing C failures. Posthoc timing can differ between machines and audit passes; original timing evidence remains in the private run and public report. The public figure contains coarse scalar bins only, not an original trajectory. Full closed-loop reproduction is impossible without the authorized data and existing backend; the observation math/tests and all reported aggregate conclusions are self-contained.

The code commit is the enclosing Phase AJ Git commit, obtained with `git log -1 -- experiments/phase_aj_ndt_pose_update`. Its parent is the explicit AI baseline. No self-referential commit SHA is inserted into the manifest.
