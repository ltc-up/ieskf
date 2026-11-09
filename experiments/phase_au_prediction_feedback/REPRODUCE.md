# AU reproduction

Use a new durable ignored private directory and separately authorized AR/AS/AT inputs; never overwrite the frozen runs. NumPy/SciPy and this repository's src are required. Set OPENBLAS_NUM_THREADS=1, PYTHONDONTWRITEBYTECODE=1, PYTHONPATH=src and IESKF_CONFIG to the **original frozen configuration**, whose hash is recorded in manifest. No GPU/ROS run is claimed by the history recovery.

```
python3 experiments/phase_au_prediction_feedback/recover_history.py \
 --motion "$MOTION_EXPORT" --bindings "$AR_LAP_BINDINGS" --run "$AR_LAP_RUN" \
 --private "$AU_PRIVATE" --output experiments/phase_au_prediction_feedback/replay_equivalence.json
PYTHONPATH=src python3 -m unittest discover -s tests
```

The original motion export, raw-cloud bindings and frozen NDT transaction/state files recover exactly AR initialization and sensor order. Every matrix, state, raw measurement and full intermediate checkpoint remains private. The public replay_equivalence.json contains only maximum discrepancies and counts. Equivalence must pass at1e−10 absolute for numerical arrays and exactly for acceptance/time before using the traces. This is a frozen-observation reconstruction, not a new causal CUDA rematch. It is suitable for local equation/feedback analysis only after the gate passes.

Private outputs: envelope_trace.npz (all revisions), canonical_trace.npz (one final application per event), canonical_checkpoints.npz, canonical_events.json, initial.npz, native_full_priors.json, ingests.json, windows.json and recovery/test logs. Header, sensor recorder, saved cloud availability and saved completion envelope are distinguished. AS fixed sections1/3/9 and ±1s windows are in window_plan.json; timestamps needed for private replay are in private windows.json. No GNSS enters the reconstruction.

After the equivalence gate, execute with the same frozen configuration:
```
python3 experiments/phase_au_prediction_feedback/analyze_feedback.py \
 --private "$AU_PRIVATE" --at-causal "$AT_CAUSAL_NPZ" --as-npz "$AS_EVALUATION_NPZ" \
 --windows "$AU_PRIVATE/windows.json" \
 --equivalence experiments/phase_au_prediction_feedback/replay_equivalence.json \
 --output experiments/phase_au_prediction_feedback/feedback_summary.json
python3 experiments/phase_au_prediction_feedback/motion_candidates.py \
 --private "$AU_PRIVATE" --windows "$AU_PRIVATE/windows.json" \
 --output experiments/phase_au_prediction_feedback/motion_evidence.json
```
The private windows.json is frozen from AU1 window_plan and corresponding original native headers (±1s). New private feedback_series/yaw_velocity_evidence NPZ files and section1/3/9 prediction_feedback/motion_candidates figures contain full diagnostics. Compressed history arrays are loaded once, not repeatedly decompressed in the inner loop. The per-event cumulative balance includes current-axis transport and does not equate vehicle displacement with error.
