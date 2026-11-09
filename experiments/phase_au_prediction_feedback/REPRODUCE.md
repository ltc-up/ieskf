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
