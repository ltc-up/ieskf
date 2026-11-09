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

Complete the frozen-window setup before analysis (refuses a different existing window definition):
```
python3 experiments/phase_au_prediction_feedback/prepare_windows.py \
 --plan experiments/phase_au_prediction_feedback/window_plan.json \
 --transactions "$AR_LAP_RUN/transactions.json" --output "$AU_PRIVATE/windows.json"
```
Then run the AU2 commands above and:
```
python3 experiments/phase_au_prediction_feedback/covariance_partition.py \
 --private "$AU_PRIVATE" --output experiments/phase_au_prediction_feedback/covariance_partition.json
python3 experiments/phase_au_prediction_feedback/counterfactual.py \
 --private "$AU_PRIVATE" --windows "$AU_PRIVATE/windows.json" \
 --equivalence experiments/phase_au_prediction_feedback/replay_equivalence.json \
 --output experiments/phase_au_prediction_feedback/counterfactual_results.json
python3 experiments/phase_au_prediction_feedback/onset_diagnostic.py \
 --series "$AU_PRIVATE/feedback_series.npz" \
 --plan experiments/phase_au_prediction_feedback/window_plan.json \
 --output experiments/phase_au_prediction_feedback/onset_diagnostic.json
python3 experiments/phase_au_prediction_feedback/summary_figures.py
python3 experiments/phase_au_prediction_feedback/verify_frozen.py \
 --at-bindings "$AT_PRIVATE_BINDINGS" --ar-run "$AR_LAP_RUN" --ar-bindings "$AR_LAP_BINDINGS" \
 --as-f "$AS_F_TRANSACTIONS" --as-n "$AS_N_TRANSACTIONS" --config "$IESKF_CONFIG" \
 --output experiments/phase_au_prediction_feedback/frozen_identity_check.json
PYTHONPATH=src python3 -m unittest discover -s tests
```

The private AT bindings are those used in AT reproduction and locate original authorized inputs. verify_frozen hashes actual AU input files and original AR/AS transaction ledgers against AT's published identities, verifies AT causal NPZ, and asserts no tracked AR/AS/AT/model source modifications against the frozen baseline. Original bag/map identities are inherited; they are not read or rehashed because AU does not rematch.

Final private outputs also include section1/3/9 uncertainty plots (correct postreset pose Jacobian), yaw/velocity/Bias plots, full counter_section*_*.npz pre/post means/P and per-branch NDT JSON with seeds, frozen finals and acceptance. Branch baselines are compared with saved canonical checkpoints at every event. Single skips use a clock advance to preserve quadrature and held-input age. The full true CUDA baseline remains the original AR/AS dataset, unchanged. Newly executed branches are fixed-observation conditional replays only. Original and corrected diagnostic logs are retained; no old experiment is overwritten.
