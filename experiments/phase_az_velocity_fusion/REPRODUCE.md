# Authorized reproduction

Checkout independent AZ branch; preserve original/AW/AX/AY/stashes. Requires existing authorized `data` binding, loop44 scalar motion/static TF, map0105 CUDA Target, original CUDA Deskew/match_fixed100 binaries and original AS/AT engineering-reference cache. Public Git excludes restricted raw points/maps/TF/full trajectories. Actual used hashes and enclosing research commit are in manifest.json.

Set `AZ_PRIVATE` to a **new persistent directory**, bind its `optical.json` to the frozen AY raw extraction (SHA in manifest). Set PYTHONPATH=src, OPENBLAS_NUM_THREADS=1, PYTHONDONTWRITEBYTECODE=1 and IESKF_CONFIG=data/history/phase_v/config/frozen.json. Reuse AY `extract_optical.py` if re-extraction is needed; source installed ROS and keep its PYTHONPATH when extracting. No GNSS is needed by extraction/runner.

```
python3 experiments/phase_az_velocity_fusion/launch.py --root "$AZ_PRIVATE" --name X40 --limit 40
python3 experiments/phase_az_velocity_fusion/launch.py --root "$AZ_PRIVATE" --name X_lap
python3 experiments/phase_az_velocity_fusion/launch.py --root "$AZ_PRIVATE" --name M_lap --managed
python3 experiments/phase_az_velocity_fusion/launch.py --root "$AZ_PRIVATE" --name M_multilap --managed --window multilap
python3 experiments/phase_az_velocity_fusion/launch.py --root "$AZ_PRIVATE" --name S_lap --managed --speed-norm
python3 experiments/phase_az_velocity_fusion/launch.py --root "$AZ_PRIVATE" --name S_multilap --managed --speed-norm --window multilap
```

For each full run call `evaluate_run.py --root "$AZ_PRIVATE" --name NAME` (add `--multilap` for continuous three-lap jobs). Evaluation is a separate process using original fixed native Header/masks/0/75/150ms, never an input to the estimator. AS fixed segments are unchanged. A and AY-B metrics are reused immutable actual results; no fixed historical final pose is fed to a candidate.

Generate exact predefined faults with `inject.py --input "$AZ_PRIVATE/optical.json" --output "$AZ_PRIVATE/optical_fault.json" --record "$AZ_PRIVATE/injection.json"`. Run S_fault_lap with `--managed --speed-norm --optical-file "$AZ_PRIVATE/optical_fault.json"`; evaluate separately. Add `--profile` for fresh S_profile_lap without faults. Compare every native seed/P/NDT/K/delta with S_lap; no time substitution/cache expansion/frame dropping.

Sensor study: `sensor_study.py --optical "$AZ_PRIVATE/optical.json" --motion data/phase_ao_ap_validation/loop44.json --states data/phase_as_accuracy_quality/F_multilap/states.json --output "$AZ_PRIVATE/sensor_study.json"`. Raw-wheel extraction: after sourcing ROS/messages, `extract_wheels.py --bag "$AZ_BAG" --optical "$AZ_PRIVATE/optical.json" --output "$AZ_PRIVATE/wheel_audit.json"`. The4wheel values are audit only and are not fused.

`PYTHONPATH=src OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests` gives206 tests. `PYTHONPATH=src python3 experiments/phase_ay_localization_solution/verify_frozen.py` checks original artifacts and marks missing historical GNSS CSV explicitly. `aggregate.py --root "$AZ_PRIVATE" --ay-root "$AY_PRIVATE" --output experiments/phase_az_velocity_fusion/results` produces non-reconstructive statistics/plots after all full/fault/profile evaluations exist.

Actual private retention: X40/X_lap, failed M40 startup, M40_fixed, M_lap/M_multilap, S40/S40_final, S_lap/S_multilap, S_fault_lap,S_profile_lap; all states/P, Sources/NDT/manager revisions/stdout/stderr, evaluation plots, sensor-only rows, raw wheel rows, initial failed tests, final regression/hash/parity records. Launch refuses existing run/log paths. No experiment directory is cleaned.

Runtime adapters use explicitly asserted frozen source identities in manifest; no production write or default change. Final added stale-wheel guard does not activate in the recorded nonzero optical driving window; final40 confirms key outputs bit-exact. New synthetic test verifies that an old wheel sample cannot falsely call a genuine optical zero a fault. Candidate remains FAILED overall; standalone branch only.
