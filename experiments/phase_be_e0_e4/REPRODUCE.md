# Reproduction

Use a new output directory on the verified mounted volume 新加卷. Check `lsblk -f`, `findmnt`, `df -h`, Git HEAD/status/branches/stash and dependency hashes first. Never fall back to home or /tmp. The public code does not redistribute restricted inputs or legacy service binaries.

Prerequisites: authorized motion/transforms, raw three-LiDAR bindings, unchanged map and initialization; existing CUDA deskew/NDT services and CPU small_gicp service; Python NumPy/SciPy/pytest; CUDA-capable hardware. The exact implementation source for every reused service is not established by a historical snapshot; binary identity is therefore essential. The private `paths.json` maps `data`, `optical`, `ndt`, `gicp`, `deskew`, `target`, `motion`. The data tree follows the archived AR/AS/AO binding schemas. External binaries must match the private identity ledger; the archived GICP source snapshot has a known dangling-expression issue and is not asserted to reproduce the reused binary. Rebuilding from that snapshot is not a valid exact reproduction.

Set TMPDIR, MPLCONFIGDIR, XDG_CACHE_HOME to directories under the new volume, `PYTHONDONTWRITEBYTECODE=1`, `OPENBLAS_NUM_THREADS=1`, `PYTHONPATH=src`. Run serial configurations to avoid cross-experiment contention. Run `python3 -m pytest tests -q -p no:cacheprovider --basetemp=$TMPDIR/tests`.

Use `python3 experiments/phase_be_e0_e4/run_loop.py --paths PATHS --out NEW_OUTPUT --window lap --archive-source-history`. Repeat with `--window multilap` without any lap reset. Controls: full-source original15D uses `--max-iterations 100 --legacy-lifecycle`; the matched candidate control uses `--holdout` and default35 iterations. These are different controls; do not attribute their difference solely to a new observation algorithm.

Candidate switches, always relative to matched control:

- Directional: `--holdout --directional --project-directional`.
- Optical XY: `--holdout --directional --optical xy` (existing AY policy; not an extra tire model).
- Dual Shadow: `--holdout --directional --candidates`.
- GMF: `--holdout --directional --candidates --gmf`.
- Robust lifecycle: GMF plus `--reject-start 300 --reject-end 330 --retract-every 200`.

For frozen pairs, run `frozen.py --paths PATHS --requests REQUESTS --out NEW_OUTPUT`. The JSON requests must preserve source/target/seed/header and native service options. Use `analyze_frozen.py` for the complete single-lap list; do not feed a single unrelated historical frame to the lap reference evaluator.

Offline-only evaluation: `evaluate.py --root PRIVATE_ROOT --run RUN`. Authorized reference caches `reference_lap.npz` / `reference_multilap.npz` require the historical full-attitude antenna lever conversion and provide header/R/G_0.0/G_0.075/G_0.15 and validity masks. The fixed-segment archive supplies tangent/corner. Neither cache is read by runtime/filter/GMF code. Use `lidar_information.py --run RUN --bindings BINDINGS --target TARGET --output NEW_JSON` for sampled sensor-lineage analysis.

Each run saves config, source hashes, per-frame full state/P, matcher outputs, actual CUDA native cloud/history, timing, lifecycle ledger and summary. Failure directories and logs are retained. Cold startup, pause/interruption, insufficient delayed history, IO handshake failures and source-version changes must remain visible. Aggregates select only declared complete runs and identify retries; they do not overwrite failures. Private paths and detailed SHA ledger are documented separately under the actual storage root.

Final dependency audit found one inherited Deskew library resolving from an old home-based installation. It was read-only during the recorded runs, not a working/output directory. Its exact SHA is now snapshotted under the private new-volume runtime/deps directory; ldd with that library path resolves no home dependency. Source runtime/replay_environment.sh before reproduction, or set LD_LIBRARY_PATH to that directory explicitly. Dependency fingerprinting was post-run; it is not presented as a pre-run complete dependency audit. Original library and checkout remain untouched.
