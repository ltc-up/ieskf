# Reproduce AG

Public numerical checks need Python3, NumPy and SciPy, with no original A2RL source or real data:

```bash
PYTHONPATH=src OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p test_surface_capture.py -v
```

Five new interface checks: full15 frozen-R objective finite differences; large single-step proposal with legal multistep path; legal boundary optimum; noise matrix truly frozen; exhausted budget not relabeled. AF foundation tests are deliberately not rerun.

Real evidence needs separately authorized local inputs matching manifest: the existing AF data folder and original native Source/Target, original prior/P, extrinsic/time contract, saved map groups and local acquisition/reference metadata. Set IESKF_DATA_ROOT and AG_COLLECTION_ROOT to local paths; create the output directory first. Never commit those payloads.

```bash
mkdir -p "$IESKF_DATA_ROOT/phase_ag_surface_evidence_capture"
PYTHONPATH=src OPENBLAS_NUM_THREADS=1 python3 experiments/phase_ag_surface_evidence_capture/audit_inputs.py --collection-root "$AG_COLLECTION_ROOT" --data-root "$IESKF_DATA_ROOT" --output "$IESKF_DATA_ROOT/phase_ag_surface_evidence_capture"
PYTHONPATH=src OPENBLAS_NUM_THREADS=1 python3 experiments/phase_ag_surface_evidence_capture/run_surface_capture.py --data-root "$IESKF_DATA_ROOT" --output "$IESKF_DATA_ROOT/phase_ag_surface_evidence_capture"
PYTHONPATH=src OPENBLAS_NUM_THREADS=1 python3 experiments/phase_ag_surface_evidence_capture/probe_saved.py --data-root "$IESKF_DATA_ROOT" --collection-root "$AG_COLLECTION_ROOT" --output "$IESKF_DATA_ROOT/phase_ag_surface_evidence_capture"
```

The collection layout is encoded by the read-only inventory interface; hashes and complete local provenance identify the inspected files. GitHub alone can reproduce synthetic numerics, not real capture/calibration without restricted inputs. All map paths, absolute timestamps, raw headers/coordinates/full states/P/planes are ignored local outputs. No production integration/build/ROS publication is needed. The original prior is held fixed across all steps; per-association R is frozen at original prediction. Budgets/scales/thresholds and the supplementary microprobe disclosure are in TEST_PLAN.md; no best seed or hyperparameter sweep. Figure reproduction uses plot_results.py on the published aggregate JSON only.

```bash
python3 experiments/phase_ag_surface_evidence_capture/plot_results.py --results experiments/phase_ag_surface_evidence_capture/key_results.json --output /tmp/ag_capture_evidence.png
```
