# Reproduction and data boundary

Public Python requirements are in the repository pyproject.toml. Nine self-contained synthetic tests need NO project data/source:

```bash
export PYTHONPATH="$PWD/src:$PWD/experiments/phase_ac_geometry_capture"
OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p test_association_objective.py -v
```

Exact real-input reproduction requires separately authorized local payloads: the existing AA complete replay, PhaseV frame manifest/frozen config/source arrays, map binary, contract/extrinsic, AC saved cycle/13-start histories and deterministic seed file. See nonreconstructive input identities in manifest.json. These are not downloaded or distributed by this repository. Public logs, aggregates and explanations are self-contained, but cannot regenerate the real map experiment without those inputs.

Use a fresh local ignored output directory to avoid overwriting preserved evidence; run_candidate rejects an existing completed results file. IESKF_DATA_ROOT points to the root containing the already-preserved data hierarchy, NOT a new fabricated event stream.

```bash
export IESKF_DATA_ROOT=/authorized/local/data-root
export AD_RESULTS_ROOT="$IESKF_DATA_ROOT/new_phase_ad_results"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
mkdir -p "$AD_RESULTS_ROOT"
python3 experiments/phase_ad_association_objective/audit_paths.py --data-root "$IESKF_DATA_ROOT" --output "$AD_RESULTS_ROOT"
python3 experiments/phase_ad_association_objective/detail_switches.py --data-root "$IESKF_DATA_ROOT" --output "$AD_RESULTS_ROOT"
python3 experiments/phase_ad_association_objective/run_candidate.py --data-root "$IESKF_DATA_ROOT" --output "$AD_RESULTS_ROOT/candidate"
python3 experiments/phase_ad_association_objective/audit_precision.py --data-root "$IESKF_DATA_ROOT" --output "$AD_RESULTS_ROOT"
python3 experiments/phase_ad_association_objective/audit_smooth_paths.py --data-root "$IESKF_DATA_ROOT" --output "$AD_RESULTS_ROOT"
python3 experiments/phase_ad_association_objective/audit_endpoints.py --data-root "$IESKF_DATA_ROOT" --output "$AD_RESULTS_ROOT"
python3 experiments/phase_ad_association_objective/audit_force_balance.py --data-root "$IESKF_DATA_ROOT" --output "$AD_RESULTS_ROOT"
```

Capture stdout of path/candidate/precision commands into path_audit.log/candidate.log/precision.log under AD_RESULTS_ROOT for the summary tool. Create the chosen public output directory. Then summarize.py --private-root "$AD_RESULTS_ROOT" --public-root /chosen/deidentified/output produces only its explicit whitelist. A/B histories are read from the sibling phase_ac_geometry_capture directory by summarize.py; preserve that original relative hierarchy or adjust the summary path only, not scientific inputs.

No A2RL production module is copied/imported or required for the synthetic objective tests. Frozen real-input readers reuse independent AC research adapters and private binary/NumPy payloads, not live ROS. No ROS topic or TF is published. Exact CPU timings depend on host/cache and are not hard real-time guarantees.


Evidence boundary: executed frozen-input results are stated above; AC results are explicitly historical reuse. Physical pose correctness, calibrated uncertainty, and continuous Z/vz improvement are not inferred. Raw points, full states/P, map, timestamps, identities and sensitive configuration stay in ignored local data. Speculative next steps are not executed results.
