# Reproduction and publication boundary

Public requirements are numpy/scipy/matplotlib from pyproject.toml. Standalone NEW tests do not require A2RL source/data and do not rerun AD's tests:

```bash
export PYTHONPATH="$PWD/src:$PWD/experiments/phase_ac_geometry_capture"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 -m unittest discover -s tests -p test_measurement_information.py -v
python3 experiments/phase_ae_measurement_information/run_synthetic.py --output /chosen/fresh/local/synthetic-output
```

The synthetic generator specifies all points/prior/truth; no private input. Original AD association_objective.py remains byte-identical. density_audit.py changes ONLY input-record views/normal cache for diagnostics; information_audit.py differentiates the SAME energy and provides signed terms. No production source import or ROS publishing.

Exact real-input reproduction needs separately authorized local AA/V/AC/AD payloads described by manifest hashes: first frozen Source/map, full prediction/P/native time contract, AD precision endpoints and original13 seeds. These are not public. Reports/logs/aggregates stand alone for conclusions but cannot recreate the actual map experiment without authorized payloads.

Use FRESH output to avoid overwriting evidence; density preparation and completed density runner reject accidental repeats. Preserve the existing data hierarchy including history/phase_v, inputs, phase_aa_z_validation, phase_ac_geometry_capture and phase_ad_association_objective.

```bash
export IESKF_DATA_ROOT=/authorized/local/data-root
export AE_RESULTS_ROOT="$IESKF_DATA_ROOT/new_phase_ae_results"
mkdir -p "$AE_RESULTS_ROOT"
python3 experiments/phase_ae_measurement_information/prepare_density.py --data-root "$IESKF_DATA_ROOT" --output "$AE_RESULTS_ROOT"
python3 experiments/phase_ae_measurement_information/run_density.py --data-root "$IESKF_DATA_ROOT" --output "$AE_RESULTS_ROOT"
python3 experiments/phase_ae_measurement_information/run_validation.py --data-root "$IESKF_DATA_ROOT" --output "$AE_RESULTS_ROOT"
python3 experiments/phase_ae_measurement_information/complete_validation.py --data-root "$IESKF_DATA_ROOT" --output "$AE_RESULTS_ROOT"
python3 experiments/phase_ae_measurement_information/audit_directions.py --data-root "$IESKF_DATA_ROOT" --output "$AE_RESULTS_ROOT"
python3 experiments/phase_ae_measurement_information/bind_density_factors.py --data-root "$IESKF_DATA_ROOT" --output "$AE_RESULTS_ROOT"
python3 experiments/phase_ae_measurement_information/run_synthetic.py --output "$AE_RESULTS_ROOT/synthetic"
```

Capture stdout to prepare.log/density.log/validation.log/multiplicity.log/synthetic.log in that root for summarize.py. After creating a public output directory, summarize.py --private-root "$AE_RESULTS_ROOT" --public-root /chosen/deidentified/output exports ONLY its explicit whitelist. In the actual execution run_validation overlapped density optimization, so complete_validation filled missing posthoc scores later without rerunning fits. Sequential reproduction above produces the same scientific comparisons; CPU times/cache are not reproduced exactly. Training density choices use training only; G1/G2 and full-data fits are NEVER labeled independent validation.

Private retained evidence includes complete signed corrections,15D states/P, full physical/white Hessian matrices/eigenvectors, fixed-factor IDs and weights, density alias/mask identity, neighbor-disjoint fold IDs, untruncated per-point validation arrays, all failures and construction logs. Public version includes independent code, synthetic controls, deidentified relative aggregates/spectra/axis loadings, logs and charts only. No raw map/point/trajectory/device/extrinsic data or restricted project source.


Evidence boundary: real-input values above are deidentified diagnostics, synthetic controls are explicitly separate, and historical AD results are reused rather than repeated. No absolute accuracy, statistical calibration or production qualification is inferred. Full Source/map/normal/state/P/trajectory records, factor identities and matrices remain in ignored local data. Proposed future model work is not an executed fix.
