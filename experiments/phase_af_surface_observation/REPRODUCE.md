# Reproduce AF

Public synthetic reproduction needs only this repository, Python3, NumPy and SciPy. No A2RL source/map/input is needed:

```bash
PYTHONPATH=src OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p test_surface_observation.py -v
PYTHONPATH=src OPENBLAS_NUM_THREADS=1 python3 experiments/phase_af_surface_observation/run_synthetic.py --output /tmp/af_synthetic_results.json
```

23 tests, synthetic rank/copy/shared-error checks and numerical posterior reproduce independently. To recreate the public synthetic/statistical plots, use plot_results.py with the published aggregate file. Plots do not require raw point clouds:

```bash
python3 experiments/phase_af_surface_observation/plot_results.py --results experiments/phase_af_surface_observation/key_results.json --output /tmp/af_surface_results.png
```

Real frozen analysis requires separately authorized local data matching manifest digests: saved native Source, map, AA first prediction/P, AD saved candidate coordinates and original lever/time contract. These payloads and original production sources are NOT distributed. Merely cloning GitHub cannot reproduce the real-data numbers. Set IESKF_DATA_ROOT to that local ignored input directory; do not hardcode a machine path:

```bash
PYTHONPATH=src OPENBLAS_NUM_THREADS=1 python3 experiments/phase_af_surface_observation/run_synthetic.py --output "$IESKF_DATA_ROOT/phase_af_surface_observation/synthetic_results.json"
PYTHONPATH=src OPENBLAS_NUM_THREADS=1 python3 experiments/phase_af_surface_observation/run_frozen.py --data-root "$IESKF_DATA_ROOT" --output "$IESKF_DATA_ROOT/phase_af_surface_observation"
PYTHONPATH=src OPENBLAS_NUM_THREADS=1 python3 experiments/phase_af_surface_observation/analyze_saved.py --output "$IESKF_DATA_ROOT/phase_af_surface_observation"
```

The output directory must already exist. Existing research compare_states loader validates Source/map hashes; native time equals prediction state time. Public defaults equal all frozen numeric configuration entries; private topic labels are excluded. Full plane frames, associations, r/H/J/P/state and covariance proposals remain local. No ROS publishers, TF owner, production build, live replay or continuous recursion is invoked.

Extraction scales5/2.5/10m and synthetic error laws are predefined in TEST_PLAN. Only5m is the baseline. Extraction uses exact Target dedup, TLS and unchanged inherited thresholds; unresolved components are excluded. All frozen outcomes including rejection are retained. Time results are CPU wall observations, not a production performance test. See manifest for code/input hashes and versions; public code origin is independently authored research, not copied production code.
