# Geometry-IESKF research

Independent offline research in inertial error-state estimation, wheel constraints, map point-plane factors and iterative full-state updates. This is a research prototype with known convergence and uncertainty limitations.

## Public boundary

Only independently written algorithms, synthetic smoke checks, portable tools and redacted research summaries are distributed. Production project code, sensor data, maps, trajectories, equipment details, extrinsics and real-data results are excluded. Historical originals are retained locally without rewriting. No third-party source is vendored and no production installation is replaced.

## Run the portable smoke checks

```bash
python3 -m pip install -e .
PYTHONPATH=src python3 -m unittest discover -s tests -v
```

Python, NumPy, SciPy and matplotlib are required. The public default configuration is an uncalibrated research example; it contains no device extrinsics. For authorized private replay use `IESKF_CONFIG`, `IESKF_DATA_ROOT` and optionally `A2RL_ROOT`; see docs/A2RL_INTEGRATION.md. Real-input reproduction requires separately authorized local inputs.

Start with docs/EXPERIMENT_INDEX.md and docs/RESEARCH_STATUS.md. The legacy iterative update intentionally retains partial iteration-limit acceptance to reproduce historical behavior. Do not use it as a production filter or a calibrated posterior.
