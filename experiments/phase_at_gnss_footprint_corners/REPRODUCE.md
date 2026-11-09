# Reproduce AT

Requires separately authorized original loop44, historical AR1468 transactions, AS single_lap/evaluation.npz and geometry_multilap_full/quality.jsonl, original motion export, historical GNSS CSV/config/URDF candidate and AS left/right boundary CSV. Identity hashes are in manifest.json. No raw production source or private geometry is distributed here.

Source ROS2 Jazzy and the existing vehicle workspace for vectornav_msgs, tf2_msgs and xacro. Set PYTHONPATH to include this repository's src without removing ROS paths, OPENBLAS_NUM_THREADS=1 and PYTHONDONTWRITEBYTECODE=1. Python dependencies are NumPy/SciPy/PyYAML/matplotlib plus GeographicLib CartConvert and ROS deserializers. No new package installation or GNSS geodesy implementation is required.

Create a private bindings JSON with keys: bag, motion, run, as_npz, gnss, historical_gnss_config, historical_urdf, left, right, quality, private. Values are authorized local paths. `private` must be a persistent ignored directory. Historical candidate choice must be explicit; do not silently substitute current calibration.

```
python3 experiments/phase_at_gnss_footprint_corners/reproduce.py "$AT_BINDINGS"
# After raw extraction, an actual reproducibility rerun used:
python3 experiments/phase_at_gnss_footprint_corners/reproduce.py "$AT_BINDINGS" --reuse-raw
python3 experiments/phase_at_gnss_footprint_corners/summary_figures.py
PYTHONPATH=src python3 -m unittest discover -s tests
```

extract_raw_reference.py reads the original bag read-only: GPS1/GPS2, raw attitude and static TF. bind_static.py compares shared recorded/historical transforms, labels missing links and extracts only the historical visual box into private data. analyze.py performs typeA/typeB time experiments, exact native-time prior/NDT/posterior decomposition, interpolation validity, own-pose 3D footprint projection and full segment/polygon checks. supplement.py joins existing AS quality by Header and validates boundary topology. No estimator updates are executed.

Outputs: public aggregate statistics.json, corner_quality.json, static_audit.json and aggregate figures; private raw_reference/reference_bound JSON, visual_outline, causal.npz with native state/gain/covariance/reference arrays, per-frame polygon records, rear_axle_overlay, time_sensitivity, corner1/3/9_decomposition and yaw_innovation_bias, corner3/9_visual_polygon plots. Full private reproduction logs and bindings are ignored. Tyre outlines are not generated when measurements are unavailable.

Frame association: native LiDAR Header; same frozen AS corner IDs/tangents; GPS gap≤.600001s, attitude gap≤.05s, oldEKF gap≤.1s, historical curve adjacent gap≤.100001s. No extrapolated sample is scored. Delay hypotheses0/75/150ms are fixed sensitivity cases, not calibrated options. Native-time results are not AS's published-state-time evaluation. Original AR/AS transaction SHA identities must match before comparison.

Reported qualifications: conversion equivalence VERIFIED; actual GNSS/track/visual-body accuracy CONDITIONAL; tyre/safety/independent ground truth NOT_VALIDATED. See FAILURES before interpreting zero modeled crossing counts.
