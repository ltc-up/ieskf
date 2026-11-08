# Reproduction
Checkout the AP commit and verify AO freeze3667c9c6da84b82c1d9e4676b38436ed9f8f6da1. Use separately authorized private originals identified by AO/AP manifests. Full maps/bags/trajectories/extrinsics and production binaries are excluded from Git.

Set `RESEARCH`, `PRIVATE`, `ORIGINAL_DATA`, `FROZEN` to the checkout, AO private outputs, original authorized historical research data and unchanged configuration. Reproduce AO extraction/local outputs first; retain all failures. Then:

```bash
cd "$RESEARCH"
export PYTHONPATH="$RESEARCH/src:${PYTHONPATH:-}"
export OPENBLAS_NUM_THREADS=1
export IESKF_CONFIG="$FROZEN"
python3 experiments/phase_ap_robustness_validation/run_faults.py --input "$PRIVATE/loop44.json" --local "$PRIVATE/local44" --output "$PRIVATE/AP_faults.json"
python3 experiments/phase_ap_robustness_validation/run_geometry_fault.py --data "$ORIGINAL_DATA" --output "$PRIVATE/AP_geometry.json"
python3 -m unittest discover -s tests
```

The geometry runner uses original Target/Source and stored AM multistart records. It writes no map and launches no new NDT fitter. Full local history uses read-only memmap and exact int64 timestamp arrays. Standard-output/run errors must be retained. The fixed first shadow probe is expected to lack startup odom bracket on this real fixture; do not move it to a favorable frame. A first result without reason instrumentation is retained separately.

Fault conditions are declared in TEST_PLAN and SCENARIOS; no parameters are optimized. Whole-record controls and injected cases are all retained, even though trusted coverage is0. Source truncation/end-of-stream remains in AO results. No script grants GNSS qualification or reconstructs an independent truth trajectory. ROS2 online callbacks/deadlines, partial-LiDAR preprocessing and actual new fault-time CUDA matching require additional validated inputs/engineering and are not represented by these commands.
