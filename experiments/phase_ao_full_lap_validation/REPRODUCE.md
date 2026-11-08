# Reproduction
Public code never downloads private bags, maps, extrinsics or production binaries. Use independently authorized originals identified by the manifest. Original production checkout remains read only.

Set shell variables `RESEARCH`, `PRIVATE`, `BAG35`, `BAG44`, `BAG7`, `BAG8`, `R2`, `FROZEN` to the authorized checkout, empty ignored/private result directory, original bag locations, exported fixture and unchanged Phase V/AL frozen configuration. Source ROS Jazzy and the workspace that provides `vectornav_msgs`/other required message type support. Do not replace ROS's existing PYTHONPATH.

```bash
cd "$RESEARCH"
export PYTHONPATH="$RESEARCH/src:${PYTHONPATH:-}"
export OPENBLAS_NUM_THREADS=1
export IESKF_CONFIG="$FROZEN"
python3 experiments/phase_ao_full_lap_validation/extract_bag.py --bag "$BAG35" --output "$PRIVATE/loop35.json"
python3 experiments/phase_ao_full_lap_validation/extract_bag.py --bag "$BAG44" --output "$PRIVATE/loop44.json"
python3 experiments/phase_ao_full_lap_validation/extract_mcap.py --bag "$BAG7" --output "$PRIVATE/original7.json"
python3 experiments/phase_ao_full_lap_validation/extract_mcap.py --bag "$BAG8" --output "$PRIVATE/original8.json"
python3 experiments/phase_ao_full_lap_validation/timing_comparison.py --dataset "$R2" --output "$PRIVATE/timing.json"
python3 experiments/phase_ao_full_lap_validation/run_local.py --input "$PRIVATE/original7.json" --private-output "$PRIVATE/local7" --output "$PRIVATE/local7_summary.json"
python3 experiments/phase_ao_full_lap_validation/compare_local_timing.py --input "$PRIVATE/original7.json" --local "$PRIVATE/local7" --output "$PRIVATE/local_timing.json"
python3 experiments/phase_ao_full_lap_validation/run_global.py --input "$PRIVATE/original7.json" --local "$PRIVATE/local7" --output "$PRIVATE/global7_summary.json"
python3 -m unittest discover -s tests
```

Repeat local/global commands for each extracted input, retaining the entire selected recording and any failure. No repeated 14.7 s segment or post-failure restart is used. A full raw extraction is not a completed estimator lap. Raw outputs include complete covariance and remain private; public reports contain only aggregate results. `local_stamps_ns.npy` is the exact int64 clock; the float timestamp column in `local_trace.npy` is diagnostic only and must not be used for nanosecond identity. The valid-row count is saved separately because the bounded output allocation includes unfilled rows after a failure. The initial state in timing comparison is shared by all three modes, not proof of online startup.

Local whole-stream code stops at existing IMU-gap gates; it does not invent unavailable samples, restart on a more favorable interval, or fuse zero-Header wheel messages. Header sorting, recorder order and checkpoint replay are separately reported. ROS callback order and real executor backlog are unavailable, so no online-equivalence PASS is reproducible from these inputs.
