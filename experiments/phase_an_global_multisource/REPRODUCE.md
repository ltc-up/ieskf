# Reproduce AN

Public unit tests and SYNTHETIC recovery fixture:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
PYTHONPATH=src python3 experiments/phase_an_global_multisource/run_recovery.py
```

Real run requires original AA/AJ data and private AL/AM outputs matching manifest. Install/source ROS2 Jazzy plus authorized vectornav_msgs type support. Set IESKF_BAG to original bag and IESKF_DATA_ROOT to original historical data; no paths/config values are embedded. Then:

```bash
PYTHONPATH=src:$PYTHONPATH python3 experiments/phase_an_global_multisource/audit_gnss.py --data-root "$IESKF_DATA_ROOT" --bag "$IESKF_BAG" --private-output data/phase_an_global_multisource --public-output experiments/phase_an_global_multisource
OPENBLAS_NUM_THREADS=1 PYTHONPATH=src python3 experiments/phase_an_global_multisource/run_comparison.py --data-root "$IESKF_DATA_ROOT" --work-data data --private-output data/phase_an_global_multisource --public-output experiments/phase_an_global_multisource
ROS_LOG_DIR=/tmp/phase_an_ros_logs ROS_DOMAIN_ID=174 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONPATH=src:$PYTHONPATH python3 -m ndt_global.ros_global --events data/phase_an_global_multisource/ros_events.json --report experiments/phase_an_global_multisource/ros_results.json
```

No production imports are needed by core modules. ROS bag extraction uses installed type support only; copying private vehicle/SDK source is unnecessary. Unqualified position records stay unqualified; do not edit flags to make a real replay accept. Local inputs are immutable AL outputs; AN does not rerun matching or feed another branch future results as local prediction. Exact code revision is enclosing AN commit (`git log -1 -- src/ndt_global/global_manager.py`).20ms scheduler delay is emulated,not measured live latency. Public aggregate outputs are reviewed; complete positions/P/trajectories/CDR records remain ignored.
